/****************************************************************************
 * arch/risc-v/src/vesp/vesp_serial.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/serial/serial.h>
#include <nuttx/arch.h>
#include <nuttx/bits.h>

#include "riscv_internal.h"

#include "hardware/vesp_uart.h"
#include "chip.h"

#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define CONSOLE_DEV g_uart0port

#ifndef CONFIG_UART0_RXBUFSIZE
#  define CONFIG_UART0_RXBUFSIZE 256
#endif

#ifndef CONFIG_UART0_TXBUFSIZE
#  define CONFIG_UART0_TXBUFSIZE 256
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

 typedef enum
{
  UART_NO_PARITY,
  UART_ODD_PARITY,
  UART_EVEN_PARITY,
  UART_CONST_0_PARITY,
  UART_CONST_1_PARITY
} uart_parity_type_t;

struct uart_config_s
{
  uint8_t            idx;       /* UART idx */
  uint32_t           baud;      /* Configured baud */
  uint8_t            data_bits; /* Number of bits */
  uint8_t            stop_bits; /* Stop bits */
  uart_parity_type_t parity;    /* Parity selection */
};

struct vesp_uart_s
{
  uint8_t              rx_irq; /* IRQ from UARTs RX queue */
  uint8_t              tx_irq; /* IRQ from UARTs TX queue */
  struct uart_config_s config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  vesp_setup(struct uart_dev_s *dev);
static void vesp_shutdown(struct uart_dev_s *dev);
static int  vesp_attach(struct uart_dev_s *dev);
static void vesp_detach(struct uart_dev_s *dev);
static int  vesp_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  vesp_receive(struct uart_dev_s *dev, unsigned int *status);
static void vesp_rxint(struct uart_dev_s *dev, bool enable);
static bool vesp_rxavailable(struct uart_dev_s *dev);
static void vesp_send(struct uart_dev_s *dev, int ch);
static void vesp_txint(struct uart_dev_s *dev, bool enable);
static bool vesp_txready(struct uart_dev_s *dev);
static bool vesp_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

 /* UART0 I/O buffers */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

/* UART0 operations */

static const struct uart_ops_s g_uart_ops =
{
  .setup       = vesp_setup,
  .shutdown    = vesp_shutdown,
  .attach      = vesp_attach,
  .detach      = vesp_detach,
  .ioctl       = vesp_ioctl,
  .receive     = vesp_receive,
  .rxint       = vesp_rxint,
  .rxavailable = vesp_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send        = vesp_send,
  .txint       = vesp_txint,
  .txready     = vesp_txready,
  .txempty     = vesp_txempty,
};

/* UART0 private info */

static struct vesp_uart_s g_uart0priv =
{
  .rx_irq = VESP_UART0_RX_IRQ + RISCV_IRQ_ASYNC,
  .tx_irq = VESP_UART0_TX_IRQ + RISCV_IRQ_ASYNC,
  .config =
  {
    .idx       = 0,
    .baud      = VESP_UART0_BAUD,
    .data_bits = VESP_UART0_DATA_BITS,
    .stop_bits = VESP_UART0_STOP_BITS,
    .parity    = VESP_UART0_PARITY_BITS
  },
};

/* UART0 device structure */

static uart_dev_t g_uart0port =
{
  .isconsole = 1,
  .recv =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = (void *)&g_uart0priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* UART0 interrupt handlers */

/****************************************************************************
 * Name: __vesp_rx_irq_handler
 *
 * Description:
 *   This is the UART RX interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int __vesp_rx_irq_handler(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  uart_recvchars(dev);

  return OK;
}

/****************************************************************************
 * Name: __vesp_tx_irq_handler
 *
 * Description:
 *   This is the UART TX interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int __vesp_tx_irq_handler(int irq, void *context, void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
  uart_xmitchars(dev);

  return OK;
}

/* UART operations */

/****************************************************************************
 * Name: vesp_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/
static int vesp_setup(struct uart_dev_s *dev)
{
  struct vesp_uart_s *priv = (struct vesp_uart_s *)dev->priv;
  uint32_t setup_reg = 0;

  /* Setup parity */

  switch (priv->config.parity)
  {
    case UART_NO_PARITY:
      setup_reg |= FIELD_PREP(VESP_UART_SETUP_PARITY_ON, 0x0);
    break;
      
    case UART_ODD_PARITY:
      setup_reg |= FIELD_PREP(VESP_UART_SETUP_PARITY_ON, 0x1) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_CONST, 0x0) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_TYPE, 0x0);
    break;

    case UART_EVEN_PARITY:
      setup_reg |= FIELD_PREP(VESP_UART_SETUP_PARITY_ON, 0x1) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_CONST, 0x0) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_TYPE, 0x1);
    break;

    case UART_CONST_0_PARITY:
      setup_reg |= FIELD_PREP(VESP_UART_SETUP_PARITY_ON, 0x1) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_CONST, 0x1) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_TYPE, 0x0);
    break;

    case UART_CONST_1_PARITY:
      setup_reg |= FIELD_PREP(VESP_UART_SETUP_PARITY_ON, 0x1) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_CONST, 0x1) |
                   FIELD_PREP(VESP_UART_SETUP_PARITY_TYPE, 0x1);
    break;
  }

  /* Setup stop bits (-1 because they are encoded into a bit) */

  setup_reg |= FIELD_PREP(VESP_UART_SETUP_STOP_BITS,
                          priv->config.stop_bits - 1);
  
  /* Setup baudrate */

  setup_reg |= FIELD_PREP(VESP_UART_SETUP_CLK_PER_BAUD,
                          VESP_SYSTEM_CLK_HZ / priv->config.baud);

  /* Write setup */

  putreg32(setup_reg, VESP_UART0_SETUP);

  return OK;
}

/****************************************************************************
 * Name: vesp_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/
static void vesp_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: vesp_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/
static int vesp_attach(struct uart_dev_s *dev)
{
  struct vesp_uart_s *priv = (struct vesp_uart_s *)dev->priv;

  int ret = irq_attach(priv->rx_irq, __vesp_rx_irq_handler, (void *)dev);
  if (ret != OK)
  {
    irqerr("IRQERR: Failed to attach rx_irq handler\n");
    return ret;
  }
  irqinfo("IRQINFO: Attached rx_irq handler\n");

  ret = irq_attach(priv->tx_irq, __vesp_tx_irq_handler, (void *)dev);
#ifdef CONFIG_DEBUG_IRQ_ERR
  if (ret != OK)
    irqerr("IRQERR: Failed to attach tx_irq handler\n");
  else
#endif
#ifdef CONFIG_DEBUG_IRQ_INFO
    irqinfo("IRQINFO: Attached tx_irq handler\n");
#endif

  return ret;
}

/****************************************************************************
 * Name: vesp_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/
static void vesp_detach(struct uart_dev_s *dev)
{
  struct vesp_uart_s *priv = (struct vesp_uart_s *)dev->priv;

  irq_detach(priv->rx_irq);
  irq_detach(priv->tx_irq);
}

/****************************************************************************
 * Name: vesp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/
static int vesp_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: vesp_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/
static int vesp_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint32_t rxdata = getreg32(VESP_UART0_RXDATA);

  /* Check errors */

  if ((rxdata & VESP_UART_RXDATA_RX_ERROR) || 
      (rxdata & VESP_UART_RXDATA_RX_FRAME_ERROR) || 
      (rxdata & VESP_UART_RXDATA_RX_PARITY_ERROR))
  {
    // TODO: report right error
    *status = 1;
    return 0;
  }
  
  if ((rxdata & VESP_UART_RXDATA_RX_BREAK))
  {
    // TODO: report right error
    *status = 2;
    return 0;
  }  
  
  if (!(rxdata & VESP_UART_RXDATA_RX_VALID))
  {
    /* Valid bit is inversed. */

    *status = OK;
    return FIELD_GET(VESP_UART_RXDATA_RWORD, rxdata);
  }

  // TODO: report right error
  *status = 3;
  return 0;
}

/****************************************************************************
 * Name: vesp_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/
static void vesp_rxint(struct uart_dev_s *dev, bool enable)
{
  struct vesp_uart_s *priv = (struct vesp_uart_s *)dev->priv;
  
  if (enable)
    up_enable_irq(priv->rx_irq);
  else
    up_disable_irq(priv->rx_irq);
}

/****************************************************************************
 * Name: vesp_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/
static bool vesp_rxavailable(struct uart_dev_s *dev)
{
  return getreg32(VESP_UART0_FIFO) & VESP_UART_FIFO_RX_FIFO_READY;
}

/****************************************************************************
 * Name: vesp_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/
static void vesp_send(struct uart_dev_s *dev, int ch)
{
  /* Wait until there is space for a byte in TX fifo */
  while(!(getreg32(VESP_UART0_FIFO) & VESP_UART_FIFO_TX_FIFO_READY));

  uint32_t send_data = FIELD_PREP(VESP_UART_TXDATA_TWORD, (uint8_t)ch);
  putreg32(send_data, VESP_UART0_TXDATA);
}

/****************************************************************************
 * Name: vesp_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/
static void vesp_txint(struct uart_dev_s *dev, bool enable)
{
  struct vesp_uart_s *priv = (struct vesp_uart_s *)dev->priv;
  
  if (enable)
    up_enable_irq(priv->tx_irq);
  else
    up_disable_irq(priv->tx_irq);
}

/****************************************************************************
 * Name: vesp_txready
 *
 * Description:
 *   Return true if the transmit data register is not full
 *
 ****************************************************************************/
static bool vesp_txready(struct uart_dev_s *dev)
{
  return getreg32(VESP_UART0_FIFO) & VESP_UART_FIFO_TX_FIFO_READY;
}

/****************************************************************************
 * Name: vesp_txempty
 *
 * Description:
 *   Return true if the transmit data register is empty
 *
 ****************************************************************************/
static bool vesp_txempty(struct uart_dev_s *dev)
{
  uint32_t fifo_status = getreg32(VESP_UART0_FIFO);
  
  return FIELD_GET(VESP_UART_FIFO_TX_FIFO_FREE, fifo_status) ==
         (CONFIG_UART0_TXBUFSIZE - 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vesp_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/
void vesp_earlyserialinit(void)
{
  /* Initialize CONSOLE_DEV */
  CONSOLE_DEV.isconsole = true;

  vesp_setup(&CONSOLE_DEV);
}

/****************************************************************************
 * Name: vesp_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/
void vesp_serialinit(void)
{
  /* Initialize CONSOLE_DEV */
  CONSOLE_DEV.open_count = 0;
  CONSOLE_DEV.isconsole = true;

  /* Register the console */
  DEBUGVERIFY(uart_register("/dev/console", &CONSOLE_DEV));
}

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/
void riscv_earlyserialinit(void)
{
  vesp_earlyserialinit();
}

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/
void riscv_serialinit(void)
{
  vesp_serialinit();
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/
int up_putc(int ch)
{
  irqstate_t flags = enter_critical_section();

  /* Check for LF */
  if (ch == '\n')
  {
    /* Add CR */
    
    /* Wait until there is space for a byte in TX fifo */
    while(!(getreg32(VESP_UART0_FIFO) & VESP_UART_FIFO_TX_FIFO_READY));
    putreg32('\r', VESP_UART0_TXDATA);
  }

  /* Wait until there is space for a byte in TX fifo */
  while(!(getreg32(VESP_UART0_FIFO) & VESP_UART_FIFO_TX_FIFO_READY));
  putreg32(ch, VESP_UART0_TXDATA);

  leave_critical_section(flags);
  return ch;
}