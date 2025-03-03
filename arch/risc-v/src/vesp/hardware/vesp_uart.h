/****************************************************************************
 * arch/risc-v/src/vesp/hardware/vesp_uart.h
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

#ifndef __ARCH_RISCV_SRC_VESP_HARDWARE_VESP_UART_H
#define __ARCH_RISCV_SRC_VESP_HARDWARE_VESP_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

 #include <nuttx/bits.h>

/****************************************************************************
 * Pre-preprocessor Definitions
 ****************************************************************************/

#define VESP_UART0_BASE_ADDR  0xE0000020 

/* VESP_UART_REGISTERS memory map */
enum VESP_UART_REGS
{
  VESP_UART_SETUP  = 0x0,
  VESP_UART_FIFO   = 0x4,
  VESP_UART_RXDATA = 0x8,
  VESP_UART_TXDATA = 0xc
};

#define VESP_UART0_SETUP   (VESP_UART0_BASE_ADDR + VESP_UART_SETUP)
#define VESP_UART0_FIFO    (VESP_UART0_BASE_ADDR + VESP_UART_FIFO)
#define VESP_UART0_RXDATA  (VESP_UART0_BASE_ADDR + VESP_UART_RXDATA)
#define VESP_UART0_TXDATA  (VESP_UART0_BASE_ADDR + VESP_UART_TXDATA)

/* UART_SETUP registers */
#define VESP_UART_SETUP_BITS_PER_WORD  GENMASK(29, 28)
#define VESP_UART_SETUP_STOP_BITS      BIT(27)
#define VESP_UART_SETUP_PARITY_ON      BIT(26)
#define VESP_UART_SETUP_PARITY_CONST   BIT(25)
#define VESP_UART_SETUP_PARITY_TYPE    BIT(24)
#define VESP_UART_SETUP_CLK_PER_BAUD   GENMASK(23, 0)

/* UART_FIFO registers */
#define VESP_UART_FIFO_TX_FIFO_LEN         GENMASK(31, 28)
#define VESP_UART_FIFO_TX_FIFO_FREE        GENMASK(27, 18)
#define VESP_UART_FIFO_TX_FIFO_HIGH_ORDER  BIT(17)
#define VESP_UART_FIFO_TX_FIFO_READY       BIT(16)
#define VESP_UART_FIFO_RX_FIFO_LEN         GENMASK(15, 12)
#define VESP_UART_FIFO_RX_FIFO_OCC         GENMASK(11, 2)
#define VESP_UART_FIFO_RX_FIFO_HIGH_ORDER  BIT(1)
#define VESP_UART_FIFO_RX_FIFO_READY       BIT(0)

/* UART_RXDATA registers */
#define VESP_UART_RXDATA_RX_ERROR         BIT(12)
#define VESP_UART_RXDATA_RX_BREAK         BIT(11)
#define VESP_UART_RXDATA_RX_FRAME_ERROR   BIT(10)
#define VESP_UART_RXDATA_RX_PARITY_ERROR  BIT(9)
#define VESP_UART_RXDATA_RX_VALID         BIT(8)
#define VESP_UART_RXDATA_RWORD            GENMASK(7, 0)

/* UART_TXDATA registers */
#define VESP_UART_TXDATA_TX_HALF_FULL          BIT(14)
#define VESP_UART_TXDATA_TX_NOT_FULL           BIT(13)
#define VESP_UART_TXDATA_TX_OVERFLOW_RECORDED  BIT(12)
#define VESP_UART_TXDATA_TX_BREAK              BIT(9)
#define VESP_UART_TXDATA_TX_BUSY               BIT(8)
#define VESP_UART_TXDATA_TWORD                 GENMASK(7, 0)

/* IRQ numbers assignments */
#define VESP_UART0_RX_IRQ  16
#define VESP_UART0_TX_IRQ  17

/* UART0 default configuration */
#define VESP_UART0_BAUD         115200
#define VESP_UART0_DATA_BITS    8
#define VESP_UART0_STOP_BITS    1
#define VESP_UART0_PARITY_BITS  0

#endif /* __ARCH_RISCV_SRC_VESP_HARDWARE_VESP_UART_H */