/****************************************************************************
 * arch/risc-v/src/vesp/hardware/vesp_gpio.h
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

#ifndef __ARCH_RISCV_SRC_VESP_HARDWARE_VESP_GPIO_H
#define __ARCH_RISCV_SRC_VESP_HARDWARE_VESP_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-preprocessor Definitions
 ****************************************************************************/

#define VESP_GPIO0_BASE_ADDR  0xE0000010

/* VESP_GPIO_REGISTERS memory map */
enum VESP_GPIO_REGS
{
  VESP_GPIO_DIR = 0x0,
  VESP_GPIO_WR  = 0x4,
  VESP_GPIO_RD  = 0x8
};

#define VESP_GPIO0_WR   (VESP_GPIO0_BASE_ADDR + VESP_GPIO_WR)
#define VESP_GPIO0_RD   (VESP_GPIO0_BASE_ADDR + VESP_GPIO_RD)
#define VESP_GPIO0_DIR  (VESP_GPIO0_BASE_ADDR + VESP_GPIO_DIR)

#endif /* __ARCH_RISCV_SRC_VESP_HARDWARE_VESP_GPIO_H */