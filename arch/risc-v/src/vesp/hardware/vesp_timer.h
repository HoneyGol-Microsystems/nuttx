/****************************************************************************
 * arch/risc-v/src/vesp/hardware/vesp_timer.h
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

#define VESP_TIMER_BASE_ADDR  0xE0000060

/* VESP_TIMER_REGISTERS memory map */
enum VESP_TIMER_REGS
{
  VESP_MTIME_REG     = 0x0,
  VESP_MTIMECMP_REG  = 0x8,
  VESP_MTIMETICK_REG = 0x10
};

#define VESP_TIMER_MTIME      (VESP_TIMER_BASE_ADDR + VESP_MTIME_REG)
#define VESP_TIMER_MTIMECMP   (VESP_TIMER_BASE_ADDR + VESP_MTIMECMP_REG)
#define VESP_TIMER_MTIMETICK  (VESP_TIMER_BASE_ADDR + VESP_MTIMETICK_REG)

#endif /* __ARCH_RISCV_SRC_VESP_HARDWARE_VESP_GPIO_H */