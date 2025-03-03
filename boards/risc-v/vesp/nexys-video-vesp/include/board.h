/****************************************************************************
 * boards/risc-v/vesp/nexys-video-vesp/include/board.h
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

#ifndef __BOARDS_RISCV_VESP_NEXYS_VIDEO_VESP_INCLUDE_BOARD_H
#define __BOARDS_RISCV_VESP_NEXYS_VIDEO_VESP_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/bits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

// Nexys Video LEDs
#define BOARD_LED_CNT       8
#define BOARD_ALL_LED_MASK  GENMASK(BOARD_LED_CNT-1, 0)

// Nexys Video OLED control (mapped to GPIO after LEDs)
#define BOARD_OLED_VDD_N_MASK     BIT(BOARD_LED_CNT)
#define BOARD_OLED_RST_N_MASK     BOARD_OLED_VDD_N_MASK << 1
#define BOARD_OLED_VBAT_N_MASK    BOARD_OLED_RST_N_MASK << 1
#define BOARD_OLED_DATA_CMD_MASK  BOARD_OLED_VBAT_N_MASK << 1

// NuttX Event LEDs
#define LED_CPU             BIT(0)
// #define LED_STARTED         BIT(1)
#define LED_HEAPALLOCATE    BIT(1)
#define LED_IRQSENABLED     BIT(2)
#define LED_STACKCREATED    BIT(3)
#define LED_INIRQ           BIT(4)
#define LED_SIGNAL          BIT(5)
#define LED_ASSERTION       BIT(6)
#define LED_PANIC           BIT(7)

#endif /* __BOARDS_RISCV_VESP_NEXYS_VIDEO_VESP_INCLUDE_BOARD_H */