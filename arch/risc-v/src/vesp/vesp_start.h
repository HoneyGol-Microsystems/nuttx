/****************************************************************************
 * arch/risc-v/src/vesp/vesp_start.h
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

#ifndef __ARCH_RISC_V_SRC_VESP_VESP_START_H
#define __ARCH_RISC_V_SRC_VESP_VESP_START_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: vesp_board_initialize
 *
 * Description:
 *   This initialization function is called in __vesp_start(). It should
 *   not be called from common OS logic. It should initialize low-level
 *   things like GPIO, power settings, DRAM initialization, etc.
 *   The OS has not been initialized at this point, so you cannot allocate
 *   memory or initialize device drivers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void vesp_board_initialize(void);

#endif /* __ARCH_RISC_V_SRC_VESP_VESP_START_H */
  