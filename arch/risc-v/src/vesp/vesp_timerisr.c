/****************************************************************************
 * arch/risc-v/src/vesp/vesp_timerisr.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/timers/arch_alarm.h>

#include "riscv_internal.h"
#include "riscv_mtimer.h"

#include "hardware/vesp_timer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/
void up_timer_initialize(void)
{
  uint64_t timer_freq;

  timer_freq  = getreg32(VESP_TIMER_MTIMETICK);
  timer_freq |= ((uint64_t)(getreg32(VESP_TIMER_MTIMETICK + 4)) << 32);

  struct oneshot_lowerhalf_s *lower = riscv_mtimer_initialize(
    VESP_TIMER_MTIME, VESP_TIMER_MTIMECMP,
    RISCV_IRQ_MTIMER, timer_freq);

  DEBUGASSERT(lower);

  up_alarm_set_lowerhalf(lower);
}
