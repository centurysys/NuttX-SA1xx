/****************************************************************************
 * configs/sa-1xx/src/up_dipsw.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>
#include "sa1xx-internal.h"

#ifdef CONFIG_ARCH_DIPSW

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Pin configuration for SA-1xx dipsw.  This array is indexed by
 * the DIPSW_* definitions in board.h
 */

static const uint16_t g_dipsw[NUM_DIPSW] =
{
    GPIO_DIPSW0,
    GPIO_DIPSW1,
    GPIO_DIPSW2,
    GPIO_DIPSW3
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dipswinit
 ****************************************************************************/

void up_dipswinit(void)
{
    int i;

    /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are 
     * configured for all pins.
     */
    for (i = 0; i < NUM_DIPSW; i++)
        stm32_configgpio(g_dipsw[i]);
}

/****************************************************************************
 * Name: up_dipsw
 ****************************************************************************/

uint8_t up_dipsw(void)
{
    uint8_t ret = 0;
    int i;

    /* Check that state of each switch */
    for (i = 0; i < NUM_DIPSW; i++) {
        /* A HIGH value means that the switch is on.
         */
        bool sw_on = stm32_gpioread(g_dipsw[i]);

        /* Accumulate the set of depressed keys */
        if (sw_on)
            ret |= (1 << i);
    }

    return ret;
}

#endif /* CONFIG_ARCH_DIPSW */
