/****************************************************************************
 * configs/sa-1xx/src/up_dins.c
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

#ifdef CONFIG_ARCH_DINS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Pin configuration for each SA-1xx din.  This array is indexed by
 * the DIN_* definitions in board.h
 */

static const uint16_t g_dins[NUM_DINS] =
{
	GPIO_DIO_IN0,
	GPIO_DIO_IN1,
	GPIO_DIO_IN2,
	GPIO_DIO_IN3
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dininit
 *
 * Description:
 *   up_dininit() must be called to initialize din resources.  After
 *   that, up_dins() may be called to collect the current state of all
 *   dins or up_irqdin() may be called to register din interrupt
 *   handlers.
 *
 ****************************************************************************/

void up_dininit(void)
{
    int i;

    /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are 
     * configured for all pins.
     */
    for (i = 0; i < NUM_DINS; i++)
        stm32_configgpio(g_dins[i]);
}

/****************************************************************************
 * Name: up_dins
 ****************************************************************************/

uint8_t up_dins(void)
{
    uint8_t ret = 0;
    int i;

    /* Check that state of each key */
    for (i = 0; i < NUM_DINS; i++) {
        /* A LOW value means that the DIN is asserted. */
        bool off = stm32_gpioread(g_dins[i]);

        /* Accumulate the set of asserted DINs */
        if (!off)
            ret |= (1 << i);
    }

    return ret;
}

/************************************************************************************
 * Din support.
 *
 * Description:
 *   up_dininit() must be called to initialize din resources.  After
 *   that, up_dins() may be called to collect the current state of all
 *   dins or up_irqdin() may be called to register din interrupt
 *   handlers.
 *
 *   After up_dininit() has been called, up_dins() may be called to
 *   collect the state of all dins.  up_dins() returns an 8-bit bit set
 *   with each bit associated with a din.  See the DIN_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   up_irqdin() may be called to register an interrupt handler that will
 *   be called when a din is depressed or released.  The ID value is a
 *   din enumeration value that uniquely identifies a din resource. See the
 *   DIN_* definitions in board.h for the meaning of enumeration
 *   value.  The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_IRQDINS
xcpt_t up_irqdin(int id, xcpt_t irqhandler)
{
    xcpt_t oldhandler = NULL;

    /* The following should be atomic */
    if (id >= MIN_IRQDIN && id <= MAX_IRQDIN)
        oldhandler = stm32_gpiosetevent(g_dins[id], true, true, true, irqhandler);

    return oldhandler;
}
#endif
#endif /* CONFIG_ARCH_DINS */
