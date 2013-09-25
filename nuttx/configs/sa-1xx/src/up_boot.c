/************************************************************************************
 * configs/sa-1xx/src/up_boot.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "stm32_pwr.h"
#include "up_arch.h"
#include "nvic.h"
#include "sa1xx-internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define FLASH_START ((void *) 0x08020000)
#define FLASH_SIZE  (0x20000 * 7)
#define FLASH_END   (FLASH_START + FLASH_SIZE)
#define SRAM_START  0x20000000
#define SRAM_END    (SRAM_START + 112 * 1024)

typedef void (*app_entry)(void);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifdef CONFIG_SA1XX_BOOTLOADER
static void stm32_jump_to_app(void *vec);
#endif

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void)
{
    uint32_t regval;

#ifdef CONFIG_SA1XX_BOOTLOADER
    stm32_jump_to_app(FLASH_START);
#endif
    /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak function
     * stm32_spiinitialize() has been brought into the link.
     */
#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)
    if (stm32_spiinitialize)
        stm32_spiinitialize();
#endif

    /* Initialize USB if the 1) OTG FS controller is in the configuration and 2)
     * disabled, and 3) the weak function stm32_usbinitialize() has been brought 
     * the weak function stm32_usbinitialize() has been brought into the build.
     * Presumeably either CONFIG_USBDEV or CONFIG_USBHOST is also selected.
     */
#ifdef CONFIG_STM32_OTGFS
    if (stm32_usbinitialize)
        stm32_usbinitialize();
#endif

#ifdef CONFIG_DEBUG_JTAG
    regval  = getreg32(STM32_DBGMCU_CR);
    regval |= DBGMCU_CR_SLEEP;
    putreg32(regval, STM32_DBGMCU_CR);
#endif

    /* Configure on-board LEDs if LED support has been selected. */
#ifdef CONFIG_ARCH_LEDS
    up_ledinit();
#endif

    stm32_pwr_enablebkp();
}

#ifdef CONFIG_SA1XX_BOOTLOADER
static void set_MSP(uint32_t stack)
{
	__asm__ volatile("MSR msp, %0\n\t" : : "r" (stack));
} __attribute__ ((naked))

static void stm32_jump_to_app(void *vec)
{
	app_entry func;
	uint32_t entry, stack;

	stack = *((uint32_t *) vec);
	entry = *(((uint32_t *) vec) + 1);

	if ((stack >= SRAM_START && stack < SRAM_END) &&
		(entry >= FLASH_START && entry < FLASH_END)) {
		info("jump to user application...\n");

		func = (app_entry) entry;
		irqsave();
		set_MSP(stack);
		putreg32((uint32_t) vec, NVIC_VECTAB);
		func();
		for(;;);
	}

	info("bootloader.\n");
}
#endif
