/****************************************************************************
 * config/sa-1xx/src/up_nsh.c
 * arch/arm/src/board/up_nsh.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#if defined(CONFIG_STM32_WWDG) || defined(CONFIG_STM32_IWDG)
#  include <nuttx/watchdog.h>
#endif

#if defined(CONFIG_GRAN) && defined(CONFIG_GRAN_SINGLE)
#  include <nuttx/gran.h>
#endif

#include "stm32.h"
#include "sa1xx-internal.h"

#ifdef CONFIG_STM32_SDIO
extern int up_sdioinitialize(void);
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#define HAVE_USBDEV   1
#define HAVE_USBHOST  1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device is USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_GRAN) && defined(CONFIG_GRAN_SINGLE)
#define CCM_SIZE (64 * 1024)  // 64KiB
static char g_gran_heap[CCM_SIZE] __attribute__ ((section(".ccm")));
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
#if defined(CONFIG_STM32_SPI2) || defined(HAVE_MMCSD) || defined(HAVE_USBHOST) || \
    defined(CONFIG_I2C_LM75) || defined(CONFIG_STM32_WWDG) || defined(CONFIG_STM32_IWDG) || \
    defined(CONFIG_GRAN)
    int ret;
#endif

#if defined(CONFIG_GRAN) && defined(CONFIG_GRAN_SINGLE)
    /* Initialize GRAN HEAP, size: 64bytes, align: 8bytes */
    memset(g_gran_heap, 0, CCM_SIZE);
    ret = gran_initialize(g_gran_heap, CCM_SIZE, 6, 3);

    if (ret != OK)
        message("nsh_archinitialize: Failed to initialize GRAN allocator\n");
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_LM75) && defined(CONFIG_STM32_I2C2)
    /* Register the temperature sensor */
    ret = stm32_lm75initialize("/dev/temp0");
#endif

#if defined(CONFIG_STM32_WWDG) || defined(CONFIG_STM32_IWDG)
    ret = up_wdginitialize();
#endif

    ret = stm32_at24xxinitialize();

#ifdef CONFIG_STM32_SPI2
    ret = sa1xx_spiflash_initialize();
#endif

    /* Mount the SDIO-based MMC/SD block driver */

#ifdef CONFIG_STM32_SDIO
    ret = up_sdioinitialize();
#endif

#ifdef HAVE_USBHOST
    /* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
     * will monitor for USB connection and disconnection events.
     */
    ret = stm32_usbhost_initialize();

    if (ret != OK) {
        message("nsh_archinitialize: Failed to initialize USB host: %d\n", ret);
        return ret;
    }
#endif

    return OK;
}
