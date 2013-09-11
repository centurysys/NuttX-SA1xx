/************************************************************************************
 * configs/sa-1xx/src/up_sdio.c
 * arch/arm/src/board/up_sdio.c
 *
 *   Copyright (C) 2012 Century Systems. All rights reserved.
 *   Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
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

#include <arch/board/board.h>
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_STM32_SDIO
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#endif

#include "stm32.h"
#include "sa1xx-internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#define HAVE_MMCSD    1

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO support
 * is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_MMCSD
#endif

/* Default MMC/SD minor number */

#ifdef HAVE_MMCSD
#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif

/* Default MMC/SD SLOT number */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error "Only one MMC/SD slot"
#    undef CONFIG_NSH_MMCSDSLOTNO
#  endif

#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
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

#ifdef HAVE_MMCSD
static struct sdio_dev_s *sdio;
static bool sd_stat = false;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

#ifdef HAVE_MMCSD
/* IRQ handler */
static int sdio_cd_irq(int irq, void *context)
{
    bool inserted;
    bool changed = false;

    inserted = !stm32_gpioread(GPIO_SDIO_CD);

    if (inserted == true) {
        if (sd_stat == false) {
            message("SDcard inserted, initializing...\n");
            changed = true;
        }
    } else {
        /* inserted == false */
        if (sd_stat == true) {
            message("SDcard released, de-initializing...\n");
            changed = true;
        }
    }

    if (changed) {
        sdio_mediachange(sdio, inserted);
        sd_stat = inserted;
    }

    return OK;
}

static xcpt_t stm32_sdioattach(xcpt_t irqhandler)
{
    return stm32_gpiosetevent(GPIO_SDIO_CD, true, true, false, irqhandler);
}
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

int up_sdioinitialize(void)
{
#ifdef HAVE_MMCSD
    int ret;

    /* Mount the SDIO-based MMC/SD block driver */

    /* First, enable SD-Card slot GPIO-pins */
    stm32_configgpio(GPIO_SDIO_VDD_ENB);
    stm32_configgpio(GPIO_SDIO_CD);
    stm32_configgpio(GPIO_SDIO_WP);

    /* SD-Card slot power on */
    stm32_gpiowrite(GPIO_SDIO_VDD_ENB, true);

    /* Get an instance of the SDIO interface */
    sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

    if (!sdio) {
        message("nsh_archinitialize: Failed to initialize SDIO slot %d\n",
                CONFIG_NSH_MMCSDSLOTNO);
        return -ENODEV;
    }

    /* Now bind the SDIO interface to the MMC/SD driver */
    ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

    if (ret != OK) {
        message("nsh_archinitialize: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
        return ret;
    }

    sd_stat = !stm32_gpioread(GPIO_SDIO_CD);

    if (sd_stat) {
        message("SDcard present, initializing...\n");
        sdio_mediachange(sdio, true);
    }

    stm32_sdioattach(sdio_cd_irq);
#endif

    return OK;
}

