/************************************************************************************
 * configs/sa-1xx/src/up_spi.c
 * arch/arm/src/board/up_spi.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32.h"
#include "sa1xx-internal.h"

#ifdef CONFIG_STM32_SPI2

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

#ifndef info
#define info(fmt, arg...) do {sched_lock(); lib_lowprintf(fmt, ##arg); sched_unlock();} while (0)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SA-1xx board.
 *
 ************************************************************************************/

void weak_function stm32_spiinitialize(void)
{
    /* SPI2 connects to M25P64 SPI-Flash */
#if 0
#ifdef CONFIG_STM32_SPI2
    stm32_configgpio(GPIO_SPI2_NSS);
    stm32_configgpio(GPIO_SPI2_SCK);
    stm32_configgpio(GPIO_SPI2_MISO);
    stm32_configgpio(GPIO_SPI2_MOSI);
#endif
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
    spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

    if (devid == SPIDEV_FLASH) {
        /* Set the GPIO low to select and high to de-select */
        stm32_gpiowrite(GPIO_SPI2_NSS, !selected);
    }
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
    return SPI_STATUS_PRESENT;
}

int sa1xx_spiflash_initialize(void)
{
    int ret;
    struct spi_dev_s *spi;
    struct mtd_dev_s *mtd;

    /* Configure SPI-based devices */

    /* Get the SPI port */
    spi = up_spiinitialize(2);

    if (!spi) {
        spidbg("%s: Failed to initialize SPI port 2\n", __FUNCTION__);
        return -ENODEV;
    }

    /* Now bind the SPI interface to the N25Q064 SPI FLASH driver */
    spidbg("Initializing N25Q064 SPI-Flash...\n");
    mtd = n25q_initialize(spi);

    if (!mtd) {
        spidbg("%s: Failed to bind SPI port 2 to the SPI FLASH driver\n", __FUNCTION__);
        return -ENODEV;
    }

    ret = ftl_initialize(1, mtd);

    return ret;
}
#endif
#endif /* CONFIG_STM32_SPI2 */
