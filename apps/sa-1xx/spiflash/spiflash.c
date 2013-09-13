/****************************************************************************
 * SPI Flash initialize/mount utility
 *
 *  Copyright (C) 2012 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/09/13 09:43:34 kikuchi
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <errno.h>

#ifdef CONFIG_STM32_SPI2
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd.h>
#endif

#include "common.h"

static void usage(void)
{
    sched_lock();

    info("usage\n");
    info("  spimount <mount-point>\n");

    sched_unlock();
}

int spimount_main(int argc, char **argv)
{
#if defined(CONFIG_STM32_SPI2)
    int ret;
    struct spi_dev_s *spi;
    struct mtd_dev_s *mtd;

    if (argc < 2) {
        usage();
        return -1;
    }

    /* Configure SPI-based devices */

    /* Get the SPI port */
    spi = up_spiinitialize(2);

    if (!spi) {
        info("%s: Failed to initialize SPI port 2\n", argv[0]);
        return -ENODEV;
    }

    /* Now bind the SPI interface to the N25Q064 SPI FLASH driver */
    info("Initializing N25Q064 SPI-Flash...\n");
    mtd = n25q_initialize(spi);

    if (!mtd) {
        info("%s: Failed to bind SPI port 2 to the SPI FLASH driver\n", argv[0]);
        return -ENODEV;
    }

    ret = ftl_initialize(1, mtd);

    return ret;
#else
    info("%s: need to set CONFIG_STM32_SPI2=y\n", argv[0]);
    return ERROR;
#endif
}
