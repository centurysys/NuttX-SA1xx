/****************************************************************************
 * SPI Flash initialize/mount utility
 *
 *  Copyright (C) 2012 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2012/11/28 17:04:05 kikuchi
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
#  include <nuttx/spi.h>
#  include <nuttx/mtd.h>

#  ifdef CONFIG_FS_NXFFS
#    include <sys/mount.h>
#    include <nuttx/fs/nxffs.h>
#  endif
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
#if defined(CONFIG_STM32_SPI2) && defined(CONFIG_FS_NXFFS)
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

    ret = nxffs_initialize(mtd);

    if (ret < 0) {
        info("%s: NXFFS initialization failed: %d (%s)\n",
             argv[0], ret, strerror(ret));
        return ret;
    }

    /* Mount the file system at /mnt/spifi */
    info("Mounting SPI-Flash to %s\n", argv[1]);
    ret = mount(NULL, argv[1], "nxffs", 0, NULL);

    if (ret < 0) {
        info("%s: Failed to mount the NXFFS volume to %s: %d (%s)\n",
             argv[0], argv[1], errno, strerror(errno));
        return ret;
    }

    info("Mount succeeded.\n");
    return OK;
#else
    info("%s: need to set CONFIG_STM32_SPI2=y and CONFIG_FS_NXFFS=y\n", argv[0]);
    return ERROR;
#endif
}
