/****************************************************************************
 * SPI Flash initialize/mount utility
 *
 *  Copyright (C) 2012 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2012/11/27 16:56:40 kikuchi
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <nuttx/mtd.h>

#include "common.h"

static void usage(void)
{
    sched_lock();

    info("usage\n");
    info("  mtdutil <device-path>\n");

    sched_unlock();
}

int mtdutil_main(int argc, char **argv)
{
    struct inode *inode;
    const struct block_operations *ops;
    int ret;
    struct mtd_geometry_s geo;

    ret = open_blockdriver("/dev/mtdblock0" ,0, &inode);
    if (ret < 0) {
        err("Failed to open '/dev/mtdblock0': %d\n", ret);
        return ret;
    }

    ops = inode->u.i_bops;

    if (ops->ioctl) {
        ret = ops->ioctl(inode, MTDIOC_GEOMETRY, (unsigned long) &geo);

        if (ret == 0) {
            info("blocksize: %d\n", geo.blocksize);
            info("erasesize: %d\n", geo.erasesize);
            info("neraseblocks: %d\n", geo.neraseblocks);
        }
    }

    close_blockdriver(inode);

    return OK;
}
