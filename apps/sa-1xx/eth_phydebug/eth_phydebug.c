/****************************************************************************
 * Ethernet PHY dump
 *
 *  Copyright (C) 2012 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2012/11/21 13:08:26 kikuchi
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "common.h"

extern void stm32_eth_phydebug(void);

int eth_phydebug_main(int argc, char **argv)
{
    stm32_eth_phydebug();

    return OK;
}
