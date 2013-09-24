/****************************************************************************
 * SPI Flash initialize/mount utility
 *
 *  Copyright (C) 2012 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/09/24 09:49:08 kikuchi
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
    info("  eepromtest <device-path>\n");

    sched_unlock();
}

static void fill_eeprom(int fd, int len)
{
	int res;
	short i;

	for (i = 0; i < len / sizeof(short); i++) {
		if ((i % 100) == 0)
			printf("count: %d\n", i);

		res = write(fd, &i, sizeof(short));
		if (res <= 0) {
			printf("write failed.\n");
			break;
		}
	}
}

static void dump_eeprom(int fd, int len)
{
	int res;
	short i;
	unsigned char val;

	lseek(fd, 0, SEEK_SET);

	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			printf("%04x:", i);

		res = read(fd, &val, 1);
		printf(" %02x", val);

		if ((i % 16) == 15)
			printf("\n");
	}
}

int eepromtest_main(int argc, char **argv)
{
    int fd;

	if ((fd = open("/dev/mtd0", O_RDWR)) < 0) {
		info("open(mtd0) failed.\n");
		return -1;
	}

	fill_eeprom(fd, 2048);
	dump_eeprom(fd, 2048);

	close(fd);

    return OK;
}
