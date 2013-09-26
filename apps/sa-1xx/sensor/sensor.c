/****************************************************************************
 * LM75 Sensor Test
 *
 *  Copyright (C) 2013 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/09/26 11:53:34 kikuchi
 ****************************************************************************/

#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/sensors/lm75.h>

#include "common.h"

int sensor_main(int argc, char **argv)
{
	int fd, len, res, temp;
	float t;
	unsigned long notused = 0;

	if ((fd = open("/dev/temp0", O_RDONLY)) < 0) {
		info("unable to open /dev/temp0.\n");
		return ERROR;
	}

	if ((res = ioctl(fd, SNIOC_POWERUP, notused)) < 0) {
		info("ioctl(SNIOC_POWERUP) failed.\n");
		res = ERROR;
		goto ret;
	}

	if ((len = read(fd, (char *) &temp, sizeof(temp))) <= 0) {
		info("unable to read from /dev/temp0.\n");
		res = ERROR;
		goto ret;
	}

	temp = temp >> 11;
	t = ((float) temp) / 2.0;
	info("temp: %.1f [deg]\n", temp, t);

	res = OK;

ret:
	close(fd);

	return res;
}
