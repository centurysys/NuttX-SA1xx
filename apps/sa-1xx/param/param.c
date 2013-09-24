/****************************************************************************
 * EEPROM parameters get/set utility
 *
 *  Copyright (C) 2013 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/09/24 15:27:57 kikuchi
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
#include "sa1xx-internal.h"

static void usage(void)
{
	sched_lock();

	info("usage\n");
	info("  param get <parameter_name>\n");
	info("  param set <parameter_name> <value>\n");

	info("\navailable parameter_name:\n");
	sa1xx_print_paramnames();

	sched_unlock();
}

static int get_param(const char *param_name)
{
	char val[16];
	int len, res;

	res = sa1xx_get_parameter(param_name, val, 16);

	if (res > 0) {
		val[res] = '\0';
		printf("%s\n", val);
		res = OK;
	} else {
		printf("parameter: %s not found.\n", param_name);
		res = ERROR;
	}

	return res;
}

static int set_param(const char *param_name, char *val)
{
	char val_new[16];
	int len, res;

	res = sa1xx_set_parameter(param_name, val, strlen(val) - 1);

	if (res > 0) {
		res = sa1xx_get_parameter(param_name, val_new, 16);

		if (res > 0) {
			val_new[res] = '\0';
			printf("%s\n", val_new);
		}
	}

	return res;
}

int param_main(int argc, char **argv)
{
	int res, i;

	if (argc < 2) {
		usage();
		return ERROR;
	}

	if (strncmp(argv[1], "get", 3) == 0 && argc == 3) {
		res = get_param(argv[2]);
	} else if (strncmp(argv[1], "set", 3) == 0 && argc == 4) {
		res = set_param(argv[2], argv[3]);
	} else
		res = ERROR;

	return res;
}
