/************************************************************************************
 * configs/sa-1xx/src/up_eeprom.c
 * arch/arm/src/board/up_eeprom.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Takeyoshi Kikuchi <kikuchi@centurysys.jp>
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
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>

#if defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C2)

/************************************************************************************
 * Definitions
 ************************************************************************************/

#ifndef info
#define info(fmt, arg...) do {sched_lock(); lowsyslog(fmt, ##arg); sched_unlock();} while (0)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

typedef unsigned char u8;
typedef unsigned int u32;

/* EEPROM contents */
struct hw_parameter {
	u8 macaddr[6];
	u8 reserved0[6];

	struct {
		u8 year[4];
		u8 month[2];
		u8 day[2];
	} manufacture_date;

	u32 serial;

	struct {
		u8 number[5];
		u8 model[2];
		u8 revision;
	} board;
};

#define param_offset(name) ((long) &(((struct hw_parameter *) 0)->name))

typedef enum {
	T_STRING,
	T_INT,
	T_MACADDR
} param_type_t;

struct eeprom_param {
	char *name;
	int offset;
	int len;
	param_type_t type;
	char *description;
};

static struct eeprom_param param_idx[] = {
	{
		"macaddr",
		param_offset(macaddr),
		6, T_MACADDR,
		"Ethernet MAC address"
	},
#if 0
	{
		"manufacture_year",
		param_offset(manufacture_date.year),
		4, T_STRING,
		"ManufactureDate (Year)"
	},
	{
		"manufacture_month",
		param_offset(manufacture_date.month),
		2, T_STRING,
		"ManufactureDate (Month)"
	},
	{
		"manufacture_day",
		param_offset(manufacture_date.day),
		2, T_STRING,
		"ManufactureDate (Day)"
	},
#endif
	{
		"manufacture_date",
		param_offset(manufacture_date),
		8, T_STRING,
		"ManufactureDate"
	},
	{
		"serial",
		param_offset(serial),
		4, T_INT,
		"SerialNumber"
	},
	{
		"board_number",
		param_offset(board.number),
		5, T_STRING,
		"Board Number"
	},
	{
		"board_model",
		param_offset(board.model),
		2, T_STRING,
		"Board Model"
	},
	{
		"board_revision",
		param_offset(board.revision),
		1, T_STRING,
		"Board Revision"
	}
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int write_eeprom(char *buf, int offset, int len)
{
	int fd, ret;

	if ((fd = open("/dev/mtd0", O_WRONLY)) < 0)
		return -errno;

	lseek(fd, offset, SEEK_SET);
	ret = write(fd, buf, len);

	close(fd);

	return ret;
}

static int read_eeprom(char *buf, int offset, int len)
{
	int fd, ret;

	if ((fd = open("/dev/mtd0", O_RDONLY)) < 0)
		return -errno;

	lseek(fd, offset, SEEK_SET);
	ret = read(fd, buf, len);

	close(fd);

	return ret;
}

static struct eeprom_param *find_param(const char *paramname)
{
	int i;
	struct eeprom_param *p;

	for (i = 0; i < ARRAY_SIZE(param_idx); i++) {
		p = &param_idx[i];

		if (strncmp(p->name, paramname, strlen(p->name)) == 0)
			return p;
	}

	return NULL;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sa1xx_get_parameter
 *
 * Description:
 *   Get parameter from EEPROM
 ************************************************************************************/

int sa1xx_get_parameter(const char *paramname, char *buf, int size, int convert)
{
	struct eeprom_param *p;
	int res, i;
	char tmp[size + 1];

	if (!buf)
		return -EFAULT;

	p = find_param(paramname);
	if (!p)
		return -EFAULT;

	res = read_eeprom(tmp, p->offset, p->len);

	if (p->type == T_MACADDR) {
		if (convert != 0) {
			for (i = 0; i < 6; i++) {
				sprintf(&buf[i * 2], "%02x", (int) tmp[i]);
			}
			res = res * 2;
			buf[i * 2] = '\0';
		} else {
			memcpy(buf, tmp, p->len);
		}
	} else if (p->type == T_INT) {
		if (convert != 0)
			res = sprintf(buf, "%d", *(int *) tmp);
		else
			strncpy(buf, tmp, p->len);
	} else {
		strncpy(buf, tmp, p->len);
	}

	return res;
}

/************************************************************************************
 * Name: sa1xx_set_parameter
 *
 * Description:
 *   Set parameter to EEPROM
 ************************************************************************************/

int sa1xx_set_parameter(const char *paramname, char *buf, int size)
{
	struct eeprom_param *p;
	int res, i;
	char tmp[16];

	if (!buf)
		return -EFAULT;

	p = find_param(paramname);
	if (!p)
		return -EFAULT;

	if (p->type == T_MACADDR) {
		for (i = 0; i < 6; i++) {
			char _tmp[3];
			strncpy(_tmp, &buf[i * 2], 2);
			tmp[i] = strtoul(_tmp, NULL, 16);
		}
		tmp[i] = '\0';
	} else if (p->type == T_INT) {
		long val = strtol(buf, NULL, 10);
		memcpy(tmp, &val, sizeof(val));
	} else {
		strncpy(tmp, buf, p->len);
	}

	res = write_eeprom(tmp, p->offset, p->len);

	return res;
}

/************************************************************************************
 * Name: sa1xx_print_paramnames
 *
 * Description:
 *   Print all parameter name
 ************************************************************************************/

void sa1xx_print_paramnames(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(param_idx); i++)
		info("  %s (%s)\n", param_idx[i].name, param_idx[i].description);
}

#endif /* defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C2) */
