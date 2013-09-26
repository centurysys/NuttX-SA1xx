/****************************************************************************
 * DIPSW test
 *
 *  Copyright (C) 2013 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/09/26 18:38:00 kikuchi
 ****************************************************************************/

#include <stdio.h>

#include "common.h"

int dipsw_main(int argc, char **argv)
{
	uint8_t val;

	up_dipswinit();
	val = up_dipsw();

	info("DIPSW: 0x%02x\n", val);

	return OK;
}
