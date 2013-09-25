/****************************************************************************
 * Software Reset
 *
 *  Copyright (C) 2013 Century Systems
 *  Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
 *
 *  Last Modified: 2013/09/25 18:16:09 kikuchi
 ****************************************************************************/

#include "common.h"
#include "up_arch.h"
#include "nvic.h"

int sysreset_main(int argc, char **argv)
{
	uint32_t val;

	val = (0x5fa << NVIC_AIRCR_VECTKEY_SHIFT) | NVIC_AIRCR_SYSRESETREQ;
	putreg32(val, NVIC_AIRCR);

	/* notreached... */
	return OK;
}
