/*
 ============================================================================
 Name        : Blinky.c
 Author      : Hoan Hoang
 Version     :
 Copyright   : Copyright 2014, I-SYST
 Description : Hello World in C
 ============================================================================
 */

#include <stdio.h>
#include "LPC17xx.h"
#include "imm_lpc1769.h"
#include "iopincfg.h"

IOPINCFG g_IOPinCfg[] = {
	{IMM_LPC1769_LED_BLUE_PORT, IMM_LPC1769_LED_BLUE_PIN, 0, IOPINDIR_OUTPUT,
	 IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{IMM_LPC1769_LED_YEL_PORT, IMM_LPC1769_LED_YEL_PIN, 0, IOPINDIR_OUTPUT,
	 IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{IMM_LPC1769_LED_RED_PORT, IMM_LPC1769_LED_RED_PIN, 0, IOPINDIR_OUTPUT,
	 IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

/*
 *
 * Print a greeting message on standard output and exit.
 *
 * On embedded platforms this might require semi-hosting or similar.
 *
 * For example, for toolchains derived from GNU Tools for Embedded,
 * to enable semi-hosting, the following was added to the linker:
 *
 * --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
 *
 * Adjust it for other toolchains.
 *
 */

int
main(void)
{
	IOPinCfg(g_IOPinCfg, 3);

	int i = 0, k = 2;
	while (1)
	{
		// Set current LED
		LPC_GPIO0[g_IOPinCfg[i].PortNo].FIOSET = (1 << g_IOPinCfg[i].PinNo);

		// Clear previous LED
		LPC_GPIO0[g_IOPinCfg[k].PortNo].FIOCLR = (1 << g_IOPinCfg[k].PinNo);

		// Little delay here
		for (int j = 0; j < 1000000; j++);

		i++;
		if (i > 2)
			i = 0;
		k++;
		if (k > 2)
			k = 0;

	}

	return 0;
}
