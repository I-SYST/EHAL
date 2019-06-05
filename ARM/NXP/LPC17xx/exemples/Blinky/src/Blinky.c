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
#include "coredev/iopincfg.h"
#include "coredev/i2c.h"
#include "seep.h"
#include "idelay.h"
#include "system_core_clock.h"

IOPINCFG g_IOPinCfg[] = {
	{IMM_LPC1769_LED_BLUE_PORT, IMM_LPC1769_LED_BLUE_PIN, 0, IOPINDIR_OUTPUT,
	 IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{IMM_LPC1769_LED_YEL_PORT, IMM_LPC1769_LED_YEL_PIN, 0, IOPINDIR_OUTPUT,
	 IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{IMM_LPC1769_LED_RED_PORT, IMM_LPC1769_LED_RED_PIN, 0, IOPINDIR_OUTPUT,
	 IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const IOPINCFG s_I2CPins[] = {
	{IMM_LPC1769_SEEPMAC_SDA_PORT, IMM_LPC1769_SEEPMAC_SDA_PIN, 3, IOPINDIR_BI,
	 IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{IMM_LPC1769_SEEPMAC_SCL_PORT, IMM_LPC1769_SEEPMAC_SCL_PIN, 3, IOPINDIR_OUTPUT,
	 IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}
};
// I2C1 config
//
const I2CCFG g_I2C1Cfg = {
	1,	// I2C interface number
	{
		{IMM_LPC1769_SEEPMAC_SDA_PORT, IMM_LPC1769_SEEPMAC_SDA_PIN, 3, IOPINDIR_BI,
		 IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
		{IMM_LPC1769_SEEPMAC_SCL_PORT, IMM_LPC1769_SEEPMAC_SCL_PIN, 3, IOPINDIR_OUTPUT,
		 IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}
	},
	100000,		// data rate in Hz
	I2CMODE_MASTER,
	0, 	// Slave mode address
	3	// Max retry
};

I2CDEV g_I2cDev;

static const SEEP_CFG s_SeepCfg = {
	0xa0>>1,
	1,	// Address length
	8,	// Page size
};

SEEPDEV g_Seep;

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
	uint8_t macaddr[6];

	memset(macaddr, 0, sizeof(macaddr));

	I2CInit(&g_I2cDev, &g_I2C1Cfg);

	SeepInit(&g_Seep, &s_SeepCfg, &g_I2cDev.DevIntrf);

	// Read MAC address
	SeepRead(&g_Seep, 0xfa, macaddr, 6);

	if (macaddr[0] !=0 || macaddr[1] != 4 || macaddr[2] != 0xa3)
	{
		// LPCXpresso board
		SystemCoreClockSet(CORE_FREQ, OSC_FREQ_12MHZ);
	}
	else
	{
		// Set system clock for IMM-LPC1769 board
		SystemCoreClockSet(CORE_FREQ, OSC_FREQ_16MHZ);
	}
	IOPinCfg(g_IOPinCfg, 3);

	int i = 0, k = 2;
	while (1)
	{
		// Set current LED
		LPC_GPIO0[g_IOPinCfg[i].PortNo].FIOSET = (1 << g_IOPinCfg[i].PinNo);

		// Clear previous LED
		LPC_GPIO0[g_IOPinCfg[k].PortNo].FIOCLR = (1 << g_IOPinCfg[k].PinNo);

		// Little delay here
		//for (int j = 0; j < 1000000; j++);
		usDelay(1000000);
		i++;
		if (i > 2)
			i = 0;
		k++;
		if (k > 2)
			k = 0;

	}

	return 0;
}
