/*--------------------------------------------------------------------------
File   : system_LPC17xx.c

Author : Hoang Nguyen Hoan          Jan. 23, 2012

Desc   : Implementation of CMSIS Cortex-M3 Device for LPC17xx Device Series
		 Added SystemSetCoreClock for on the fly clock setting

Copyright (c) 2012, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdint.h>

#include "LPC17xx.h"
#include "system_LPC17xx.h"

#define OSC_FREQ				OSC_FREQ_16MHZ  /* Installed oscillator frequency */
#define IRC_FREQ				(4000000UL)		/* Internal RC freq */
#define RTC_FREQ				(32768UL)		/* RTC frequency */

/* Select system clock oscillator source */
#define DEF_CLK_SRC			IRC_FREQ
//#define CLKSRCSEL_VAL		CLKSRCSEL_MAIN

/*
 * This variable indicates actual Core Clock (cclk) in Hz
 * Update this by calling SystemCoreClockUpdate.
 *
 * Default on reset, internal RC is selected
 */
uint32_t SystemCoreClock = IRC_FREQ;/*!< System Clock Frequency (Core Clock)*/
uint32_t SystemClkFreq = IRC_FREQ;	/* System clock frequency, reset default IRC */
extern uint32_t SystemMicroSecLoopCnt;

uint32_t GetSysClk(void)
{
	uint32_t sysclk;

	switch (LPC_SC->CLKSRCSEL & 0x03)
	{
		case CLKSRCSEL_MAIN:
			sysclk = SystemClkFreq;//OSC_FREQ;
			break;
		case CLKSRCSEL_RTC:
			sysclk = RTC_FREQ;
			break;
		default:
			sysclk = IRC_FREQ;
			break;
	}

	return sysclk;
}

/*
 * Update SystemCoreClock based on registers values
 */
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
	uint32_t sysclk;
	uint32_t pll0stat = LPC_SC->PLL0STAT;
	int32_t	n, m;

	sysclk = GetSysClk();

	if ((pll0stat & 0x3000000) == 0x3000000)
	{
		/* PLL0 is connected */
		m = (pll0stat & 0xefff) + 1;
		n = ((pll0stat >> 16) & 0xff) + 1;

		/* FCCO = (2 × M × FIN) / N */
		SystemCoreClock = sysclk * 2 * m / n;
	}
	else
		SystemCoreClock = sysclk;

	/* adjust to cclk divider */
	SystemCoreClock /= (LPC_SC->CCLKCFG & 0xFF) + 1;
	SystemMicroSecLoopCnt = (SystemCoreClock / 16000000);
}

uint32_t SystemCoreClockSet(uint32_t CoreFreq, int ClkFreq)
{
	uint32_t temp = 0;
	uint32_t sysclk = ClkFreq;
	uint32_t clksrcsel = CLKSRCSEL_IRC;	// Default internal RC

	/* Init system clock */

	SystemClkFreq = ClkFreq;

	if (ClkFreq > IRC_FREQ)
	{
		// Select MAIN clk source
		// OSCEN - Oscillator enable
		temp = 0x20;
		clksrcsel = CLKSRCSEL_MAIN;
		if (SystemClkFreq > 15000000)
			temp |= 0x10;	/* Oscillator range 15MHz-25MHz */

		LPC_SC->SCS = temp;
		while ((LPC_SC->SCS & 0x70) == 0x50);	/* Wait for main oscillator ready */

	}
	else if (ClkFreq < IRC_FREQ)
	{
		clksrcsel = CLKSRCSEL_RTC;
	}


	/* Setting PLL0 */

	/* find CCLKDIV & PLLCLK to get closes match */
	int32_t cclkdiv = 1;
	int32_t pllclk = 0;
	int32_t delta = 0, predelta = CoreFreq;
	int i;

	for (i = 1; i < 257; i++)
	{
		pllclk = CoreFreq * i;
		/* pllclk range is 275MHz - 550MHz */
		if (pllclk < 275000000UL)
			continue;
		if (pllclk > 550000000UL)
			break;
		/* try to select closest useable frequency for USB clock */
		delta = CoreFreq - (pllclk / 48000000UL) * 48000000UL / i;
		if (delta < predelta)
		{
			predelta = delta;
			cclkdiv = i;
		}
	}

	pllclk = CoreFreq * cclkdiv;

	/* Set cclk divider */
	LPC_SC->CCLKCFG = (cclkdiv - 1) & 0xff;
	LPC_SC->CLKSRCSEL = clksrcsel; //CLKSRCSEL_VAL;

	/* Find M & N value */
	/* Formula M = (FCCO × N) / (2 × FIN) */
	sysclk <<= 1;

	for (i = 1; i < 33; i++)
	{
		int t = pllclk * i;
		temp = t / sysclk;
		if ((t % sysclk) == 0)
			break;
	}

	/* results : i is N, temp is M */
	i--;
	temp--;
	temp = ((i & 0xff) << 16) | (temp & 0xefff);
	LPC_SC->PLL0CFG = temp;
	LPC_SC->PLL0FEED  = 0xAA;
	LPC_SC->PLL0FEED  = 0x55;

	LPC_SC->PLL0CON   = 0x01;
	LPC_SC->PLL0FEED  = 0xAA;
	LPC_SC->PLL0FEED  = 0x55;
	/* Wait for PLL0 to lock */
	while (!(LPC_SC->PLL0STAT & (1<<26)));

	LPC_SC->PLL0CON   = 0x03;
	LPC_SC->PLL0FEED  = 0xAA;
	LPC_SC->PLL0FEED  = 0x55;
	/* Wait for PLL0 connect */
	while (!(LPC_SC->PLL0STAT & (1<<25)));

	/* USBCLK divider */
	temp = (pllclk / 48000000) - 1;
	LPC_SC->USBCLKCFG = temp & 0xf;

	SystemCoreClockUpdate();

	/* Calc Flash accelerator timming */
	temp = 0;
	if (SystemCoreClock > 20000000)
		temp = SystemCoreClock / 20000000 - 1;
	if (temp > 5)
		temp = 5;
	LPC_SC->FLASHCFG = (LPC_SC->FLASHCFG & ~0xf000) | (temp << 12);

	return SystemCoreClock;
}

void SystemConnectPLL1(void)
{
	uint32_t sysclk = GetSysClk();
	int m, p, found = 0;
	uint32_t clk;
	uint32_t val = 0;

	if ((48000000 % sysclk) != 0)
	{
		p = 1;
		for (m = 1; m < 33 && found == 0; m++)
		{
			/* try multiplier first with USBCLK = M × FOSC */
			clk = sysclk * m;
			if (clk == 48000000)
			{
				val = m - 1;
				found = 1;
				break;
			}
			/* try divider FCCO = FOSC × M × 2 × P */
			for (p = 1; p < 9; p<<=1)
			{
				clk *= (p << 1);
				if ((clk % 48000000) == 0)
				{
					val = (m - 1) | ((p -1) << 5);
					found = 1;
					break;
				}
			}
		}
	}
	else
	{
		val = 48000000 / sysclk - 1;
	}

	if (val)
	{
		LPC_SC->PLL1CFG   = val;
		LPC_SC->PLL1FEED  = 0xAA;
		LPC_SC->PLL1FEED  = 0x55;

		LPC_SC->PLL1CON   = 0x01;
		LPC_SC->PLL1FEED  = 0xAA;
		LPC_SC->PLL1FEED  = 0x55;
		/* Wait for PLL1 to lock */
		while (!(LPC_SC->PLL1STAT & (1<<10)));

		LPC_SC->PLL1CON   = 0x03;
		LPC_SC->PLL1FEED  = 0xAA;
		LPC_SC->PLL1FEED  = 0x55;
		/* Wait for PLL1 to connect */
		while (!(LPC_SC->PLL1STAT & (1<< 9)));
	}
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
	SystemCoreClockSet(CORE_FREQ, DEF_CLK_SRC);
}
