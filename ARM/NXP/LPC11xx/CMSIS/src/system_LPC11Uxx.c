/*--------------------------------------------------------------------------
File   : system_LPC11Uxx.c

Author : Hoang Nguyen Hoan          Dec. 9, 2014

Desc   : Implementation of CMSIS Cortex-M0 Device for LPC11Uxx Device Series
		 Added SystemSetCoreClock for on the fly clock setting

Copyright (c) 2014, I-SYST inc., all rights reserved

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
#include <stdbool.h>

#include "system_core_clock.h"
#include "LPC11Uxx.h"


#define OSC_FREQ				XTAL_FREQ_12MHZ	// Installed oscillator frequency
#define IRC_FREQ				(12000000UL)	// Internal RC freq */

// Select system clock oscillator source
#define DEF_CLK_SRC				IRC_FREQ

uint32_t SystemCoreClock = 48000000UL;	// System Clock Frequency (Core Clock)
uint32_t SystemMainClkFreq = IRC_FREQ;		// System clock frequency, reset default IRC

static inline uint32_t GetSysPllClk(void)
{
	if ((LPC_SYSCON->SYSPLLCLKSEL & SYSPLLCLKSEL_MASK) == SYSPLLCLKSEL_SYSOSC)
		return OSC_FREQ;
	else
		return IRC_FREQ;
}

/*
 * Update SystemCoreClock based on registers values
 */
void SystemCoreClockUpdate(void)
{
	uint32_t sysclk;
	uint32_t mainclk = LPC_SYSCON->MAINCLKSEL;
	int32_t	p = 1, m;

	//sysclk = GetSysClk();

	switch (mainclk & MAINCLKSEL_MASK)
	{
	case MAINCLKSEL_PLLIN:
		sysclk = GetSysPllClk();
		break;
	case MAINCLKSEL_PLLOUT:
		/* PLL is connected */
		m = (LPC_SYSCON->SYSPLLCTRL & 0x1f) + 1;
		p = 1 << ((LPC_SYSCON->SYSPLLCTRL >> 5) & 3);

		/* FCCO = (2 × P x M × FIN) */
		sysclk = GetSysPllClk() * 2 * m * p;
		break;
	case MAINCLKSEL_WDT:
		sysclk = GetSysPllClk();
		break;
	case MAINCLKSEL_IRC:
		sysclk = IRC_FREQ;
	}

	// fclko = fcco /(p << 1);
	SystemMainClkFreq = sysclk / (p << 1);

	/* adjust to cclk divider */
	if (LPC_SYSCON->SYSAHBCLKDIV & 0xff)
		SystemCoreClock = SystemMainClkFreq / (LPC_SYSCON->SYSAHBCLKDIV & 0xff);
}

uint32_t SystemSetCoreClock(bool Crystal, int ClkFreq)
{
	uint32_t sysclk = IRC_FREQ;

	if (Crystal)
	{
		// Use crystal osc source
		LPC_SYSCON->PDRUNCFG     &= ~PDRUNCFG_SYSOSC_PD;
		sysclk = ClkFreq;
		LPC_SYSCON->SYSPLLCLKSEL = SYSPLLCLKSEL_SYSOSC;
		LPC_SYSCON->USBPLLCLKSEL = USBPLLCLKSEL_SYSOSC;
	}
	else
	{
		LPC_SYSCON->SYSPLLCLKSEL = SYSPLLCLKSEL_IRC;
		LPC_SYSCON->USBPLLCLKSEL = USBPLLCLKSEL_IRC;
	}
	LPC_SYSCON->SYSPLLCLKUEN  = 0x01;               /* Update Clock Source      */
	LPC_SYSCON->SYSPLLCLKUEN  = 0x00;               /* Toggle Update Register   */
	LPC_SYSCON->SYSPLLCLKUEN  = 0x01;
	while (!(LPC_SYSCON->SYSPLLCLKUEN & 0x01));     /* Wait Until Updated       */

	// Calc PLL values
	uint32_t m = 1;
	uint32_t p = 0;
	uint32_t div = 1;

	// FCCO = M * Fclkin * 2 * P
	// Find best fit m & p values
	uint32_t fclko = 48000000;

	while (p < 4)
	{
		uint32_t fcco = m * sysclk * ((1 << p) << 1);

		if (fcco > 156000000UL && fcco < 320000000)
		{
			fclko = fcco /((1 << p) << 1);
			div = fclko / 48000000;
			if ((fclko % 48000000) == 0)
				break;
		}
		if (++m >= 0x20 || fcco >= 320000000)
		{
			m = 1;
			p++;
		}
	}

	uint32_t pll = (m - 1) | (p << 5);

	// Core clock
	LPC_SYSCON->SYSPLLCTRL = pll;
	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSPLL_PD;	// Power-up SYSPLL
	while (!(LPC_SYSCON->SYSPLLSTAT & 0x01));	    // Wait Until PLL Locked

	LPC_SYSCON->SYSAHBCLKDIV  = div;

	LPC_SYSCON->MAINCLKSEL    = MAINCLKSEL_PLLOUT;
	LPC_SYSCON->MAINCLKUEN    = 0x00;
	LPC_SYSCON->MAINCLKUEN    = 0x01;
	while (!(LPC_SYSCON->MAINCLKUEN & 0x01));

	if (Crystal == true)
	{
	// USB clock
	LPC_SYSCON->PDRUNCFG     &= ~PDRUNCFG_USBPLL_PD;	// Power-up USB PLL
	LPC_SYSCON->USBPLLCLKSEL  = USBPLLCLKSEL_SYSOSC;	// Select PLL Input
	LPC_SYSCON->USBPLLCLKUEN  = 0x00;               	// Toggle Update Register
	LPC_SYSCON->USBPLLCLKUEN  = 0x01;
	while (!(LPC_SYSCON->USBPLLCLKUEN & 0x01));     	// Wait Until Updated

	// USB PLL
	LPC_SYSCON->USBPLLCTRL    = pll;
	while (!(LPC_SYSCON->USBPLLSTAT   & 0x01));     	// Wait Until PLL Locked
	LPC_SYSCON->USBCLKSEL     = USBCLKSEL_PLLOUT;		// Select USB PLL
	}
	LPC_SYSCON->USBCLKDIV	= div;      				// Set USB clock divider
	LPC_SYSCON->USBCLKUEN	= 0x00;               		// Toggle Update Register
	LPC_SYSCON->USBCLKUEN	= 0x01;

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<16);				// Power-up IOCON
	LPC_SYSCON->PDRUNCFG     &= ~PDRUNCFG_USBPAD_PD;	// Power-up USB PHY         */

	return fclko;
}

void SystemInit (void)
{
	SystemSetCoreClock(true, OSC_FREQ);
	SystemCoreClockUpdate();
}
