/**-------------------------------------------------------------------------
@file	system_sam4e.c

@brief	CMSIS System initializations


@author	Hoang Nguyen Hoan
@date	June. 24, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#include "sam4e.h"

#include "system_sam4e.h"

#define MAINOSC_FREQ_MIN		3000000
#define MAINOSC_FREQ_MAX		20000000

#define USB_FREQ				48000000
#define PLLA_FREQ				240000000

/* Clock Settings (120MHz) */
#define SYS_BOARD_OSCOUNT   (CKGR_MOR_MOSCXTST(0x8U))
#define SYS_BOARD_PLLAR     (CKGR_PLLAR_ONE \
		| CKGR_PLLAR_MULA(0x13U) \
		| CKGR_PLLAR_PLLACOUNT(0x3fU) \
		| CKGR_PLLAR_DIVA(0x1U))
#define SYS_BOARD_MCKR      (PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK)

#define SYS_CKGR_MOR_KEY_VALUE	CKGR_MOR_KEY_PASSWD/* Key to unlock MOR register */

#pragma pack(push, 4)
typedef struct {
	OSC_TYPE Type;
	uint32_t Freq;		//!< in Hz
} OSC;
#pragma pack(pop)

static OSC s_MainOsc = {
	OSC_TYPE_XTAL,
	12000000
};

uint32_t SystemCoreClock = 120000000;


bool SystemSetMainOsc(OSC_TYPE Type, uint32_t Freq)
{
	if (Freq < MAINOSC_FREQ_MIN || Freq > 20000000)
	{
		return false;
	}

	s_MainOsc.Type = Type;

	if (Type == OSC_TYPE_RC)
	{
		if (Freq < 8000000)
		{
			s_MainOsc.Freq = 4000000;
		}
		else if (Freq < 16000000)
		{
			s_MainOsc.Freq = 8000000;
		}
		else
		{
			s_MainOsc.Freq = 12000000;
		}
	}
	else
	{
		s_MainOsc.Freq = Freq;
	}
	SystemInit();
}

void SystemSetPLLA()
{
	uint32_t div = 0;
	uint32_t mul = 0;

	for (int i = 1; i < 256; i++)
	{
		uint32_t freq =  s_MainOsc.Freq / i;
		if ((PLLA_FREQ % freq) == 0)
		{
			div = i;
			mul = PLLA_FREQ / freq - 1;
			break;
		}
	}

	SAM4E_PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_PLLACOUNT(0x3f) |
							CKGR_PLLAR_DIVA(div) | CKGR_PLLAR_MULA(mul);
	while ((SAM4E_PMC->PMC_SR & PMC_SR_LOCKA) == 0);
}

void SystemInit()
{
	uint32_t mor = 0;

	// WDT is on by default.  Disable it.
	SAM4E_WDT->WDT_MR = WDT_MR_WDDIS;

	SAM4E_EFC->EEFC_FMR = EEFC_FMR_FWS(5)|EEFC_FMR_CLOE;

	if (s_MainOsc.Type == OSC_TYPE_RC)
	{
		// Internal RC
		mor = CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCRCEN;
		switch (s_MainOsc.Freq)
		{
			case 8000000:
				mor |= CKGR_MOR_MOSCRCF_8_MHz;
				break;
			case 16000000:
				mor |= CKGR_MOR_MOSCRCF_12_MHz;
				break;
			default:
				s_MainOsc.Freq = 4000000;
				break;
		}
	}
	else
	{
		// External oscillator

		// Must force to RC before changing to external
		if (SAM4E_PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)
		{
			SAM4E_PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTST(0x8U) |
					CKGR_MOR_MOSCRCEN;

			while ((SAM4E_PMC->PMC_SR & PMC_SR_MOSCXTS) == 0);
		}
		mor = CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL;
	}

	SAM4E_PMC->CKGR_MOR = mor | CKGR_MOR_MOSCXTST(0x8U);
	while ((SAM4E_PMC->PMC_SR & PMC_SR_MOSCXTS) == 0);

	// Select MAINCLK
	SAM4E_PMC->PMC_MCKR = (SAM4E_PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
			PMC_MCKR_CSS_MAIN_CLK;

	while ((SAM4E_PMC->PMC_SR & PMC_SR_MCKRDY) == 0);

	SystemSetPLLA();

	SAM4E_PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;
	while ((SAM4E_PMC->PMC_SR & PMC_SR_MCKRDY) );

	SystemCoreClockUpdate();
}

void SystemCoreClockUpdate( void )
{
	uint32_t clk = 0;
	uint32_t mckr = SAM4E_PMC->PMC_MCKR;
	uint32_t tmp = 0;

	switch (mckr & PMC_MCKR_CSS_Msk)
	{
		case PMC_MCKR_CSS_SLOW_CLK:	// SLCK
			if ( SAM4E_SUPC->SUPC_SR & SUPC_SR_OSCSEL)
			{
				clk = 32768;
			}
			else
			{
				clk = 32000;
			}
		break;

		case PMC_MCKR_CSS_MAIN_CLK:	// MAINCK
			if (SAM4E_PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)
			{
				clk = s_MainOsc.Freq;
			}
			else
			{
				switch (SAM4E_PMC->CKGR_MOR & CKGR_MOR_MOSCRCF_Msk)
				{
					case CKGR_MOR_MOSCRCF_4_MHz:
						clk = 4000000;
						break;

					case CKGR_MOR_MOSCRCF_8_MHz:
						clk = 8000000;
						break;

					case CKGR_MOR_MOSCRCF_12_MHz:
						clk = 12000000;
						break;

					default:
						break;
				}
			}
			break;

		case PMC_MCKR_CSS_PLLA_CLK:	// PLLACK
			clk = s_MainOsc.Freq;
			tmp = SAM4E_PMC->CKGR_PLLAR;
			clk *= ((tmp & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos) + 1U;
			clk /= ((tmp & CKGR_PLLAR_DIVA_Msk) >> CKGR_PLLAR_DIVA_Pos);
			break;
	}

	mckr &= PMC_MCKR_PRES_Msk;
	if (mckr == PMC_MCKR_PRES_CLK_3)
	{
		clk /= 3;
	}
	else
	{
		clk >>= (mckr >> PMC_MCKR_PRES_Pos);
	}

	SystemCoreClock = clk;

	if (SystemCoreClock < CHIP_FREQ_FWS_0)
	{
		SAM4E_EFC->EEFC_FMR = EEFC_FMR_FWS(0) | EEFC_FMR_CLOE;
	}
	else if (SystemCoreClock < CHIP_FREQ_FWS_1)
	{
		SAM4E_EFC->EEFC_FMR = EEFC_FMR_FWS(1) | EEFC_FMR_CLOE;
	}
	else if (SystemCoreClock < CHIP_FREQ_FWS_2)
	{
		SAM4E_EFC->EEFC_FMR = EEFC_FMR_FWS(2) | EEFC_FMR_CLOE;
	}
	else if (SystemCoreClock < CHIP_FREQ_FWS_3)
	{
		SAM4E_EFC->EEFC_FMR = EEFC_FMR_FWS(3) | EEFC_FMR_CLOE;
	}
	else if (SystemCoreClock < CHIP_FREQ_FWS_4)
	{
		SAM4E_EFC->EEFC_FMR = EEFC_FMR_FWS(4) | EEFC_FMR_CLOE;
	}
	else
	{
		SAM4E_EFC->EEFC_FMR = EEFC_FMR_FWS(5) | EEFC_FMR_CLOE;
	}
}

