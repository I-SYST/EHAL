/*--------------------------------------------------------------------------
File   : system_stm32f0xx.c

Author : Hoang Nguyen Hoan          June 5, 2019

Desc   : Implementation of CMSIS SystemInit for STM32F0xx Device Series

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#include <stdbool.h>
#include <stdlib.h>

#include "stm32f0xx.h"
#include "system_core_clock.h"

#define STM32F0XX_HSE_XTAL_EN			false
#define STM32F0XX_HSE_XTAL_FREQ			0

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK;

static uint32_t s_XtlFreq = 0;

void SystemCoreClockUpdate (void)
{
	uint32_t cfgr = RCC->CFGR;

	if (cfgr & RCC_CFGR_SW_PLL)
	{
		uint32_t sysclk = 0;

		if (cfgr & RCC_CFGR_PLLSRC_HSE_PREDIV)
		{
			// Crystal
			sysclk = s_XtlFreq / ((RCC->CFGR2 & 4) + 1);
		}
		else
		{
			// HSI / 2
			sysclk = 4000000;
		}

		uint32_t m = ((cfgr & RCC_CFGR_PLLMUL_Msk) >> RCC_CFGR_PLLMUL_Pos) + 2;
		sysclk = sysclk * m;

		SystemCoreClock = sysclk;
	}
	else if (cfgr & RCC_CFGR_SWS_HSE)
	{
		// HSE
		SystemCoreClock = s_XtlFreq;
	}
	else
	{
		// HSI 8MHz
		SystemCoreClock = 8000000;
	}
}

//
// ClkFreq = Crystal frequency
//
uint32_t SystemCoreClockSet(bool bCrystal, uint32_t ClkFreq)
{
	uint32_t cfgr = 0;
	uint32_t cfgr2 = 0;

	RCC->CR |= RCC_CR_HSION;

	while ((RCC->CR & RCC_CR_HSIRDY) == 0);

	RCC->CR |= RCC_CR_CSSON;

	// Always select PLL for max core frequency
	cfgr |= RCC_CFGR_SW_PLL;

	if (bCrystal)
	{
		// Save crystal frequency
		s_XtlFreq = ClkFreq;

		RCC->CR |= RCC_CR_HSEON;

		while ((RCC->CR & RCC_CR_HSERDY) == 0);

		cfgr |= RCC_CFGR_PLLSRC_HSE_PREDIV;
		uint32_t div = 1;
		int32_t cdiff = SYSTEM_CORE_CLOCK;
		uint32_t mul = 2;

		// find matching clock div/mul
		for (int i = 1; i <= 16; i++)
		{
			uint32_t clk = ClkFreq / i;

			for (int j = 2; j <= 16; j++)
			{
				uint32_t sysclk = clk * j;

				if (sysclk <= SYSTEM_CORE_CLOCK)
				{
					int diff = SYSTEM_CORE_CLOCK - sysclk;
					if (diff < cdiff)
					{
						cdiff = diff;
						div = i;
						mul = j;
					}
				}
			}
		}
		cfgr |= (mul - 2U) * RCC_CFGR_PLLMUL3;
		cfgr2 |= div - 1;
		RCC->CFGR2 = cfgr2;
	}
	else
	{
		// internal 8MHz RC
		s_XtlFreq = 0;

		cfgr |= RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12;
	}

	RCC->CFGR = cfgr;
	RCC->CIR = 0x00000000U;
	RCC->CR |= RCC_CR_PLLON;

	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	SystemCoreClockUpdate();
}

void SystemInit(void)
{
	SystemCoreClockSet(STM32F0XX_HSE_XTAL_EN, STM32F0XX_HSE_XTAL_FREQ);
}
