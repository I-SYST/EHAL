/*--------------------------------------------------------------------------
File   : system_LPC17xx.h

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
#ifndef __SYSTEM_LPC17xx_H__
#define __SYSTEM_LPC17xx_H__

#include <stdint.h>

/* system clock (sysclk) selections */
#define CLKSRCSEL_IRC		0	/* Internal RC oscillator */
#define CLKSRCSEL_MAIN		1	/* Main oscillator */
#define CLKSRCSEL_RTC		2	/* Realtime clock oscillator */

/* Crystal or Oscillator frequencies in Hz */
#define OSC_FREQ_12MHZ			(12000000UL)    /* Oscillator frequency             */
#define OSC_FREQ_16MHZ			(16000000UL)    /* Oscillator frequency             */

/* Set target core frequence value in Hz */
#define CORE_FREQ			120000000UL

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
__attribute__((weak)) void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock 
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate (void);

/**
 *
 * @param CoreFreq 	- Core target frequency
 * @param ClkSel	- Clock source selection
 *						CLKSRCSEL_IRC - Internal osc
 *						CLKSRCSEL_MAIN - Main osc
 *						CLKSRCSEL_RTC -	Realtime osc
 *
 * @return
 * 		Return actual frequency in Hz
 *
 * @brief	Setup core clock (cclk) & update SystemCoreClock variable
 */
uint32_t SystemSetCoreClock(uint32_t CoreFreq, int ClkSel);

void SystemConnectPLL1(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_LPC17xx_H__ */
