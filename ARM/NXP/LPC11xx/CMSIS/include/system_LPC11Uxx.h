/*--------------------------------------------------------------------------
File   : system_LPC11Uxx.h

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

#ifndef __SYSTEM_LPC11UXX_H__
#define __SYSTEM_LPC11UXX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define XTAL_FREQ_16MHZ			16000000UL
#define XTAL_FREQ_12MHZ			12000000UL

#define PDRUNCFG_IRCOUT_PD		1		// IRC oscillator output power-down
#define PDRUNCFG_IRC_PD			2		// IRC oscillator power-down#define
#define PDRUNCFG_FLASH_PD		4		// Flash power-down
#define PDRUNCFG_BOD_PD			8		// BOD power-down
#define PDRUNCFG_ADC_PD			0x10	// ADC power-down
#define PDRUNCFG_SYSOSC_PD		0x20	// Crystal oscillator power-down Powered
#define PDRUNCFG_WDTOSC_PD		0x40	// Watchdog oscillator power-down
#define PDRUNCFG_SYSPLL_PD		0x80	// System PLL power-down
#define PDRUNCFG_USBPLL_PD		0x100	// USB PLL power-down
#define PDRUNCFG_USBPAD_PD		0x400	// USB transceiver power-down configuration

#define SYSOSCCTRL_BYPASS		1
#define SYSOSCCTRL_FREQRANGE0	0		// 0 - 20 MHz
#define SYSOSCCTRL_FREQRANGE15	2		// 15 - 25 MHz


#define MAINCLKSEL_MASK			3		// Clcok selec mask
#define MAINCLKSEL_IRC			0		// IRC Oscillator
#define MAINCLKSEL_PLLIN		1		// PLL input
#define MAINCLKSEL_WDT			2		// Watchdog oscillator
#define MAINCLKSEL_PLLOUT		3		// PLL output

#define SYSPLLCLKSEL_MASK		3
#define SYSPLLCLKSEL_IRC		0		// IRC
#define SYSPLLCLKSEL_SYSOSC		1		// Crystal Oscillator (SYSOSC)

#define USBPLLCLKSEL_MASK		3
#define USBPLLCLKSEL_IRC		0
#define USBPLLCLKSEL_SYSOSC		1

#define USBCLKSEL_PLLOUT		0
#define USBCLKSEL_MAINCLK		1

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
void SystemInit (void);

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

uint32_t SystemSetCoreClock(bool Crystal, int ClkFreq);

#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_LPC11UXX_H__

