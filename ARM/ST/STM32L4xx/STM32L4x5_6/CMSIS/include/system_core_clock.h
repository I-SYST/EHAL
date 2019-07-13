/**-------------------------------------------------------------------------
@file	system_core_clock.h

@brief

Contains core specific default clock and related clock factors for calculating
delay loops, etc...


@author	Hoang Nguyen Hoan
@date	Aug. 30, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/

#ifndef __SYSTEM_CORE_CLOCK_H__
#define __SYSTEM_CORE_CLOCK_H__

typedef enum __System_Clock_Src {
	SYSCLK_SRC_RC,		// Internal RC
	SYSCLK_SRC_XTAL,	// External crystal
	SYSCLK_SRC_OSC,		// External oscillator
} SYSCLK_SRC;

#define SYSTEM_CORE_CLOCK				80000000UL	// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(47UL)		// TODO: Adjustment value for nanosec delay

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Setup core clock & update SystemCoreClock variable
 *
 * @param ClkSrc	- Clock source selection
 *						CLKSRCSEL_IRC - Internal osc
 *						CLKSRCSEL_MAIN - Main osc
 *						CLKSRCSEL_RTC -	Realtime osc
 * @param CoreFreq 	- Core target frequency
 *
 * @return
 * 		Return actual frequency in Hz
 *
 */
uint32_t SystemCoreClockSet(SYSCLK_SRC ClkSrc, uint32_t CoreFreq);

#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_CORE_CLOCK_H__
