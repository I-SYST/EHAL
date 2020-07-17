/**-------------------------------------------------------------------------
@file	system_core_clock.h

@brief	Contains generic system clock definitions and settings

These functions can be implemented per target.

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

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include <stdint.h>

#pragma pack(push, 4)

///
/// Enum defining clock oscillator types
///
/// Many integrated circuits allow the section of different type of oscillator to use as clock
/// source. This enum defines commonly used types.
typedef enum __Osc_Type {
	OSC_TYPE_RC,	//!< internal RC
	OSC_TYPE_XTAL,	//!< external crystal
	OSC_TYPE_TCXO,	//!< external oscillator
} OSC_TYPE;

///
/// This structure defines the the MCU oscillators
///
/// Most MCU has 2 oscillator.
/// The core oscillator often referred to as high frequency or main or core
/// The low frequency is often a 32768Hz oscillator for realtime clock or low power clock
///
/// Modern compiler such as the GNU GCC allow declaration of weak type global variable
/// that can be overloaded.  In the library system startup code contains a weak global
/// variable '__WEAK MCU_OSC g_McuOsc' that define a default oscillator.  Firmware can
/// overload this variable in the main firmware with the oscillator selection for the
/// intended board.
///
typedef struct __Mcu_Osc {
	OSC_TYPE HFType;		//!< Core, high frequency oscillator type
	uint32_t HFFreq;		//!< Core, high frequency oscillator frequency in Hz
	OSC_TYPE LFType;		//!< Low frequency oscillator type
	uint32_t LFFreq;		//!< Low frequency oscillator frequency in Hz (usually 32768)
} MCU_OSC;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Select core clock oscillator type
 *
 * @param	ClkSrc : Clock source selection
 *						OSC_TYPE_RC - Internal RC
 *						OSC_TYPE_XTAL - External crystal
 *						OSC_TYPE_CTXO -	External oscillator
 * @param	OscFreq : Oscillator frequency
 *
 * @return	true - success
 *
 */
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq);

/**
 * @brief	Select low frequency clock oscillator type
 *
 * @param	ClkSrc : Clock source selection
 *						OSC_TYPE_RC - Internal RC
 *						OSC_TYPE_XTAL - External crystal
 *						OSC_TYPE_CTXO -	External oscillator
 * @param	OscFreq : Oscillator frequency
 *
 * @return	true - success
 *
 */
bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq);

/**
 * @brief	Get core clock frequency
 *
 * @return	Core frequency in Hz.
 */
uint32_t SystemCoreClockGet();

/**
 * @brief	Get peripheral clock frequency (PCLK)
 *
 * Most often the PCLK numbering starts from 1 (PCLK1, PCLK2,...).
 * Therefore the clock Idx parameter = 0 indicate PCK1, 1 indicate PCLK2
 *
 * @param	Idx : Zero based peripheral clock number. Many processors can
 * 				  have more than 1 peripheral clock settings.
 *
 * @return	Peripheral clock frequency in Hz.
 * 			0 - Bad clock number
 */
uint32_t SystemPeriphClockGet(int Idx);

/**
 * @brief	Set peripheral clock (PCLK) frequency
 *
 * Most often the PCLK numbering starts from 1 (PCLK1, PCLK2,...).
 * Therefore the clock Idx parameter = 0 indicate PCK1, 1 indicate PCLK2
 *
 * @param	Idx  : Zero based peripheral clock number. Many processors can
 * 				   have more than 1 peripheral clock settings.
 * @param	Freq : Clock frequency in Hz.
 *
 * @return	Actual frequency set in Hz.
 * 			0 - Failed
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq);


#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_CORE_CLOCK_H__
