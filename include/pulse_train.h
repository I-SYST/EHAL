/**-------------------------------------------------------------------------
@file	pulse_train.h

@brief	GPIO pulse train test function.

This is a GPIO test function. It generate a pulse on each GPIO pin in turn.
It can be used to test assembly to check that all pins are properly soldered.
Check can be view with an Oscilloscope where only one GPIO can pulse at a time.

@author	Hoang Nguyen Hoan
@date	July 14, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#ifndef __PULSE_TRAIN_H__
#define __PULSE_TRAIN_H__

#include "coredev/iopincfg.h"

#pragma pack(push, 4)

/// Pulse train polarity
typedef enum __Pulse_train_polarity {
	PULSE_TRAIN_POL_LOW,		//!< Active low pulse
	PULSE_TRAIN_POL_HIGH		//!< Active high pulse
} PULSE_TRAIN_POL;

/// Pulse train GPIO pin
typedef struct __Pulse_train_pins {
	int PortNo;					//!< GPIO port number
	int PinNo;					//!< GPIO pin number
	int PinOp;					//!< Pin GPIO mode select
} PULSE_TRAIN_PIN;

/// Pulse train configuration data
typedef struct __Pulse_train_config {
	PULSE_TRAIN_PIN *pPins;		//!< IO pins array to pulse
	int NbPins;					//!< Total number of pins
	uint32_t Period;			//!< Pulse period in usec
	PULSE_TRAIN_POL Pol;		//!< Pulse train polarity
} PULSE_TRAIN_CFG;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Generate a pulse train on GPIO pins.
 *
 * @param	pCfg : Pointer to pulse train configuration data
 * @param	Loop : Number of time to pulse train loop
 * 					0 - loop forever, well almost forever (4 billion times).
 */
void PulseTrain(PULSE_TRAIN_CFG *pCfg, uint32_t Loop);

#ifdef __cplusplus
}
#endif

#endif // __PULSE_TRAIN_H__

