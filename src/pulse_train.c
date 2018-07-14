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

#include "pulse_train.h"
#include "iopinctrl.h"
#include "idelay.h"

/**
 * Generate a pulse train on GPIO pins.
 *
 * @param	pCfg : Pointer to pulse train configuration data
 * @param	Loop : Number of time to pulse train loop
 * 					0 - loop forever, well almost forever (4 billion times).
 */
void PulseTrain(PULSE_TRAIN_CFG *pCfg, uint32_t Loop)
{
	IOPinCfg(pCfg->pPins, pCfg->NbPins);

	do {
		for (int i = 0; i < pCfg->NbPins; i++ )
		{
			IOPinToggle(pCfg->pPins[i].PortNo, pCfg->pPins[i].PinNo);
			usDelay(pCfg->Period);
			IOPinToggle(pCfg->pPins[i].PortNo, pCfg->pPins[i].PinNo);
		}
	} while (--Loop > 0);
}




