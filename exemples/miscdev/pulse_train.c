/**-------------------------------------------------------------------------
@example	pulse_train.c

@brief	Pulse train example

This example shows how to use GPIO to generate a pulse train.
A pulse moving from 1 GPIO to the next

@author	Hoang Nguyen Hoan
@date	Aug. 31, 2014

@license

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

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "pulse_train.h"

#include "board.h"

/// Pulse train config
static const PULSE_TRAIN_PIN s_PulseTrainPins[] = PULSE_TRAIN_PINS_MAP;

PULSE_TRAIN_CFG g_PulseTrainCfg = {
	.pPins = (PULSE_TRAIN_PIN *)s_PulseTrainPins,
	.NbPins = sizeof(s_PulseTrainPins) / sizeof(PULSE_TRAIN_PIN),
	.Period = 1,
	.Pol = PULSE_TRAIN_POL_HIGH
};

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	// Pulse train forever
	PulseTrain(&g_PulseTrainCfg, 0);

	return 0;
}


