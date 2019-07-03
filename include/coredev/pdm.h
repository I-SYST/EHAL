/**-------------------------------------------------------------------------
@file	pdm.h

@brief	Implementation of Pulse density modulation interface


@author	Hoang Nguyen Hoan
@date	May 17, 2019

@license

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

----------------------------------------------------------------------------*/
#ifndef __PDM_H__
#define __PDM_H__

#include "device_intrf.h"

#pragma pack(push, 1)

typedef enum __PDM_OpMode {
	PDM_OPMODE_MONO,
	PDM_OPMODE_STEREO
} PDM_OPMODE;

typedef enum __PDM_SamplMode {
	PDM_SMPLMODE_FALING,
	PDM_SMPLMODE_RISING
} PDM_SMPLMODE;

typedef struct __PDM_DevInterf	PDMDEV;

typedef void (*PDMEvtHandler)(PDMDEV *pDev);

typedef struct __PDM_Config {
	uint8_t PinClk;					//!< Clock pin
	uint8_t PinDIn;					//!< Data in pin
	uint32_t Freq;					//!< PDM clock frequency
	PDM_SMPLMODE SmplMode;
	PDM_OPMODE OpMode;
	int8_t GainLeft;
	int8_t GainRight;
	bool bIntEn;					//! Interrupt enable
	int	IntPrio;					//! Interrupt priority
	PDMEvtHandler *EvtHandler;		//! Pointer to event handler
} PDM_CFG;

struct __PDM_DevInterf {
	PDM_CFG CfgData;
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool PdmInit(PDMDEV *pDev, PDM_CFG *pCfg);

#ifdef __cplusplus
}
#endif

#endif // __PDM_H__
