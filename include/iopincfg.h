/*--------------------------------------------------------------------------
File   : iopincfg.h

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : Generic I/O pin config

Copyright (c) 2011, I-SYST inc., all rights reserved

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
#ifndef __IOPINCFG_H__
#define __IOPINCFG_H__

#include <stdint.h>
#include <stdio.h>

// I/O pin resistor config
typedef enum _iopin_resistor {
	IOPINRES_NONE,
	IOPINRES_PULLUP,
	IOPINRES_PULLDOWN,
	IOPINRES_FOLLOW
} IOPINRES;

// I/O pin direction config
typedef enum _iopin_dir {
    IOPINDIR_INPUT = 0,
    IOPINDIR_OUTPUT = 1,
    IOPINDIR_BI = 2,		// Bidirectional
} IOPINDIR;

// I/O pin type
typedef enum {
	IOPINTYPE_NORMAL = 0,
	IOPINTYPE_OPENDRAIN = 1
} IOPINTYPE;

#pragma pack(push,4)

typedef struct _iopin_cfg {
	int 		PortNo;		// Port number
	int 		PinNo;		// Pin number
	int 		PinOp;		// Pin function select index from 0, MCU dependent
	IOPINDIR	PinDir;		// Pin direction
	IOPINRES 	Res;		// Pin resistor setting
	IOPINTYPE	Type;		// I/O type
} IOPINCFG;

#pragma pack(pop)

#ifdef 	__cplusplus
extern "C" {
#endif

/*
 * Configure individual I/O pin.
 *
 * Note : This function is MCU dependent. Needs to be implemented per MCU
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 			Dir     : I/O direction
 *			Resistor : Resistor config
 *			Type 	: I/O type
 *
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type);

/*
 * Configure I/O pin with IOPIN_CFG data structure. Can be used for batch config
 *
 * @param   pCfg   : Pointer to an array gpio pin configuration
 *          NbPins : Number of gpio pins to configure 
*/
inline __attribute__((always_inline)) void IOPinCfg(const IOPINCFG *pCfg, int NbPins) {
	if (pCfg == NULL || NbPins <= 0)
		return;

	for (int i = 0; i < NbPins; i++)
	{
		IOPinConfig(pCfg[i].PortNo, pCfg[i].PinNo, pCfg[i].PinOp, pCfg[i].PinDir,
					pCfg[i].Res, pCfg[i].Type);
	}
}


#ifdef __cplusplus
}
#endif

#endif	// __IOPINCFG_H__
