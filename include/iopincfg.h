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

typedef enum _iopin_res_mode {
	IOPIN_RESMODE_NONE,
	IOPIN_RESMODE_PULLUP,
	IOPIN_RESMODE_PULLDOWN,
	IOPIN_RESMODE_FOLLOW
} IOPIN_RESMODE;

typedef enum _iopin_dir {
    IOPIN_DIR_INPUT,
    IOPIN_DIR_OUTPUT
} IOPIN_DIR;

typedef enum {
	IOPIN_MODE_NORMAL = 0,
	IOPIN_MODE_OPENDRAIN = 1
} IOPIN_MODE;

typedef struct _iopin_cfg {
	int 			PortNo;		// Port number
	int 			PinNo;		// Pin number
	int 			PinOp;		// Pin function select index from 0, MCU dependent
	IOPIN_DIR		PinDir;		// Pin direction
	IOPIN_RESMODE 	ResMode;	// Pin resistor setting
	IOPIN_MODE		PinMode;
} IOPIN_CFG;

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
 *			ResMode : Resistor config
 *			PinMode : Pin mode
 *
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPIN_DIR Dir,
				 IOPIN_RESMODE ResMode, IOPIN_MODE PinMode);

/*
 * Configure I/O pin with IOPIN_CFG data structure. Can be used for batch config
 *
 * @param   pCfg   : Pointer to an array gpio pin configuration
 *          NbPins : Number of gpio pins to configure 
*/
__inline__  __attribute__((always_inline)) void IOPinCfg(IOPIN_CFG *pCfg, int NbPins) {
	if (pCfg == NULL || NbPins <= 0)
		return;

	for (int i = 0; i < NbPins; i++)
	{
		IOPinConfig(pCfg[i].PortNo, pCfg[i].PinNo, pCfg[i].PinOp, pCfg[i].PinDir,
					pCfg[i].ResMode, pCfg[i].PinMode);
	}
}


#ifdef __cplusplus
}
#endif

#endif	// __IOPINCFG_H__
