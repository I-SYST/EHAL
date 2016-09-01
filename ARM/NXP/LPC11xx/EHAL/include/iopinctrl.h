/*--------------------------------------------------------------------------
File   : iopinctrl.h

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : General I/O pin control implementation specific 
		 This file must be named iopinctrl.h no matter which target

		 This is LPC11xx implementation

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>
#include "LPC11Uxx.h"

static inline void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	if (Dir == IOPINDIR_OUTPUT)
		LPC_GPIO->DIR[PortNo] |= 1 << PinNo;
	else if (Dir == IOPINDIR_INPUT)
		LPC_GPIO->DIR[PortNo] &= ~(1 << PinNo);
}

static inline int IOPinRead(int PortNo, int PinNo)
{
	return ((LPC_GPIO->PIN[PortNo] >> PinNo ) & 1);
}

static inline void IOPinSet(int PortNo, int PinNo)
{
	LPC_GPIO->SET[PortNo] = (1 << PinNo);
}

static inline void IOPinClear(int PortNo, int PinNo)
{
	LPC_GPIO->CLR[PortNo] = (1 << PinNo);
}

static inline void IOPinToggle(int PortNo, int PinNo)
{
	LPC_GPIO->NOT[PortNo] = (1 << PinNo);
}

static inline uint32_t IOPinReadPort(int PortNo)
{
	return LPC_GPIO->PIN[PortNo];
}

static inline void IOPinWrite8Port(int PortNo, uint8_t Data)
{
	LPC_GPIO->B[PortNo] = Data;
}

static inline void IOPinWrite16Port(int PortNo, uint16_t Data)
{
	LPC_GPIO->B[PortNo] = Data & 0xFF;
	LPC_GPIO->B[PortNo + 1] = (Data >> 8) & 0xFF;
}

static inline void IOPinWrite32Port(int PortNo, uint32_t Data)
{
	LPC_GPIO->W[PortNo] = Data;
}

#endif	// __IOPINCTRL_H__
