/*--------------------------------------------------------------------------
File   : iopinctrl.h

Author : Hoang Nguyen Hoan          Nov.20, 2011

Desc   : General I/O pin control implementation specific 
		 This file must be named iopinctrl.h no matter which target

		 This is Freescale implementation

Copyright (c) 2011, I-SYST inc., all rights reserved
Copyright (c) 2015, Motsai, all rights reserved

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

#include "iopincfg.h"
#include "fsl_gpio_hal.h"

extern GPIO_Type *g_GpioBase[];

static inline void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
    if (Dir == IOPINDIR_OUTPUT)
        GPIO_HAL_SetPinDir(g_GpioBase[PortNo], PinNo, kGpioDigitalOutput);
    else if (Dir == IOPINDIR_INPUT)
        GPIO_HAL_SetPinDir(g_GpioBase[PortNo], PinNo, kGpioDigitalInput);
}

static inline int IOPinGetPin(int PortNo, int PinNo)
{
    return GPIO_HAL_ReadPinOutput(g_GpioBase[PortNo], PinNo);
}

static inline void IOPinSet(int PortNo, int PinNo)
{
    GPIO_HAL_SetPinOutput(g_GpioBase[PortNo], PinNo);
}

static inline void IOPinClear(int PortNo, int PinNo)
{
    GPIO_HAL_ClearPinOutput(g_GpioBase[PortNo], PinNo);
}

static inline void IOPinToggle(int PortNo, int PinNo)
{
    GPIO_HAL_TogglePinOutput(g_GpioBase[PortNo], PinNo);
}

static inline int IOPinReadPort(int PortNo)
{
    return GPIO_HAL_ReadPortOutput(g_GpioBase[PortNo]);
}

static inline void IOPinWritePort(int PortNo, int Data)
{
    GPIO_HAL_WritePortOutput(g_GpioBase[PortNo], Data);
}


#endif	// __IOPINCTRL_H__
