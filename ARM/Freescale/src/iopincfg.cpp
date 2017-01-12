/*--------------------------------------------------------------------------
File   : iopincfg.c

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : Generic I/O pin config
         Freescale KL26Z implementation

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
#include "fsl_port_hal.h"
#include "fsl_gpio_hal.h"

#include "iopincfg.h"

PORT_Type *g_PortBase[] = PORT_BASE_PTRS;
int g_NbIOPort = sizeof(g_PortBase)/sizeof(PORT_Type *);

GPIO_Type *g_GpioBase[] = GPIO_BASE_PTRS;
int g_NbGpioPort = sizeof(g_GpioBase)/sizeof(GPIO_Type *);

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	PORT_Type *base;

	if (PortNo >= g_NbIOPort || PortNo < 0 || PinNo <0)
		return;

	base = g_PortBase[PortNo];

	PORT_HAL_SetMuxMode(base, PinNo, (port_mux_t)PinOp);

	switch(Resistor)
	{
		case IOPINRES_NONE:
			PORT_HAL_SetPullCmd(base, PinNo, false);
			break;
		case IOPINRES_PULLUP:
			PORT_HAL_SetPullCmd(base, PinNo, true);
			PORT_HAL_SetPullMode(base, PinNo, kPortPullUp);
			break;
		case IOPINRES_PULLDOWN:
			PORT_HAL_SetPullCmd(base, PinNo, true);
			PORT_HAL_SetPullMode(base, PinNo, kPortPullDown);
			break;
		case IOPINRES_FOLLOW:
			break;
	}

	GPIO_Type *gpiobase = g_GpioBase[PortNo];

	if (Dir == IOPINDIR_OUTPUT)
	{
		GPIO_HAL_SetPinDir(gpiobase, PinNo, kGpioDigitalOutput);
	}
	else
	{
		GPIO_HAL_SetPinDir(gpiobase, PinNo, kGpioDigitalInput);
	}
}




