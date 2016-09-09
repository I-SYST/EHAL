/*--------------------------------------------------------------------------
File   : lpc11Uxx_iopincfg.c

Author : Hoang Nguyen Hoan          Oct. 25, 2014

Desc   : I/O pin config implementation on LPC11Uxx

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/

#include <stdio.h>
#include "LPC11Uxx.h"

#include "iopincfg.h"

// I/O Pin configuration bits definitions
#define IOCON_PIN_FUNCTION			0
#define IOCON_PIN_FUNCTION_MASK		3
#define IOCON_PIN_MODE				3
#define IOCON_PIN_MODE_MASK			3
#define IOCON_PIN_OPENDRAIN			10
#define IOCON_PIN_HYS				5

static uint32_t g_PinPortOffset[3] = {
	0, 0x60, 0xf0
};

/*
 * Configure individual I/O pin
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 			Dir     : I/O direction
 *			Res 	: Resistor config
 *			Type	: I/O type
 *
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	if (PortNo < 0 || PinNo < 0)
		return;

	uint32_t *pincfgreg = (uint32_t *)((uint32_t)LPC_IOCON  + g_PinPortOffset[PortNo]);

	// Configure direction
	if (Dir == IOPINDIR_OUTPUT)
		LPC_GPIO->DIR[PortNo] |= (1 << PinNo);
	else
		LPC_GPIO->DIR[PortNo] &= ~(1 << PinNo);

	pincfgreg += PinNo;

	// Configure open drain
	*pincfgreg &= ~(1 << IOCON_PIN_OPENDRAIN);
	if (Type == IOPINTYPE_OPENDRAIN)
		*pincfgreg |= (1 << IOCON_PIN_OPENDRAIN);

	// Configure pin function
	*pincfgreg &= ~(3 << IOCON_PIN_FUNCTION);
	*pincfgreg |= (PinOp & 3) << IOCON_PIN_FUNCTION;

	// Configure pin resistor
	int rmode = 0;
	switch (Resistor)
	{
		case IOPINRES_NONE:
			rmode = 0;
			break;
		case IOPINRES_PULLUP:
			rmode = 2;
			break;
		case IOPINRES_PULLDOWN:
			rmode = 1;
			break;
		case IOPINRES_FOLLOW:
			rmode = 3;
			break;
	}
	*pincfgreg &= ~(3 << IOCON_PIN_MODE);
	*pincfgreg |= (rmode & 3) << IOCON_PIN_MODE;
	*pincfgreg |= 1 << IOCON_PIN_HYS;
}




