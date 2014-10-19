/*--------------------------------------------------------------------------
File   : iopincfg.c

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

#include <stdio.h>
#include "LPC17xx.h"

#include "iopincfg.h"

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
	uint32_t *pinselreg = (uint32_t *)&LPC_PINCON->PINSEL0 + (PortNo << 1);
	uint32_t *pinmodereg = (uint32_t *)&LPC_PINCON->PINMODE0 + (PortNo << 1);
	uint32_t *pinodreg = (uint32_t *)&LPC_PINCON->PINMODE_OD0 + PortNo;

	// Configure direction
	if (Dir == IOPINDIR_OUTPUT)
		LPC_GPIO0[PortNo].FIODIR |= (1 << PinNo);
	else
		LPC_GPIO0[PortNo].FIODIR &= ~(1 << PinNo);

	// Configure open drain
	*pinodreg &= ~(1 << PinNo);
	if (Type == IOPINTYPE_OPENDRAIN)
		*pinodreg |= (1 << PinNo);

	if (PinNo > 15)
	{
		pinselreg++;
		pinmodereg++;
		PinNo -= 16;
	}

	PinNo <<= 1;

	// Configure pin function
	*pinselreg &= ~(3 << PinNo);
	*pinselreg |= (PinOp & 3) << PinNo;

	// Configure pin resistor
	*pinmodereg &= ~(3 << PinNo);

	int rmode = 0;
	switch (Resistor)
	{
		case IOPINRES_NONE:
			rmode = 2;
			break;
		case IOPINRES_PULLUP:
			rmode = 0;
			break;
		case IOPINRES_PULLDOWN:
			rmode = 3;
			break;
		case IOPINRES_FOLLOW:
			rmode = 1;
			break;
	}
	*pinmodereg |= (rmode & 3) << PinNo;
}




