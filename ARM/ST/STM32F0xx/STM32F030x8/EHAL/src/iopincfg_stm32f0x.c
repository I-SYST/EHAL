/**-------------------------------------------------------------------------
@file	iopincfg_stm32f0xx.c

@brief	I/O pin configuration implementation on STM32F0x series

@author	Hoang Nguyen Hoan
@date	June 3, 2019

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

#include <stdio.h>
#include <stdbool.h>

#include "stm32f0xx.h"

#include "coredev/iopincfg.h"

#define IOPIN_MAX_INT			(8)

#pragma pack(push, 4)
typedef struct {
	IOPINSENSE Sense;
	IOPINEVT_CB SensEvtCB;
    uint16_t PortPinNo;
} IOPINSENS_EVTHOOK;
#pragma pack(pop)

static IOPINSENS_EVTHOOK s_GpIOSenseEvt[IOPIN_MAX_INT + 1] = { {0, NULL}, };

/**
 * @brief Configure individual I/O pin. nRF51 only have 1 port so PortNo is not used
 *
 * @Param 	PortNo	: Port number
 * 						Special port flag 0x80 (bit 7 set) for nRF91 secure port
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 						ignore for Nordic
 *
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	if (PortNo == -1 || PinNo == -1)
		return;

	uint32_t pos = PinNo << 1;

	reg->MODER &= ~(GPIO_MODER_MODER0_Msk << pos);
	if (Dir == IOPINDIR_OUTPUT)
	{
		reg->MODER |= GPIO_MODER_MODER0_0 << pos;
	}

	uint32_t pull = reg->PUPDR & ~(3 << pos);

	switch (Resistor)
	{
		case IOPINRES_FOLLOW:
		case IOPINRES_PULLUP:
			pull |= GPIO_PUPDR_PUPDR0_0 << pos;
			break;
		case IOPINRES_PULLDOWN:
			pull |=  GPIO_PUPDR_PUPDR0_1 << pos;
			break;
		case IOPINRES_NONE:
			break;
	}

	reg->OTYPER = 0;
	if (Type == IOPINTYPE_OPENDRAIN)
	{
		reg->OTYPER = (1 << PinNo);
	}

	reg->PUPDR = pull;
}

/**
 * @brief	Disable I/O pin
 *
 * Some hardware such as low power mcu allow I/O pin to be disconnected
 * in order to save power. There is no enable function. Reconfigure the
 * I/O pin to re-enable it.
 *
 * @param	PortNo 	: Port number
 * @param	PinNo	: Pin Number
 */
void IOPinDisable(int PortNo, int PinNo)
{
	if (PortNo == -1 || PinNo == -1)
		return;

	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	reg->MODER &= ~(GPIO_MODER_MODER0_Msk << (PinNo << 1));
}

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisableInterrupt(int IntNo)
{
}

/**
 * @brief Enable I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 * NOTE : Port event interrupt is set when IntNo = -1.  Port event mode only
 * high transition is detected no matter the setting of pin sense
 *
 * @param	IntNo	: Interrupt number. -1 for port event interrupt
 * 			IntPrio : Interrupt priority
 * 			PortNo  : Port number (up to 32 ports)
 * 			PinNo   : Pin number (up to 32 pins)
 * 			Sense   : Sense type of event on the I/O pin
 * 			pEvtCB	: Pointer to callback function when event occurs
 */
bool IOPinEnableInterrupt(int IntNo, int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPINEVT_CB pEvtCB)
{

    return true;
}

int IOPinFindAvailInterrupt()
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (s_GpIOSenseEvt[i].SensEvtCB == NULL)
		{
			return i;
		}
	}

	return -1;
}

/**
 * @brief	Allocate I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change. This function will automatically
 * allocate available interrupt number to use for the pin.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 *
 * @Param	IntPrio : Interrupt priority
 * @Param	PortNo  : Port number (up to 32 ports)
 * @Param	PinNo   : Pin number (up to 32 pins)
 * @Param	Sense   : Sense type of event on the I/O pin
 * @Param	pEvtCB	: Pointer to callback function when event occurs
 *
 * @return	Interrupt number on success
 * 			-1 on failure.
 */
int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPINEVT_CB pEvtCB)
{
	int intno = IOPinFindAvailInterrupt();

	if (intno >= 0)
	{
		bool res = IOPinEnableInterrupt(intno, IntPrio, PortNo, PinNo, Sense, pEvtCB);
		if (res == true)
			return intno;
	}

	return -1;
}

/**
 * @brief Set I/O pin sensing option
 *
 * Some hardware allow pin sensing to wake up or active other subsystem without
 * requiring enabling interrupts. This requires the I/O already configured
 *
 * @param	PortNo : Port number (up to 32 ports)
 * 			PinNo   : Pin number (up to 32 pins)
 * 			Sense   : Sense type of event on the I/O pin
 */
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
}

/**
 * @brief Set I/O pin drive strength option
 *
 * Some hardware allow setting pin drive strength. This requires the I/O already configured
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * 			PinNo  	: Pin number (up to 32 pins)
 * 			Strength: Pin drive strength
 */
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
}




