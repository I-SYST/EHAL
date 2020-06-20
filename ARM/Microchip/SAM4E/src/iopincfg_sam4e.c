/**-------------------------------------------------------------------------
@file	iopincfg_sam4e.c

@brief	I/O pin configuration implementation on SAM4E series

The SAM4 has one interrupt handler for each GPIO port.

@author	Hoang Nguyen Hoan
@date	June 1, 2020

@license

Copyright (c) 2020, I-SYST inc., all rights reserved

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

#include "sam4e.h"
#include "coredev/iopincfg.h"

#define IOPIN_MAX_PINOP			(5)

#if defined(_SAM4E_PIOE_INSTANCE_)
	#define IOPIN_MAX_PORT		(5)
#elif defined(_SAM4E_PIOD_INSTANCE_)
	#define IOPIN_MAX_PORT		(4)
#elif defined(_SAM4E_PIOC_INSTANCE_)
	#define IOPIN_MAX_PORT		(3)
#else
	#define IOPIN_MAX_PORT		(2)
#endif

#define IOPIN_MAX_INT			(IOPIN_MAX_PORT)

#pragma pack(push, 4)
typedef struct {
	IOPINSENSE Sense;
	IOPINEVT_CB SensEvtCB;
    uint16_t PortPinNo;
} IOPINSENS_EVTHOOK;
#pragma pack(pop)

static IOPINSENS_EVTHOOK s_GpIOSenseEvt[IOPIN_MAX_INT + 1] = { {0, NULL}, };

/**
 * @brief Configure individual I/O pin.
 *
 * @Param 	PortNo	: Port number
 * 						SAM4 ports are named A, B, C,...
 * 							0 = A, 1 = B, ...
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 						for SAM4
 * 						0 :	GPIO mode
 * 						1-4 : Peripheral A-D
 *
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	if (PortNo == -1 || PinNo == -1 || PortNo > IOPIN_MAX_PORT)
		return;

	uint32_t pinmask = 1 << PinNo;

	// Enable peripheral clock
	SAM4E_PMC->PMC_PCER0 |= 1 << (9 + PortNo);

	if (PinOp > 0 && PinOp <= IOPIN_MAX_PINOP)
	{
		// Configure as peripheral
		reg->PIO_PDR |= pinmask;
		
		uint8_t psel = PinOp - 1;
		
		if (psel & 1)
		{
			reg->PIO_ABCDSR[0] |= pinmask;
		}
		else
		{
			reg->PIO_ABCDSR[0] &= ~pinmask;
		}
		
		if (psel & 2)
		{
			reg->PIO_ABCDSR[1] |= pinmask;
		}
		else
		{
			reg->PIO_ABCDSR[1] &= ~pinmask;
		}
	}
	else
	{
		// Configure as GPIO	
		reg->PIO_PER |= pinmask;
		
		if (Dir == IOPINDIR_OUTPUT)
		{
			reg->PIO_OER |= pinmask;
			reg->PIO_OWER |= pinmask;
		}
		else
		{
			reg->PIO_ODR &= pinmask;
			reg->PIO_OWDR |= pinmask;
		}
	}
	
	reg->PIO_PUDR |= pinmask;
	reg->PIO_PPDDR |= pinmask;

	switch (Resistor)
	{
		case IOPINRES_FOLLOW:
		case IOPINRES_PULLUP:
			reg->PIO_PUER |= pinmask;
			break;
		case IOPINRES_PULLDOWN:
			reg->PIO_PPDER |= pinmask;
			break;
		case IOPINRES_NONE:
			break;
	}

	if (Type == IOPINTYPE_OPENDRAIN)
	{
		reg->PIO_MDER |= pinmask;
	}
	else
	{
		reg->PIO_MDDR |= pinmask;
	}
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

	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);
	reg->PIO_PDR |= 1 << PinNo;
}

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisableInterrupt(int IntNo)
{
	if (IntNo < 0 || IntNo >= IOPIN_MAX_INT)
	{
		return;
	}

	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + ((s_GpIOSenseEvt[IntNo].PortPinNo >> 8) & 0xFF) * 0x200);

	reg->PIO_IDR |= 1 << (s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF);
	
    s_GpIOSenseEvt[IntNo].PortPinNo = -1;
    s_GpIOSenseEvt[IntNo].Sense = IOPINSENSE_DISABLE;
    s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;

    switch (IntNo)
    {
    	case 0:
    		NVIC_DisableIRQ(PIOA_IRQn);
    		NVIC_ClearPendingIRQ(PIOA_IRQn);
    		break;
    	case 1:
    		NVIC_DisableIRQ(PIOB_IRQn);
    		NVIC_ClearPendingIRQ(PIOB_IRQn);
    		break;
    	case 2:
    		NVIC_DisableIRQ(PIOC_IRQn);
    		NVIC_ClearPendingIRQ(PIOC_IRQn);
    		break;
    	case 3:
    		NVIC_DisableIRQ(PIOD_IRQn);
    		NVIC_ClearPendingIRQ(PIOD_IRQn);
    		break;
    	case 4:
    		NVIC_DisableIRQ(PIOE_IRQn);
    		NVIC_ClearPendingIRQ(PIOE_IRQn);
    		break;
    }
}

/**
 * @brief Enable I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 * STM32 : IntNo must be same as PinNo.  It is directly related to hardware.
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
	if (IntNo < 0 || IntNo >= IOPIN_MAX_INT || IntNo != PortNo)
	{
		return false;
	}

	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	IOPinSetSense(PortNo, PinNo, Sense);
	
    s_GpIOSenseEvt[IntNo].Sense = Sense;
	s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | PinNo; // For use when disable interrupt
	s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;

	reg->PIO_IER = 1 << PinNo; 
	
	switch (IntNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(PIOA_IRQn);
			NVIC_SetPriority(PIOA_IRQn, IntPrio);
			NVIC_EnableIRQ(PIOA_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(PIOB_IRQn);
			NVIC_SetPriority(PIOB_IRQn, IntPrio);
			NVIC_EnableIRQ(PIOB_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(PIOC_IRQn);
			NVIC_SetPriority(PIOC_IRQn, IntPrio);
			NVIC_EnableIRQ(PIOC_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(PIOD_IRQn);
			NVIC_SetPriority(PIOD_IRQn, IntPrio);
			NVIC_EnableIRQ(PIOD_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(PIOE_IRQn);
			NVIC_SetPriority(PIOE_IRQn, IntPrio);
			NVIC_EnableIRQ(PIOE_IRQn);
			break;
    }

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
	if (PortNo < 0 || PinNo < 0 || PortNo > IOPIN_MAX_PORT || PinNo > 31)
	{
		return;
	}
	
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);
	uint32_t pinmask = 1 << PinNo;
	
	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			reg->PIO_FELLSR |= pinmask;
			reg->PIO_ESR |= pinmask;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			reg->PIO_REHLSR |= pinmask;
			reg->PIO_ESR |= pinmask;
			break;
		case IOPINSENSE_TOGGLE:
			reg->PIO_AIMDR |= pinmask;
			break;
	}
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
	// Not available
}

/**
 * @brief Set I/O pin speed option
 *
 * Some hardware allow setting pin speed. This requires the I/O already configured
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @Param	PinNo  	: Pin number (up to 32 pins)
 * @Param	Speed	: Pin speed
 */
void IOPinSetSpeed(int PortNo, int PinNo, IOPINSPEED Speed)
{
	// Not avail
}

void PIOA_Handler(void)
{
	uint32_t status = SAM4E_PIOA->PIO_ISR;
	
	if (status)
	{
		if (s_GpIOSenseEvt[0].SensEvtCB)
			s_GpIOSenseEvt[0].SensEvtCB(status);
	}

	NVIC_ClearPendingIRQ(PIOA_IRQn);
}

void PIOB_Handler(void)
{
	uint32_t status = SAM4E_PIOA->PIO_ISR;
	
	if (status)
	{
		if (s_GpIOSenseEvt[1].SensEvtCB)
			s_GpIOSenseEvt[1].SensEvtCB(status);
	}

	NVIC_ClearPendingIRQ(PIOB_IRQn);
}

void PIOC_Handler(void)
{
	uint32_t status = SAM4E_PIOA->PIO_ISR;
	
	if (status)
	{
		if (s_GpIOSenseEvt[2].SensEvtCB)
			s_GpIOSenseEvt[2].SensEvtCB(status);
	}

	NVIC_ClearPendingIRQ(PIOC_IRQn);
}

void PIOD_Handler(void)
{
	uint32_t status = SAM4E_PIOA->PIO_ISR;
	
	if (status)
	{
		if (s_GpIOSenseEvt[3].SensEvtCB)
			s_GpIOSenseEvt[3].SensEvtCB(status);
	}

	NVIC_ClearPendingIRQ(PIOD_IRQn);
}

void PIOE_Handler(void)
{
	uint32_t status = SAM4E_PIOA->PIO_ISR;
	
	if (status)
	{
		if (s_GpIOSenseEvt[4].SensEvtCB)
			s_GpIOSenseEvt[4].SensEvtCB(status);
	}

	NVIC_ClearPendingIRQ(PIOE_IRQn);
}

