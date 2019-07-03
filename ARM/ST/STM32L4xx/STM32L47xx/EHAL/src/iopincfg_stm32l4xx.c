/**-------------------------------------------------------------------------
@file	iopincfg_stm32l4xx.c

@brief	I/O pin configuration implementation on STM32L4x series

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

#include "stm32l4xx.h"

#include "coredev/iopincfg.h"

#define IOPIN_MAX_INT			(16)
#define IOPIN_MAX_PORT			(9)

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
 * 						STM32 ports are named A, B, C,...
 * 							0 = A, 1 = B, ...
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 						for STM32
 * 						bit 0-3 :
 * 							0 : GPIO input
 * 							1 : GPIO output
 * 							2 : Alternate function (function on bit 4-7)
 * 							3 : Analog
 * 						bit 4-7 : Alternate function 0-7
 *
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	if (PortNo == -1 || PinNo == -1 || PortNo > IOPIN_MAX_PORT)
		return;

	uint32_t tmp;

	RCC->AHB2ENR |= 1 << PortNo;

	uint32_t pos = PinNo << 1;
	tmp = reg->MODER & ~(GPIO_MODER_MODE0_Msk << pos);

	switch (PinOp & 0xF)
	{
		case 0:	// GPIO input
		case 1:	// GPIO ouput
			if (Dir == IOPINDIR_OUTPUT)
			{
				tmp |= 1 << pos;
			}

			break;
		case 2:	// Alternate function
			{
				tmp |= 2 << pos;

				pos = (PinNo & 0x7) << 2;
				int idx = PinNo >> 3;

				reg->AFR[idx] &= ~(0xf << pos);
				reg->AFR[idx] |= ((PinOp >> 4) & 0xf) << pos;

			}
			break;
		case 3:	// Analog
			tmp |= 3 << pos;
			break;
	}

	reg->MODER = tmp;

	pos = PinNo << 1;

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
	reg->PUPDR = pull;

	tmp = reg->OTYPER & ~(1 << PinNo);
	if (Type == IOPINTYPE_OPENDRAIN)
	{
		tmp |= (1 << PinNo);
	}
	reg->OTYPER = tmp;

	// Default high speed
	IOPinSetSpeed(PortNo, PinNo, IOPINSPEED_HIGH);

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
	uint32_t pos = PinNo << 1;
	uint32_t tmp = reg->MODER & ~(GPIO_MODER_MODE0_Msk << pos);
	reg->MODER = tmp;

	tmp= reg->PUPDR & ~(GPIO_PUPDR_PUPD0_Msk << pos);
	reg->PUPDR = tmp;
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

	int idx = (s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF) >> 2;
	uint32_t pos = (s_GpIOSenseEvt[IntNo].PortPinNo & 0xF) << 2;
	uint32_t mask = 0xF << pos;

	SYSCFG->EXTICR[idx] &= ~mask;

	mask = ~(1 << (s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF));

	EXTI->RTSR1 &= mask;
	EXTI->FTSR1 &= mask;
	EXTI->IMR1 &= mask;

    s_GpIOSenseEvt[IntNo].PortPinNo = -1;
    s_GpIOSenseEvt[IntNo].Sense = IOPINSENSE_DISABLE;
    s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;

    switch (IntNo)
    {
    	case 0:
    		NVIC_DisableIRQ(EXTI0_IRQn);
    		NVIC_ClearPendingIRQ(EXTI0_IRQn);
    		break;
    	case 1:
    		NVIC_DisableIRQ(EXTI1_IRQn);
    		NVIC_ClearPendingIRQ(EXTI1_IRQn);
    		break;
    	case 2:
    		NVIC_DisableIRQ(EXTI2_IRQn);
    		NVIC_ClearPendingIRQ(EXTI2_IRQn);
    		break;
    	case 3:
    		NVIC_DisableIRQ(EXTI3_IRQn);
    		NVIC_ClearPendingIRQ(EXTI3_IRQn);
    		break;
    	case 4:
    		NVIC_DisableIRQ(EXTI4_IRQn);
    		NVIC_ClearPendingIRQ(EXTI4_IRQn);
    		break;
    	default:
    		if (IntNo > 4 && IntNo < 10)
    		{
				// Shared interrupt, make sure no one still using it
    			for (int i = 5; i < 10; i++)
    			{
    				if (s_GpIOSenseEvt[i].SensEvtCB != NULL)
    				{
    					return;
    				}
    			}
        		NVIC_DisableIRQ(EXTI9_5_IRQn);
        		NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    		}
    		else
    		{
				// Shared interrupt, make sure no one still using it
    			for (int i = 10; i < IOPIN_MAX_INT; i++)
    			{
    				if (s_GpIOSenseEvt[i].SensEvtCB != NULL)
    				{
    					return;
    				}
    			}
        		NVIC_DisableIRQ(EXTI15_10_IRQn);
        		NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    		}
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
	if (IntNo < 0 || IntNo >= IOPIN_MAX_INT || IntNo != PinNo)
	{
		return false;
	}

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	int idx = IntNo >> 2;
	uint32_t pos = (IntNo & 0xFF) << 2;
	uint32_t mask = 0xF << pos;

	SYSCFG->EXTICR[idx] &= ~mask;
	SYSCFG->EXTICR[idx] |= PortNo << pos;

	mask = (1 << (PinNo & 0xFF));

	EXTI->IMR1 |= mask;

	mask &= 0x7DFF;

	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			EXTI->RTSR1 &= ~mask;
			EXTI->FTSR1 |= mask;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			EXTI->RTSR1 |= mask;
			EXTI->FTSR1 &= ~mask;
			break;
		case IOPINSENSE_TOGGLE:
			EXTI->RTSR1 |= mask;
			EXTI->FTSR1 |= mask;
			break;
	}

    s_GpIOSenseEvt[IntNo].Sense = Sense;
	s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | PinNo; // For use when disable interrupt
	s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;

	switch (IntNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(EXTI0_IRQn);
			NVIC_SetPriority(EXTI0_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(EXTI1_IRQn);
			NVIC_SetPriority(EXTI1_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(EXTI2_IRQn);
			NVIC_SetPriority(EXTI2_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI2_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(EXTI3_IRQn);
			NVIC_SetPriority(EXTI3_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI3_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(EXTI4_IRQn);
			NVIC_SetPriority(EXTI4_IRQn, IntPrio);
			NVIC_EnableIRQ(EXTI4_IRQn);
			break;
		default:
			if (IntNo > 4 && IntNo < 10)
			{
				NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
				NVIC_SetPriority(EXTI9_5_IRQn, IntPrio);
				NVIC_EnableIRQ(EXTI9_5_IRQn);
			}
			else
			{
				NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
				NVIC_SetPriority(EXTI15_10_IRQn, IntPrio);
				NVIC_EnableIRQ(EXTI15_10_IRQn);
			}
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
	// Pin sense is not avail on this STM32.  It only sets in interrupt
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
	// Not available on this STM32
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
	if (PortNo == -1 || PinNo == -1)
		return;

	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);
	uint32_t pos = PinNo << 1;
	uint32_t tmp = reg->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED0_Msk << pos);

	switch (Speed)
	{
		case IOPINSPEED_LOW:
			break;
		case IOPINSPEED_MEDIUM:
			tmp |= (1 << pos);
			break;
		case IOPINSPEED_HIGH:
			tmp |= (2 << pos);
			break;
	}

	reg->OSPEEDR = tmp;
}

void __WEAK EXTI0_IRQHandler(void)
{
	if (EXTI->PR1 & 1)
	{
		EXTI->PR1 = 1;

		if (s_GpIOSenseEvt[0].SensEvtCB)
			s_GpIOSenseEvt[0].SensEvtCB(0);

	}

	NVIC_ClearPendingIRQ(EXTI0_IRQn);
}

void __WEAK EXTI1_IRQHandler(void)
{
	if (EXTI->PR1 & 2)
	{
		EXTI->PR1 = 2;

		if (s_GpIOSenseEvt[1].SensEvtCB)
			s_GpIOSenseEvt[1].SensEvtCB(1);

	}

	NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void __WEAK EXTI2_IRQHandler(void)
{
	if (EXTI->PR1 & 4)
	{
		EXTI->PR1 = 4;

		if (s_GpIOSenseEvt[2].SensEvtCB)
			s_GpIOSenseEvt[2].SensEvtCB(2);

	}

	NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

void __WEAK EXTI3_IRQHandler(void)
{
	if (EXTI->PR1 & 8)
	{
		EXTI->PR1 = 8;

		if (s_GpIOSenseEvt[3].SensEvtCB)
			s_GpIOSenseEvt[3].SensEvtCB(3);

	}

	NVIC_ClearPendingIRQ(EXTI3_IRQn);
}

void __WEAK EXTI4_IRQHandler(void)
{
	if (EXTI->PR1 & 0x10)
	{
		EXTI->PR1 = 0x10;

		if (s_GpIOSenseEvt[4].SensEvtCB)
			s_GpIOSenseEvt[4].SensEvtCB(4);

	}

	NVIC_ClearPendingIRQ(EXTI4_IRQn);
}

void __WEAK EXTI9_5_IRQHandler(void)
{
	uint32_t mask = 1 << 5;

	for (int i = 5; i < 10; i++)
	{
		if (EXTI->PR1 & mask)
		{
			EXTI->PR1 = mask;
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i);

		}
		mask <<= 1;
	}

	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

void __WEAK EXTI15_10_IRQHandler(void)
{
	uint32_t mask = 1 << 10;

	for (int i = 10; i < IOPIN_MAX_INT; i++)
	{
		if (EXTI->PR1 & mask)
		{
			EXTI->PR1 = mask;
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i);

		}
		mask <<= 1;
	}

	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}


