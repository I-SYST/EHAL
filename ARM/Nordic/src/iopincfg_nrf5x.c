/**-------------------------------------------------------------------------
@file	iopincfg_nrf51.c

@brief	I/O pin configuration implementation on nRF5x series

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2011

@license

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

----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdbool.h>

#include "nrf.h"

#include "nrf_gpiote.h"

#include "coredev/iopincfg.h"

#define IOPIN_MAX_INT			(GPIOTE_CH_NUM)

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
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 			Dir     : I/O direction
 *			Resistor: Resistor configuration
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	uint32_t cnf = 0;
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	if (PortNo == -1 || PinNo == -1)
		return;

	if (Dir == IOPINDIR_OUTPUT)
	{
		cnf |= (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
               | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	}
	else
	{
		cnf |= (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
			   | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
	}

	switch (Resistor)
	{
		case IOPINRES_FOLLOW:	// nRF51 does not have follow mode, use pullup
		case IOPINRES_PULLUP:
			cnf |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
			break;
		case IOPINRES_PULLDOWN:
			cnf |= (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);
			break;
		case IOPINRES_NONE:
			cnf |= (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
			break;
	}

	if (Type == IOPINTYPE_OPENDRAIN)
	{
		cnf |= (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos);
	}

	reg->PIN_CNF[PinNo] = cnf;
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
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	if (PortNo == -1 || PinNo == -1)
		return;

	reg->PIN_CNF[PinNo] = (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
}

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisableInterrupt(int IntNo)
{
    if (IntNo >= IOPIN_MAX_INT)
        return;

    if (IntNo < 0)
    {
        IntNo = IOPIN_MAX_INT;
        NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_PORT_Msk;
        NRF_GPIOTE->EVENTS_PORT = 0;
    }
    else
    {
        NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
        if ((s_GpIOSenseEvt[IntNo].PortPinNo >> 8) == 1)
        {
            reg = NRF_P1;
        }

#endif

        NRF_GPIOTE->INTENCLR = (1 << IntNo);
        NRF_GPIOTE->CONFIG[IntNo] = 0;
        reg->PIN_CNF[s_GpIOSenseEvt[IntNo].PortPinNo & 0xFF] &= ~GPIO_PIN_CNF_SENSE_Msk;
    }

    s_GpIOSenseEvt[IntNo].PortPinNo = -1;
    s_GpIOSenseEvt[IntNo].Sense = IOPINSENSE_DISABLE;
    s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;

    for (int i = 0; i <= IOPIN_MAX_INT; i++)
    {
        if (s_GpIOSenseEvt[i].SensEvtCB != NULL)
            return;
    }

    NRF_GPIOTE->INTENCLR = 0xFFFFFFFF;
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_DisableIRQ(GPIOTE_IRQn);
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
    if (IntNo >= IOPIN_MAX_INT)
		return false;

	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	reg->PIN_CNF[PinNo] &= ~(GPIO_PIN_CNF_SENSE_Msk << GPIO_PIN_CNF_SENSE_Pos);
	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			NRF_GPIOTE->CONFIG[IntNo] = ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
										| ((PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
										| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
			reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			NRF_GPIOTE->CONFIG[IntNo] = ((GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
										| ((PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
										| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
			reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_TOGGLE:
			NRF_GPIOTE->CONFIG[IntNo] = ((GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
										| ((PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
										| (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
			reg->PIN_CNF[PinNo] |= (3 << GPIO_PIN_CNF_SENSE_Pos);
			break;
		default:
			;
	}

	if (IntNo < 0)
	{
		IntNo = IOPIN_MAX_INT;
		NRF_GPIOTE->EVENTS_PORT = 0;
		NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	}
	else
	{
		NRF_GPIOTE->INTENSET = (1 << IntNo);
	}

    s_GpIOSenseEvt[IntNo].Sense = Sense;
	s_GpIOSenseEvt[IntNo].PortPinNo = (PortNo << 8) | PinNo; // For use when disable interrupt
	s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, IntPrio);
    NVIC_EnableIRQ(GPIOTE_IRQn);


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
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	// Clear sense
	reg->PIN_CNF[PinNo] &= ~(GPIO_PIN_CNF_SENSE_Msk << GPIO_PIN_CNF_SENSE_Pos);
	switch (Sense)
	{
		case IOPINSENSE_DISABLE:	// Disable pin sense
			// Already done above
			break;
		case IOPINSENSE_LOW_TRANSITION:	// Event on falling edge
			reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_HIGH_TRANSITION:// Event on raising edge
			reg->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_TOGGLE:			// Event on state change
			// Not supported, use sense low for now
			reg->PIN_CNF[PinNo] |= (3 << GPIO_PIN_CNF_SENSE_Pos);
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
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	uint32_t val = ((reg->PIN_CNF[PinNo] >> GPIO_PIN_CNF_DRIVE_Pos) & GPIO_PIN_CNF_DRIVE_Msk) & 6;
	reg->PIN_CNF[PinNo] &= ~(GPIO_PIN_CNF_DRIVE_Msk << GPIO_PIN_CNF_DRIVE_Pos);
	if (Strength == IOPINSTRENGTH_STRONG)
	{
		// Stronger drive strength
		val++;
	}

	reg->PIN_CNF[PinNo] |= (val << GPIO_PIN_CNF_DRIVE_Pos);
}

void __WEAK GPIOTE_IRQHandler(void)
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE->EVENTS_IN[i])
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i);
			NRF_GPIOTE->EVENTS_IN[i] = 0;
		}
	}
	if (NRF_GPIOTE->EVENTS_PORT)
	{
        if (s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB)
            s_GpIOSenseEvt[IOPIN_MAX_INT].SensEvtCB(-1);
	    NRF_GPIOTE->EVENTS_PORT = 0;
#ifdef NRF52
	    NRF_GPIO->LATCH = 0xFFFFFFFF;	// Clear detect latch
#endif
	}

	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
}


