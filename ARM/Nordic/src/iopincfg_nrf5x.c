/*--------------------------------------------------------------------------
File   : iopincfg_nrf51.c

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : Generic I/O pin config

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
#include <stdbool.h>

#ifdef NRF51
#include "nrf51.h"
#include "nrf51_bitfields.h"
#else
#include "nrf52.h"
#include "nrf52_bitfields.h"
#define NRF_GPIO		NRF_P0
#endif
#include "nrf_gpiote.h"

#include "iopincfg.h"

#define IOPIN_MAX_INT			(GPIOTE_CH_NUM)

typedef struct {
	IOPINSENSE Sense;
	IOPINEVT_CB SensEvtCB;
} IOPINSENS_EVTHOOK;

IOPINSENS_EVTHOOK s_GpIOSenseEvt[IOPIN_MAX_INT] = { {0, NULL}, };

/*
 * Configure individual I/O pin. nRF51 only have 1 port so PortNo is not used
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 * 			PinOp	: Pin function index from 0. MCU dependent
 * 			Dir     : I/O direction
 *			Resistor : Resistor config
 *			Type	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	uint32_t cnf = 0;

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

	NRF_GPIO->PIN_CNF[PinNo] = cnf;
}

void IOPinDisbleInterrupt(int IntNo)
{
	if (IntNo >= 0 && IntNo < 8)
	{
		NRF_GPIOTE->CONFIG[IntNo] = 0;
		s_GpIOSenseEvt[IntNo].SensEvtCB = NULL;
	}
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPINEVT_CB pEvtCB)
{
	if (IntNo < 0 || IntNo >= IOPIN_MAX_INT)
		return false;

	//NRF_GPIOTE->CONFIG[IntNo] &= ~(GPIOTE_CONFIG_PORT_PIN_Msk | GPIOTE_CONFIG_POLARITY_Msk);
	switch (Sense)
	{
		case IOPINSENSE_LOW_TRANSITION:
			NRF_GPIOTE->CONFIG[IntNo] = ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
					                    | ((PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
					                    | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
			NRF_GPIO->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			NRF_GPIOTE->CONFIG[IntNo] = ((GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
					                    | ((PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
					                    | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
			NRF_GPIO->PIN_CNF[PinNo] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
			break;
		case IOPINSENSE_TOGGLE:
			NRF_GPIOTE->CONFIG[IntNo] = ((GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk)
					                    | ((PinNo << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PORT_PIN_Msk)
					                    | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
			NRF_GPIO->PIN_CNF[PinNo] |= (3 << GPIO_PIN_CNF_SENSE_Pos);
			break;
	}

	s_GpIOSenseEvt[IntNo].SensEvtCB = pEvtCB;

	NRF_GPIOTE->INTENCLR = 0xFFFFFFFF;

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, IntPrio);
    NVIC_EnableIRQ(GPIOTE_IRQn);

    NRF_GPIOTE->EVENTS_PORT = 0;

    NRF_GPIOTE->INTENSET |= (1 << IntNo);
    NRF_GPIOTE->INTENSET |= GPIOTE_INTENSET_PORT_Msk;

    return true;
}

void GPIOTE_IRQHandler(void)
{
	for (int i = 0; i < IOPIN_MAX_INT; i++)
	{
		if (NRF_GPIOTE->EVENTS_IN[i] || NRF_GPIOTE->EVENTS_PORT)
		{
			if (s_GpIOSenseEvt[i].SensEvtCB)
				s_GpIOSenseEvt[i].SensEvtCB(i);
			NRF_GPIOTE->EVENTS_IN[i] = 0;
		}
	}
	NRF_GPIOTE->EVENTS_PORT = 0;
}

