/**-------------------------------------------------------------------------
@file	analog_comp_nrf5x.cpp

@brief	Analog comparator implementation on nRF52


@author	Hoang Nguyen Hoan
@date	Aug. 13, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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
#include <string.h>
#include <stdio.h>

#include "nrf.h"

#include "idelay.h"
#include "analog_comp_nrf5x.h"

static ANALOG_COMP_DEV *s_pnRF5xLPCompDev = NULL;

extern "C" void COMP_LPCOMP_IRQHandler()
{
	ANALOG_COMP_EVT evt = ANALOG_COMP_EVT_UNKNOWN;

	if (NRF_LPCOMP->EVENTS_UP)
	{
		evt = ANALOG_COMP_EVT_HIGHER;
		NRF_LPCOMP->EVENTS_UP = 0;
	}

	if (NRF_LPCOMP->EVENTS_DOWN)
	{
		evt = ANALOG_COMP_EVT_LOWER;
		NRF_LPCOMP->EVENTS_DOWN = 0;
	}

	if (NRF_LPCOMP->EVENTS_READY)
	{
		NRF_LPCOMP->EVENTS_READY = 0;
	}

	if (NRF_LPCOMP->EVENTS_CROSS)
	{
		NRF_LPCOMP->EVENTS_CROSS = 0;
	}

	//printf("Res = %x\r\n", NRF_LPCOMP->RESULT);
	if (s_pnRF5xLPCompDev && s_pnRF5xLPCompDev->EvtHandler)
	{
		s_pnRF5xLPCompDev->EvtHandler(evt, s_pnRF5xLPCompDev);
	}
}

bool AnalogCompInit(ANALOG_COMP_DEV * const pDev, const ANALOG_COMP_CFG *pCfg)
{
	if (pDev == NULL)
	{
		return false;
	}

	s_pnRF5xLPCompDev = pDev;

	if (pCfg->RefSrc != 0)
	{
#ifdef NRF52
		int d = pCfg->CompVolt * 16 / pCfg->RefSrc;

		if (d & 1)
		{
			NRF_LPCOMP->REFSEL = d - 1;
		}
		else
		{
			NRF_LPCOMP->REFSEL = (d >> 1) - 1;
		}
#else
		int d = pCfg->CompVolt * 8 / pCfg->RefSrc;

		NRF_LPCOMP->REFSEL = d;
#endif
	}
	else
	{
		NRF_LPCOMP->REFSEL = 7;
	}

	NRF_LPCOMP->PSEL = pCfg->AnalogIn;

	if (pCfg->EvtHandler)
	{
		pDev->EvtHandler = pCfg->EvtHandler;
	}

	if (pCfg->bHystersys == true)
	{
		NRF_LPCOMP->HYST = 1;
	}
	else
	{
		NRF_LPCOMP->HYST = 0;
	}

	NRF_LPCOMP->SHORTS = LPCOMP_SHORTS_READY_SAMPLE_Msk;

	// Detect both Up & Down
	NRF_LPCOMP->ANADETECT = LPCOMP_ANADETECT_ANADETECT_Msk;

	((AnalogComp*)pDev->pPrivate)->Enable();

	// Anomaly 76 workaround
	NRF_LPCOMP->EVENTS_READY = 0;
	NRF_LPCOMP->TASKS_START = 1;
	while (NRF_LPCOMP->EVENTS_READY == 0);

	usDelay(150);

	NRF_LPCOMP->EVENTS_READY = 0;
	NRF_LPCOMP->EVENTS_CROSS = 0;
	NRF_LPCOMP->EVENTS_DOWN = 0;
	NRF_LPCOMP->EVENTS_UP = 0;

	NRF_LPCOMP->INTENSET = LPCOMP_INTENSET_UP_Msk | LPCOMP_INTENSET_DOWN_Msk;

	NVIC_ClearPendingIRQ(LPCOMP_IRQn);
	NVIC_SetPriority(LPCOMP_IRQn, pCfg->IntPrio);
	NVIC_EnableIRQ(LPCOMP_IRQn);

	return true;
}

bool AnalogCompnRF52LPComp::Init(const ANALOG_COMP_CFG &Cfg)
{
	vDevData.pPrivate = this;

	return AnalogCompInit(&vDevData, &Cfg);
}

bool AnalogCompnRF52LPComp::Enable()
{
	NRF_LPCOMP->ENABLE = 1;

	return true;
}

void AnalogCompnRF52LPComp::Disable()
{
	NRF_LPCOMP->ENABLE = 0;
}

bool AnalogCompnRF52LPComp::Start()
{
	NRF_LPCOMP->TASKS_START = 1;

	return true;
}

void AnalogCompnRF52LPComp::Stop()
{
	NRF_LPCOMP->TASKS_STOP = 1;
}


