/**-------------------------------------------------------------------------
@file	pwm_nrf5x.cpp

@brief	Generic implementation of PWM (Pulse Width Modulation) on nRF5x

@author	Hoang Nguyen Hoan
@date	May 15, 2018

@license

Copyright (c) 2018, I-SYST, all rights reserved

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

#include "pwm.h"

#include "nrf.h"

#define PWM_NRF5_MAX_DEV		3
#define PWM_NRF5_MAX_CHAN		4

typedef struct {
	NRF_PWM_Type *pReg;					//!< PWM device register pointer
	PWM_DEV	*pDev;						//!< PWM device handle
	uint32_t Clk;						//!< Base clock frequency
	uint32_t TopCount;					//!< Top counter value for PWM freq
	PWM_POL Pol[PWM_NRF5_MAX_CHAN];		//!< Polarity 0 - active 0, 1 - active high
	uint16_t Seq0[PWM_NRF5_MAX_CHAN];
	uint16_t Seq1[PWM_NRF5_MAX_CHAN];
	volatile bool bStarted;
} PWM_NRF_DEV;

static PWM_NRF_DEV s_PwmnRFDev[PWM_NRF5_MAX_DEV] = {
	{NRF_PWM0, },
	{NRF_PWM1, },
	{NRF_PWM2, },
};

bool nRF52PWMWWaitForSTop(PWM_NRF_DEV *pDev, int Timeout)
{
	while (Timeout-- > 0)
	{
		if (pDev->pReg->EVENTS_STOPPED)
		{
			pDev->pReg->EVENTS_STOPPED = 0;
			pDev->bStarted = false;

			return true;
		}
	}

	return false;
}

bool PWMInit(PWM_DEV *pDev, const PWM_CFG *pCfg)
{
	PWM_NRF_DEV *dev = NULL;

	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= PWM_NRF5_MAX_DEV)
	{
		return false;
	}

	pDev->DevNo = pCfg->DevNo;
	pDev->Mode = pCfg->Mode;
	pDev->Mode = pCfg->Mode;
	dev = &s_PwmnRFDev[pCfg->DevNo];
	dev->pDev = pDev;
	pDev->pDevData = (void*)dev;

	// Clear all events
	dev->pReg->EVENTS_LOOPSDONE = 0;
	dev->pReg->EVENTS_PWMPERIODEND = 0;
	dev->pReg->EVENTS_SEQEND[0] = 0;
	dev->pReg->EVENTS_SEQEND[1] = 0;
	dev->pReg->EVENTS_SEQSTARTED[0] = 0;
	dev->pReg->EVENTS_SEQSTARTED[1] = 0;
	dev->pReg->EVENTS_STOPPED = 0;
	dev->bStarted = false;

	PWMSetFrequency(pDev, pCfg->Freq);

	memset(dev->Seq0, 0, sizeof(uint16_t) * PWM_NRF5_MAX_CHAN);
	memset(dev->Seq1, 0, sizeof(uint16_t) * PWM_NRF5_MAX_CHAN);

	dev->pReg->DECODER = PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos;

	dev->pReg->SEQ[0].PTR = (uint32_t)dev->Seq0;
	dev->pReg->SEQ[0].CNT = PWM_NRF5_MAX_CHAN;
	dev->pReg->SEQ[0].REFRESH = 0;
	dev->pReg->SEQ[0].ENDDELAY = 0;

	dev->pReg->SEQ[1].PTR = (uint32_t)dev->Seq1;
	dev->pReg->SEQ[1].CNT = PWM_NRF5_MAX_CHAN;
	dev->pReg->SEQ[1].REFRESH = 0;
	dev->pReg->SEQ[1].ENDDELAY = 0;

	if (pCfg->bIntEn)
	{
		IRQn_Type irqno;

		dev->pReg->INTEN = PWM_INTEN_LOOPSDONE_Msk | PWM_INTEN_PWMPERIODEND_Msk |
						   //PWM_INTEN_SEQEND1_Msk | PWM_INTEN_SEQEND0_Msk |
						   //PWM_INTEN_SEQSTARTED1_Msk | PWM_INTEN_SEQSTARTED0_Msk |
						   PWM_INTEN_STOPPED_Msk;

		switch (pCfg->DevNo)
		{
			case 0:
				irqno = PWM0_IRQn;
				break;
			case 1:
				irqno = PWM1_IRQn;
				break;
			case 2:
				irqno = PWM2_IRQn;
				break;
		}
		NVIC_ClearPendingIRQ(irqno);
		NVIC_SetPriority(irqno, pCfg->IntPrio);
		NVIC_EnableIRQ(irqno);
	}

	PWMEnable(pDev);

	return true;
}

bool PWMSetFrequency(PWM_DEV *pDev, uint32_t Freq)
{
	if (pDev == NULL)
		return false;

	PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

	if (dev == NULL)
		return false;

	uint32_t f = Freq * 0x7FFF; // Get max resolution

	if (f < 250000)
	{
		// 125 KHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_128;
		dev->Clk = 125000;
	}
	else if (f < 500000)
	{
		// 250 KHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_64;
		dev->Clk = 250000;
	}
	else if (f < 1000000)
	{
		// 500 KHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_32;
		dev->Clk = 500000;
	}
	else if (f < 2000000)
	{
		// 1 MHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;
		dev->Clk = 1000000;
	}
	else if (f < 4000000)
	{
		// 2 MHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_8;
		dev->Clk = 2000000;
	}
	else if (f < 8000000)
	{
		// 4 MHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_4;
		dev->Clk = 4000000;
	}
	else if (f < 16000000)
	{
		// 8 MHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_2;
		dev->Clk = 8000000;
	}
	else
	{
		// 16 MHz
		dev->pReg->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1;
		dev->Clk = 16000000;
	}

	// Calculate top counter based on selected clock frequency
	uint32_t ct = (dev->Clk + (Freq >> 1)) / Freq;

	pDev->Freq = (dev->Clk + (ct >> 1)) / ct;

	if (pDev->Mode == PWM_MODE_CENTER)
	{
		dev->pReg->MODE = PWM_MODE_UPDOWN_UpAndDown;
		ct >>= 1;
	}
	else
	{
		dev->pReg->MODE = PWM_MODE_UPDOWN_Up;
	}

	dev->TopCount = ct;
	dev->pReg->COUNTERTOP = ct;

	if (dev->bStarted)
	{
		dev->pReg->TASKS_SEQSTART[0] = 1;
	}

	return true;
}

bool PWMEnable(PWM_DEV *pDev)
{
	if (pDev == NULL)
		return false;

	PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

	if (dev == NULL)
		return false;

	dev->pReg->ENABLE = PWM_ENABLE_ENABLE_Enabled;

	return true;
}

void PWMDisable(PWM_DEV *pDev)
{
	if (pDev != NULL)
	{
		PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

		dev->pReg->ENABLE = PWM_ENABLE_ENABLE_Disabled;
	}
}

bool PWMOpenChannel(PWM_DEV *pDev, const PWM_CHAN_CFG *pChanCfg, int NbChan)
{
	if (pDev == NULL || pChanCfg == NULL)
		return false;

	PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

	for (int i = 0; i < NbChan; i++)
	{
		if (pChanCfg[i].Chan >= 0 && pChanCfg[i].Chan < PWM_NRF5_MAX_CHAN)
		{
			dev->pReg->PSEL.OUT[pChanCfg[i].Chan] = pChanCfg[i].Pin;
			dev->Pol[pChanCfg[i].Chan] = pChanCfg[i].Pol;
		}
	}

	return true;
}

void PWMCloseChannel(PWM_DEV *pDev, int Chan)
{
	if (pDev == NULL)
		return;

	PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

	if (Chan >= 0 && Chan < PWM_NRF5_MAX_CHAN)
	{
		dev->pReg->PSEL.OUT[Chan] |= (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
	}
}

bool PWMStart(PWM_DEV *pDev, uint32_t msDur)
{
	if (pDev == NULL)
		return false;

	PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

	if (dev == NULL)
		return false;

	dev->pReg->EVENTS_STOPPED = 0;

	if (msDur)
	{
		uint32_t c = msDur * pDev->Freq / 1000UL;

		dev->pReg->SHORTS = PWM_SHORTS_LOOPSDONE_STOP_Enabled << PWM_SHORTS_LOOPSDONE_STOP_Pos;
		dev->pReg->LOOP = c >> 1;
	}
	else
	{
		dev->pReg->SHORTS = 0;
		dev->pReg->LOOP = 0;
	}

	dev->pReg->TASKS_SEQSTART[0] = 1;
	dev->bStarted = true;

	return true;
}

void PWMStop(PWM_DEV *pDev)
{
	if (pDev != NULL)
	{
		PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

		dev->pReg->TASKS_STOP = 1;
		dev->bStarted = false;
		nRF52PWMWWaitForSTop(dev, 100000);
	}
}


bool PWMSetDutyCycle(PWM_DEV *pDev, int Chan, int DutyCycle)
{
	if (pDev == NULL)
		return false;

	if (Chan < 0 || Chan >= PWM_NRF5_MAX_CHAN)
		return false;

	PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

	uint32_t x = dev->TopCount * DutyCycle / 100;

	if (pDev->Mode == PWM_MODE_CENTER)
	{
		x >>= 1;
	}

	if (dev->Pol[Chan] == PWM_POL_HIGH)
	{
		x |= 0x8000;
	}


	dev->Seq0[Chan] = x;
	dev->Seq1[Chan] = x;

	if (dev->bStarted)
	{
		dev->pReg->TASKS_SEQSTART[0] = 1;
	}

	return true;
}

bool PWMPlay(PWM_DEV *pDev, int Chan, uint32_t Freq, uint32_t Dur)
{
	if (pDev == NULL)
		return false;

	if (Chan < 0 || Chan >= PWM_NRF5_MAX_CHAN)
		return false;

	PWM_NRF_DEV *dev = (PWM_NRF_DEV*)pDev->pDevData;

	if (dev == NULL)
		return false;

	uint32_t x = (pDev->Freq / Freq) & 0x7FFF;

	if (dev->Pol[Chan] == PWM_POL_LOW)
	{
		x |= 0x8000;
	}

	//dev->pReg->COUNTERTOP = x;
	dev->Seq0[Chan] = x;
	dev->pReg->SEQ[0].REFRESH = Dur *  pDev->Freq / 1000  ;
	dev->pReg->LOOP = Dur *  pDev->Freq / 1000  ;


	dev->pReg->SHORTS = PWM_SHORTS_LOOPSDONE_STOP_Msk;

	dev->pReg->TASKS_SEQSTART[0] = 1;

	return true;
}

void nRF52PWMIrqHandler(int PwmNo)
{
	PWM_NRF_DEV *dev = &s_PwmnRFDev[PwmNo];
	PWM_EVT evt = (PWM_EVT)-1;

	if (dev->pReg->EVENTS_LOOPSDONE == 1)
	{
		evt = PWM_EVT_STARTED;
		dev->pReg->EVENTS_LOOPSDONE = 0;
	}

	if (dev->pReg->EVENTS_PWMPERIODEND == 1)
	{
		evt = PWM_EVT_PERIOD;
		dev->pReg->EVENTS_PWMPERIODEND = 0;
	}
	if (dev->pReg->EVENTS_STOPPED == 1)
	{
		evt = PWM_EVT_STOPPED;
		dev->pReg->EVENTS_STOPPED = 0;
		dev->bStarted = false;
	}

	if (evt != -1 && dev->pDev->pEvtHandler)
	{
		dev->pDev->pEvtHandler(dev->pDev, evt);
	}
}

extern "C" {

void PWM0_IRQHandler()
{
	nRF52PWMIrqHandler(0);
}

void PWM1_IRQHandler()
{
	nRF52PWMIrqHandler(1);
}

void PWM2_IRQHandler()
{
	nRF52PWMIrqHandler(2);
}

} // extern "C"
