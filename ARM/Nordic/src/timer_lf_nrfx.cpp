/**-------------------------------------------------------------------------
@file	timer_lf_nrfx.cpp

@brief	timer class implementation on Nordic nRFx series using the RTC (real time counter)

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#include "nrf.h"

#include "timer_nrfx.h"

#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
#define NRF_CLOCK		NRF_CLOCK_NS
#define NRF_RTC0		NRF_RTC0_NS
#define NRF_RTC1		NRF_RTC1_NS
#else
#define NRF_CLOCK		NRF_CLOCK_S
#define NRF_RTC0		NRF_RTC0_S
#define NRF_RTC1		NRF_RTC1_S
#define NRF_RTC2		NRF_RTC1_S
#endif
#define CLOCK_LFCLKSRC_SRC_RC		CLOCK_LFCLKSRC_SRC_LFRC
#define CLOCK_LFCLKSRC_SRC_Xtal		CLOCK_LFCLKSRC_SRC_LFXO
#endif

static TimerLFnRFx *s_pnRFxRTC[TIMER_NRFX_RTC_MAX] = {
	NULL,
};

void TimerLFnRFx::IRQHandler()
{
    uint32_t evt = 0;
    uint32_t count = vpReg->COUNTER;

    if (vpReg->EVENTS_TICK)
    {
        evt |= TIMER_EVT_TICK;
        vpReg->EVENTS_TICK = 0;
    }

    if (vpReg->EVENTS_OVRFLW)
    {
        vRollover += 0x1000000ULL;
        evt |= TIMER_EVT_COUNTER_OVR;
        vpReg->EVENTS_OVRFLW = 0;
    }

    vLastCount = count;

    for (int i = 0; i < vMaxNbTrigEvt; i++)
    {
        if (vpReg->EVENTS_COMPARE[i])
        {
            evt |= 1 << (i + 2);
            vpReg->EVENTS_COMPARE[i] = 0;
            if (vTrigger[i].Type == TIMER_TRIG_TYPE_CONTINUOUS)
            {
            	vpReg->CC[i] = (count + vCC[i]) & 0xffffff;
            }
            if (vTrigger[i].Handler)
            {
            	vTrigger[i].Handler(this, i, vTrigger[i].pContext);
            }
        }
    }

    if (vEvtHandler)
    {
        vEvtHandler(this, evt);
    }
}

extern "C" {

__WEAK void RTC0_IRQHandler()
{
	if (s_pnRFxRTC[0])
		s_pnRFxRTC[0]->IRQHandler();
}

__WEAK void RTC1_IRQHandler()
{
	if (s_pnRFxRTC[1])
		s_pnRFxRTC[1]->IRQHandler();
}
#if TIMER_NRFX_RTC_MAX > 2
__WEAK void RTC2_IRQHandler()
{
	if (s_pnRFxRTC[2])
		s_pnRFxRTC[2]->IRQHandler();
}
#endif

}   // extern "C"

TimerLFnRFx::TimerLFnRFx()
{

}

TimerLFnRFx::~TimerLFnRFx()
{

}

bool TimerLFnRFx::Init(const TIMER_CFG &Cfg)
{
    if (Cfg.DevNo < 0 || Cfg.DevNo >= TIMER_NRFX_RTC_MAX)
        return false;

	switch (Cfg.DevNo)
    {
        case 0:
			s_pnRFxRTC[0] = this;
			vpReg = NRF_RTC0;
    		vMaxNbTrigEvt = RTC0_CC_NUM;
            break;
        case 1:
			s_pnRFxRTC[1] = this;
			vpReg = NRF_RTC1;
    		vMaxNbTrigEvt = RTC1_CC_NUM;
            break;
#if TIMER_NRFX_RTC_MAX > 2
        case 2:
			s_pnRFxRTC[2] = this;
			vpReg = NRF_RTC2;
    		vMaxNbTrigEvt = RTC2_CC_NUM;
            break;
#endif
    }

    vpReg->TASKS_STOP = 1;
    vpReg->TASKS_CLEAR = 1;
    NRF_CLOCK->TASKS_LFCLKSTOP = 1;

    switch (Cfg.ClkSrc)
    {
    	case TIMER_CLKSRC_DEFAULT:
        case TIMER_CLKSRC_LFRC:
            NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos;
            break;
        case TIMER_CLKSRC_LFXTAL:
            NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
            break;
#if defined(NRF52_SERIES) || (NRF51)
        case TIMER_CLKSRC_HFRC:
        case TIMER_CLKSRC_HFXTAL:
            NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Synth << CLOCK_LFCLKSRC_SRC_Pos;
            break;
#endif
    }
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    int timout = 1000000;

    do
    {
        if ((NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk) || NRF_CLOCK->EVENTS_LFCLKSTARTED)
            break;

    } while (timout-- > 0);

    if (timout <= 0)
        return false;

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

    vEvtHandler = Cfg.EvtHandler;

    vDevNo = Cfg.DevNo;

	switch (Cfg.DevNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(RTC0_IRQn);
			NVIC_SetPriority(RTC0_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(RTC0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(RTC1_IRQn);
			NVIC_SetPriority(RTC1_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(RTC1_IRQn);
			break;
#if TIMER_NRFX_RTC_MAX > 2
		case 2:
			NVIC_ClearPendingIRQ(RTC2_IRQn);
			NVIC_SetPriority(RTC2_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(RTC2_IRQn);
			break;
#endif
	}

	// Enable tick & overflow interrupt
	vpReg->INTENSET = RTC_INTENSET_OVRFLW_Msk;
    vpReg->EVTENSET = RTC_EVTEN_OVRFLW_Msk;

    Frequency(Cfg.Freq);

    return true;
}

bool TimerLFnRFx::Enable()
{
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    switch (vDevNo)
    {
        case 0:
            NVIC_ClearPendingIRQ(RTC0_IRQn);
            NVIC_EnableIRQ(RTC0_IRQn);
            break;
        case 1:
            NVIC_ClearPendingIRQ(RTC1_IRQn);
            NVIC_EnableIRQ(RTC1_IRQn);
            break;
#if TIMER_NRFX_RTC_MAX > 2
        case 2:
            NVIC_ClearPendingIRQ(RTC2_IRQn);
            NVIC_EnableIRQ(RTC2_IRQn);
            break;
#endif
    }

    vpReg->TASKS_START = 1;

    return true;
}

void TimerLFnRFx::Disable()
{
    vpReg->TASKS_STOP = 1;

    switch (vDevNo)
    {
        case 0:
            NVIC_ClearPendingIRQ(RTC0_IRQn);
            NVIC_DisableIRQ(RTC0_IRQn);
            break;
        case 1:
            NVIC_ClearPendingIRQ(RTC1_IRQn);
            NVIC_DisableIRQ(RTC1_IRQn);
            break;
#if TIMER_NRFX_RTC_MAX > 2
        case 2:
            NVIC_ClearPendingIRQ(RTC2_IRQn);
            NVIC_DisableIRQ(RTC2_IRQn);
            break;
#endif
    }

    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
}

void TimerLFnRFx::Reset()
{
    vpReg->TASKS_CLEAR = 1;
}

uint32_t TimerLFnRFx::Frequency(uint32_t Freq)
{
    vpReg->TASKS_STOP = 1;

    uint32_t prescaler = 1;

    if (Freq > 0)
    {
    	prescaler = TIMER_NRFX_RTC_BASE_FREQ / Freq;
    	if (prescaler > 0x1000)
    	{
    		// Cap at 12 bits
    		prescaler = 0x1000;
    	}
    }

    vpReg->PRESCALER = prescaler - 1;

    vFreq = TIMER_NRFX_RTC_BASE_FREQ / prescaler;

    // Pre-calculate periods for faster timer counter to time conversion use later
    vnsPeriod = 1000000000ULL / (uint64_t)vFreq;     // Period in nsec

    vpReg->TASKS_START = 1;

    return vFreq;
}

uint64_t TimerLFnRFx::TickCount()
{
	return (uint64_t)vpReg->COUNTER + vRollover;
}

uint64_t TimerLFnRFx::EnableTimerTrigger(int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                                          TIMER_TRIGCB Handler, void *pContext)
{
    if (TrigNo < 0 || TrigNo >= vMaxNbTrigEvt)
        return 0;

    uint32_t cc = (nsPeriod + (vnsPeriod >> 1)) / vnsPeriod;

    if (cc <= 0)
    {
        return 0;
    }

    vTrigger[TrigNo].Type = Type;
    vCC[TrigNo] = cc;
    vpReg->EVTENSET = RTC_EVTEN_COMPARE0_Msk << TrigNo;

	vpReg->INTENSET = RTC_INTENSET_COMPARE0_Msk << TrigNo;

    vpReg->CC[TrigNo] = vCC[TrigNo] + vpReg->COUNTER;

    vTrigger[TrigNo].nsPeriod = vnsPeriod * (uint64_t)cc;
    vTrigger[TrigNo].Handler = Handler;
    vTrigger[TrigNo].pContext = pContext;

    return vnsPeriod * (uint64_t)cc; // Return real period in nsec
}
/*
uint32_t TimerLFnRFx::EnableTimerTrigger(int TrigNo, uint32_t msPeriod, TIMER_TRIG_TYPE Type,
                                          TIMER_TRIGCB Handler, void *pContext)
{
	return (uint32_t)(EnableTimerTrigger(TrigNo, (uint64_t)msPeriod * 1000000ULL, Type, Handler, pContext) / 1000000ULL);
}
*/
void TimerLFnRFx::DisableTimerTrigger(int TrigNo)
{
    if (TrigNo < 0 || TrigNo >= vMaxNbTrigEvt)
        return;

    vCC[TrigNo] = 0;
    vpReg->CC[TrigNo] = 0;
    vpReg->EVTENCLR = RTC_EVTEN_COMPARE0_Msk << TrigNo;
    vpReg->INTENCLR = RTC_INTENCLR_COMPARE0_Msk << TrigNo;

    vTrigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
    vTrigger[TrigNo].Handler = NULL;
    vTrigger[TrigNo].pContext = NULL;
    vTrigger[TrigNo].nsPeriod = 0;
}

int TimerLFnRFx::FindAvailTimerTrigger(void)
{
	for (int i = 0; i < vMaxNbTrigEvt; i++)
	{
		if (vTrigger[i].nsPeriod == 0)
			return i;
	}

	return -1;
}

