/**-------------------------------------------------------------------------
@file	timer_hf_nrfx.cpp

@brief	Timer class implementation on Nordic nRFx series using high frequency timer (TIMERx)

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
#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "nrf.h"

#include "timer_nrfx.h"

#if defined(NRF91_SERIES) || defined(NRF5340_XXAA_APPLICATION)
#define NRF_TIMER0			NRF_TIMER0_S
#define NRF_TIMER1			NRF_TIMER1_S
#define NRF_TIMER2			NRF_TIMER2_S
#define NRF_CLOCK			NRF_CLOCK_S
#elif defined(NRF5340_XXAA_NETWORK)
#define NRF_TIMER0			NRF_TIMER0_NS
#define NRF_TIMER1			NRF_TIMER1_NS
#define NRF_TIMER2			NRF_TIMER2_NS
#define NRF_CLOCK			NRF_CLOCK_NS
#endif

static TimerHFnRFx *s_pnRFxTimer[TIMER_NRFX_HF_MAX] = {
	NULL,
};

void TimerHFnRFx::IRQHandler()
{
    uint32_t evt = 0;
    uint32_t count;
    uint32_t t = vpReg->CC[0];	// Preserve comparator

    // Read counter
    vpReg->TASKS_CAPTURE[0] = 1;
    count = vpReg->CC[0];

    // Restore comparator
    vpReg->CC[0] = t;

    if (count < vLastCount)
    {
        // Counter wrap arround
        evt |= TIMER_EVT_COUNTER_OVR;
        vRollover += vFreq;
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
            	vpReg->CC[i] = count + vCC[i];
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

void TIMER0_IRQHandler()
{
	if (s_pnRFxTimer[0])
		s_pnRFxTimer[0]->IRQHandler();
}

void TIMER1_IRQHandler()
{
	if (s_pnRFxTimer[1])
		s_pnRFxTimer[1]->IRQHandler();
}

void TIMER2_IRQHandler()
{
	if (s_pnRFxTimer[2])
		s_pnRFxTimer[2]->IRQHandler();
}

#if TIMER_NRFX_HF_MAX > 3
void TIMER3_IRQHandler()
{
	if (s_pnRFxTimer[3])
		s_pnRFxTimer[3]->IRQHandler();
}

void TIMER4_IRQHandler()
{
	if (s_pnRFxTimer[4])
		s_pnRFxTimer[4]->IRQHandler();
}
#endif

}   // extern "C"

TimerHFnRFx::TimerHFnRFx()
{

}

TimerHFnRFx::~TimerHFnRFx()
{

}

bool TimerHFnRFx::Init(const TIMER_CFG &Cfg)
{
    if (Cfg.DevNo < 0 || Cfg.DevNo >= TIMER_NRFX_HF_MAX)
    {
        return false;
    }

    memset(vTrigger, 0, sizeof(vTrigger));

    switch (Cfg.DevNo)
    {
    	case 0:
    		s_pnRFxTimer[0] = this;
    		vpReg = NRF_TIMER0;
    		vMaxNbTrigEvt = TIMER0_CC_NUM;
    		break;
    	case 1:
    		s_pnRFxTimer[1] = this;
    		vpReg = NRF_TIMER1;
    		vMaxNbTrigEvt = TIMER1_CC_NUM;
    		break;
    	case 2:
    		s_pnRFxTimer[2] = this;
    		vpReg = NRF_TIMER2;
    		vMaxNbTrigEvt = TIMER2_CC_NUM;
    		break;
#if TIMER_NRFX_HF_MAX > 3
    	case 3:
    		s_pnRFxTimer[3] = this;
    		vpReg = NRF_TIMER3;
    		vMaxNbTrigEvt = TIMER3_CC_NUM;
    		break;
    	case 4:
    		s_pnRFxTimer[4] = this;
    		vpReg = NRF_TIMER4;
    		vMaxNbTrigEvt = TIMER4_CC_NUM;
    		break;
#endif
    }

    vpReg->TASKS_STOP = 1;
    vpReg->TASKS_CLEAR = 1;
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;


    vEvtHandler = Cfg.EvtHandler;

    vDevNo = Cfg.DevNo;

    // Only support timer mode, 32bits counter
    vpReg->MODE = TIMER_MODE_MODE_Timer;
    vpReg->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

	switch (Cfg.DevNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(TIMER0_IRQn);
			NVIC_SetPriority(TIMER0_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(TIMER0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(TIMER1_IRQn);
			NVIC_SetPriority(TIMER1_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(TIMER1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(TIMER2_IRQn);
			NVIC_SetPriority(TIMER2_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(TIMER2_IRQn);
			break;
#if TIMER_NRFX_HF_MAX > 3
		case 3:
			NVIC_ClearPendingIRQ(TIMER3_IRQn);
			NVIC_SetPriority(TIMER3_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(TIMER3_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(TIMER4_IRQn);
			NVIC_SetPriority(TIMER4_IRQn, Cfg.IntPrio);
			NVIC_EnableIRQ(TIMER4_IRQn);
			break;
#endif
	}

    // Clock source not available.  Only 64MHz XTAL

    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    int timout = 1000000;

    do
    {
        if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) || NRF_CLOCK->EVENTS_HFCLKSTARTED)
            break;

    } while (timout-- > 0);

    if (timout <= 0)
        return false;

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    Frequency(Cfg.Freq);

    return true;
}

bool TimerHFnRFx::Enable()
{
    // Clock source not available.  Only 64MHz XTAL

    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    int timout = 1000000;

    do
    {
        if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) || NRF_CLOCK->EVENTS_HFCLKSTARTED)
            break;

    } while (timout-- > 0);

    if (timout <= 0)
        return false;

    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;


    vpReg->TASKS_START = 1;

	switch (vDevNo)
	{
		case 0:
			NVIC_ClearPendingIRQ(TIMER0_IRQn);
			NVIC_EnableIRQ(TIMER0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(TIMER1_IRQn);
			NVIC_EnableIRQ(TIMER1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(TIMER2_IRQn);
			NVIC_EnableIRQ(TIMER2_IRQn);
			break;
#if TIMER_NRFX_HF_MAX > 3
		case 3:
			NVIC_ClearPendingIRQ(TIMER3_IRQn);
			NVIC_EnableIRQ(TIMER3_IRQn);
			break;
		case 4:
			NVIC_ClearPendingIRQ(TIMER4_IRQn);
			NVIC_EnableIRQ(TIMER4_IRQn);
			break;
#endif
    }

    vpReg->TASKS_START = 1;

    return true;
}

void TimerHFnRFx::Disable()
{
    vpReg->TASKS_STOP = 1;
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;

    switch (vDevNo)
    {
        case 0:
            NVIC_ClearPendingIRQ(TIMER0_IRQn);
            NVIC_DisableIRQ(TIMER0_IRQn);
            break;
        case 1:
            NVIC_ClearPendingIRQ(TIMER1_IRQn);
            NVIC_DisableIRQ(TIMER1_IRQn);
            break;
        case 2:
            NVIC_ClearPendingIRQ(TIMER2_IRQn);
            NVIC_DisableIRQ(TIMER2_IRQn);
            break;
#if TIMER_NRFX_HF_MAX > 3
        case 3:
            NVIC_ClearPendingIRQ(TIMER3_IRQn);
            NVIC_DisableIRQ(TIMER3_IRQn);
            break;
        case 4:
            NVIC_ClearPendingIRQ(TIMER4_IRQn);
            NVIC_DisableIRQ(TIMER4_IRQn);
            break;
#endif
    }

    vpReg->INTENSET = 0;
}

void TimerHFnRFx::Reset()
{
    vpReg->TASKS_CLEAR = 1;
}

uint32_t TimerHFnRFx::Frequency(uint32_t Freq)
{
    vpReg->TASKS_STOP = 1;

    uint32_t prescaler = 0;

    if (Freq > 0)
    {
        uint32_t divisor = TIMER_NRFX_HF_BASE_FREQ / Freq;

        prescaler = 31 - __CLZ(divisor);

        if (prescaler > 9)
        {
            prescaler = 9;
        }
    }

    vpReg->PRESCALER = prescaler;

    vFreq = TIMER_NRFX_HF_BASE_FREQ / (1 << prescaler);

    // Pre-calculate periods for faster timer counter to time conversion use later
    vnsPeriod = 1000000000ULL / vFreq;     // Period in nsec

    vpReg->TASKS_START = 1;

    return vFreq;
}

uint64_t TimerHFnRFx::TickCount()
{
	uint32_t t = vpReg->CC[0];	// Save comparator

	// Read counter
	vpReg->TASKS_CAPTURE[0] = 1;
	uint32_t count = vpReg->CC[0];

	vpReg->CC[0] = t;	// Restore comparator

	if (count < vLastCount)
	{
		// Counter wrap arround
		vRollover += 0x100000000ULL;;
	}

	vLastCount = count;

	return (uint64_t)vLastCount + vRollover;
}

uint64_t TimerHFnRFx::EnableTimerTrigger(int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                                          TIMER_TRIGCB const Handler, void * const pContext)
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
    vpReg->TASKS_CAPTURE[TrigNo] = 1;

    uint32_t count = vpReg->CC[TrigNo];

	vpReg->INTENSET = TIMER_INTENSET_COMPARE0_Msk << TrigNo;

    vpReg->CC[TrigNo] = count + cc;

    if (count < vLastCount)
    {
        // Counter wrap around
        vRollover += 0x100000000ULL;;
    }

    vLastCount = count;

    vTrigger[TrigNo].nsPeriod = vnsPeriod * (uint64_t)cc;
    vTrigger[TrigNo].Handler = Handler;
    vTrigger[TrigNo].pContext = pContext;

    return vnsPeriod * (uint64_t)cc; // Return real period in nsec
}
/*
uint32_t TimerHFnRFx::EnableTimerTrigger(int TrigNo, uint32_t msPeriod, TIMER_TRIG_TYPE Type,
                                          TIMER_TRIGCB const Handler, void * const pContext)
{
	return (uint32_t)(EnableTimerTrigger(TrigNo, (uint64_t)msPeriod * 1000000ULL, Type, Handler, pContext) / 1000000ULL);
}
*/
void TimerHFnRFx::DisableTimerTrigger(int TrigNo)
{
    if (TrigNo < 0 || TrigNo >= vMaxNbTrigEvt)
        return;

    vCC[TrigNo] = 0;
    vpReg->CC[TrigNo] = 0;
    vpReg->INTENCLR = TIMER_INTENSET_COMPARE0_Msk << TrigNo;

    vTrigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
    vTrigger[TrigNo].Handler = NULL;
    vTrigger[TrigNo].pContext = NULL;
    vTrigger[TrigNo].nsPeriod = 0;

}

int TimerHFnRFx::FindAvailTimerTrigger(void)
{
	for (int i = 0; i < vMaxNbTrigEvt; i++)
	{
		if (vTrigger[i].nsPeriod == 0)
			return i;
	}

	return -1;
}


