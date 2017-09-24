/*--------------------------------------------------------------------------
File   : timer_hf_nrf5x.cpp

Author : Hoang Nguyen Hoan          				Sep. 7, 2017

Desc   : timer class implementation on Nordic nRF5x series
		using high frequency timer (TIMERx)

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include "nrf.h"

#include "timer_nrf5x.h"

#define	INTERRUPT_LATENCY		11

static TimerHFnRF5x *s_pnRF5xTimer[TIMER_NRF5X_HF_MAX] = {
	NULL,
};

void TimerHFnRF5x::IRQHandler()
{
    uint32_t evt = 0;
    uint32_t count;

    vpReg->TASKS_CAPTURE[0] = 1;
    count = vpReg->CC[0];

    for (int i = 0; i < vMaxNbTrigEvt; i++)
    {
        if (vpReg->EVENTS_COMPARE[i])
        {
            evt |= 1 << (i + 2);
            vpReg->EVENTS_COMPARE[i] = 0;
            if (vTrigType[i] == TIMER_TRIG_TYPE_CONTINUOUS)
            {
            	vpReg->CC[i] = count + vCC[i] - INTERRUPT_LATENCY;
            }
        }

    }

    if (count < vLastCount)
    {
        // Counter wrap arround
        evt |= TIMER_EVT_COUNTER_OVR;
        vRollover += vFreq;
    }

    vLastCount = count;

    if (vEvtHandler)
    {
        vEvtHandler(this, evt);
    }
}

extern "C" {

void TIMER0_IRQHandler()
{
	if (s_pnRF5xTimer[0])
		s_pnRF5xTimer[0]->IRQHandler();
}

void TIMER1_IRQHandler()
{
	if (s_pnRF5xTimer[1])
		s_pnRF5xTimer[1]->IRQHandler();
}

void TIMER2_IRQHandler()
{
	if (s_pnRF5xTimer[2])
		s_pnRF5xTimer[2]->IRQHandler();
}

#ifdef NRF52
void TIMER3_IRQHandler()
{
	if (s_pnRF5xTimer[3])
		s_pnRF5xTimer[3]->IRQHandler();
}

void TIMER4_IRQHandler()
{
	if (s_pnRF5xTimer[4])
		s_pnRF5xTimer[4]->IRQHandler();
}
#endif

}   // extern "C"

TimerHFnRF5x::TimerHFnRF5x()
{

}

TimerHFnRF5x::~TimerHFnRF5x()
{

}

bool TimerHFnRF5x::Init(const TIMER_CFG &Cfg)
{
    if (Cfg.DevNo < 0 || Cfg.DevNo >= TIMER_NRF5X_HF_MAX)
    {
        return false;
    }

	vMaxNbTrigEvt = TIMER_NRF5X_HF_MAX_TRIGGER_EVT;
    switch (Cfg.DevNo)
    {
    	case 0:
    		s_pnRF5xTimer[0] = this;
    		vpReg = NRF_TIMER0;
    		break;
    	case 1:
    		s_pnRF5xTimer[1] = this;
    		vpReg = NRF_TIMER1;
    		break;
    	case 2:
    		s_pnRF5xTimer[2] = this;
    		vpReg = NRF_TIMER2;
    		break;
#ifdef NRF52
    	case 3:
    		s_pnRF5xTimer[3] = this;
    		vpReg = NRF_TIMER3;
    		vMaxNbTrigEvt = TIMER_NRF5X_HF_MAX_TRIGGER_EVT + 2;
    		break;
    	case 4:
    		s_pnRF5xTimer[4] = this;
    		vpReg = NRF_TIMER4;
    		vMaxNbTrigEvt = TIMER_NRF5X_HF_MAX_TRIGGER_EVT + 2;
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

    if (Cfg.EvtHandler)
    {
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
#ifdef NRF52
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

bool TimerHFnRF5x::Enable()
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

    if (vEvtHandler)
    {
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
#ifdef NRF52
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

    }

    vpReg->TASKS_START = 1;

    return true;
}

void TimerHFnRF5x::Disable()
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
#ifdef NRF52
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

void TimerHFnRF5x::Reset()
{
    vpReg->TASKS_CLEAR = 1;
}

uint32_t TimerHFnRF5x::Frequency(uint32_t Freq)
{
    vpReg->TASKS_STOP = 1;

    uint32_t prescaler = 0;

    if (Freq > 0)
    {
        uint32_t divisor = TIMER_NRF5X_HF_BASE_FREQ / Freq;
        prescaler = 31 - __builtin_clzl(divisor);
        if (prescaler > 9)
        {
            prescaler = 9;
        }
    }

    vpReg->PRESCALER = prescaler;

    vFreq = TIMER_NRF5X_HF_BASE_FREQ / (1 << prescaler);

    // Pre-calculate periods for faster timer counter to time conversion use later
    // for precision this value is x10 (in 100 psec)
    vnsPeriod = 10000000000ULL / vFreq;     // Period in x10 nsec

    vpReg->TASKS_START = 1;

    return vFreq;
}

uint64_t TimerHFnRF5x::TickCount()
{
	if (vpReg->INTENSET == 0)
    {
		vpReg->TASKS_CAPTURE[vDevNo] = 1;

		uint32_t count = vpReg->CC[vDevNo];

		if (count < vLastCount)
	    {
	        // Counter wrap arround
	        vRollover += vFreq;
	    }

		vLastCount = count;
    }

	return (uint64_t)vLastCount + vRollover;
}

uint64_t TimerHFnRF5x::EnableTimerTrigger(int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type)
{
    if (TrigNo < 0 || TrigNo >= vMaxNbTrigEvt)
        return 0;

    // vnsPerios is x10 nsec (100 psec) => nsPeriod * 10ULL
    uint32_t cc = (nsPeriod * 10ULL + (vnsPeriod >> 1)) / vnsPeriod;

    if (cc <= 0)
    {
        return 0;
    }

    vTrigType[TrigNo] = Type;
    vCC[TrigNo] = cc;
    vpReg->TASKS_CAPTURE[TrigNo] = 1;

    uint32_t count = vpReg->CC[TrigNo];

    if (vEvtHandler)
    {
        vpReg->INTENSET = TIMER_INTENSET_COMPARE0_Msk << TrigNo;
    }

    vpReg->CC[TrigNo] = count + cc - INTERRUPT_LATENCY;

    if (count < vLastCount)
    {
        // Counter wrap arround
        vRollover += vFreq;
    }

    vLastCount = count;

    return vnsPeriod * (uint64_t)cc / 10ULL; // Return real period in nsec
}

void TimerHFnRF5x::DisableTimerTrigger(int TrigNo)
{
    if (TrigNo < 0 || TrigNo >= vMaxNbTrigEvt)
        return;

    vTrigType[TrigNo] = TIMER_TRIG_TYPE_SINGLE;
    vCC[TrigNo] = 0;
    vpReg->CC[TrigNo] = 0;
    vpReg->INTENCLR = TIMER_INTENSET_COMPARE0_Msk << TrigNo;
}


