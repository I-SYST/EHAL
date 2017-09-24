/*--------------------------------------------------------------------------
File   : timer_nrf5x.cpp

Author : Hoang Nguyen Hoan          				Sep. 7, 2017

Desc   : timer class implementation on Nordic nRF5x series

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

typedef struct {
    NRF_RTC_Type *pReg;
    TimerLFnRF5x *pDevObj;
    uint64_t OvrCnt;
    TIMER_EVTCB EvtHandler;
    uint32_t CC[TIMER_NRF5X_RTC_MAX_TRIGGER_EVT];
    TIMER_TRIG_TYPE TrigType[TIMER_NRF5X_RTC_MAX_TRIGGER_EVT];
} NRF5X_RTC_DATA;

static NRF5X_RTC_DATA s_nRF5xRTCData[TIMER_NRF5X_RTC_MAX] = {
    { NRF_RTC0, },
    { NRF_RTC1, },
    { NRF_RTC2, }
};

static void nRF5xRTCIRQ(int TimerNo)
{
    NRF_RTC_Type *reg = s_nRF5xRTCData[TimerNo].pReg;
    uint32_t evt = 0;
    uint32_t dummy;
    uint32_t count = reg->COUNTER;

    if (reg->EVENTS_TICK)
    {
        evt |= TIMER_EVT_TICK;
        reg->EVENTS_TICK = 0;
    }

    if (reg->EVENTS_OVRFLW)
    {
        s_nRF5xRTCData[TimerNo].OvrCnt += ((Timer*)s_nRF5xRTCData[TimerNo].pDevObj)->Frequency();
        evt |= TIMER_EVT_COUNTER_OVR;
        reg->EVENTS_OVRFLW = 0;
    }

    for (int i = 0; i < TIMER_NRF5X_RTC_MAX_TRIGGER_EVT; i++)
    {
        if (reg->EVENTS_COMPARE[i])
        {
            evt |= 1 << (i + 2);
            reg->EVENTS_COMPARE[i] = 0;
            if (s_nRF5xRTCData[TimerNo].TrigType[i] == TIMER_TRIG_TYPE_CONTINUOUS)
            {
                reg->CC[i] = count + s_nRF5xRTCData[TimerNo].CC[i];
            }
        }
    }

    if (s_nRF5xRTCData[TimerNo].EvtHandler)
    {
        s_nRF5xRTCData[TimerNo].EvtHandler(s_nRF5xRTCData[TimerNo].pDevObj, evt);
    }
}

extern "C" {

void RTC0_IRQHandler()
{
	nRF5xRTCIRQ(0);
}

void RTC1_IRQHandler()
{
	nRF5xRTCIRQ(1);
}

void RTC2_IRQHandler()
{
	nRF5xRTCIRQ(2);
}

}   // extern "C"

TimerLFnRF5x::TimerLFnRF5x()
{

}

TimerLFnRF5x::~TimerLFnRF5x()
{

}

bool TimerLFnRF5x::Init(const TIMER_CFG &Cfg)
{
    if (Cfg.DevNo < 0 || Cfg.DevNo >= TIMER_NRF5X_RTC_MAX)
        return false;

    vpReg = s_nRF5xRTCData[Cfg.DevNo].pReg;

    vpReg->TASKS_STOP = 1;
    vpReg->TASKS_CLEAR = 1;
    NRF_CLOCK->TASKS_LFCLKSTOP = 1;

    switch (Cfg.ClkSrc)
    {
        case TIMER_CLKSRC_INTERNAL:
            NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos;
            break;
        case TIMER_CLKSRC_LFXTAL:
            NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
            break;
        case TIMER_CLKSRC_HFXTAL:
            NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Synth << CLOCK_LFCLKSRC_SRC_Pos;
            break;
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

    s_nRF5xRTCData[Cfg.DevNo].pDevObj = this;
    s_nRF5xRTCData[Cfg.DevNo].EvtHandler = Cfg.EvtHandler;

    vDevNo = Cfg.DevNo;

    uint32_t prescaler = 32768;

    if (Cfg.Freq > 0)
        prescaler /= Cfg.Freq;

    vpReg->PRESCALER = prescaler - 1;

    vFreq = 32768 / prescaler;

    // Pre-calculate periods for faster timer counter to time conversion use later
    vnsPeriod = 1000000000 / vFreq;     // Period in nsec

    if (Cfg.EvtHandler)
    {
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
            case 2:
                NVIC_ClearPendingIRQ(RTC2_IRQn);
                NVIC_SetPriority(RTC2_IRQn, Cfg.IntPrio);
                NVIC_EnableIRQ(RTC2_IRQn);
                break;
        }

        // Enable tick & overflow interrupt
        vpReg->INTENSET = RTC_INTENSET_OVRFLW_Msk;
    }

    vpReg->EVTENSET = RTC_EVTEN_OVRFLW_Msk;
    vpReg->TASKS_START = 1;

    return true;
}

bool TimerLFnRF5x::Enable()
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
        case 2:
            NVIC_ClearPendingIRQ(RTC2_IRQn);
            NVIC_EnableIRQ(RTC2_IRQn);
            break;
    }

    vpReg->TASKS_START = 1;

    return true;
}

void TimerLFnRF5x::Disable()
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
            NVIC_EnableIRQ(RTC1_IRQn);
            break;
        case 2:
            NVIC_ClearPendingIRQ(RTC2_IRQn);
            NVIC_EnableIRQ(RTC2_IRQn);
            break;
    }

    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
}

void TimerLFnRF5x::Reset()
{
    vpReg->TASKS_CLEAR = 1;
}

uint32_t TimerLFnRF5x::Frequency(uint32_t Freq)
{
    vpReg->TASKS_STOP = 1;

    uint32_t prescaler = 32768;

    if (Freq > 0)
        prescaler /= Freq;

    vpReg->PRESCALER = prescaler - 1;

    vFreq = 32768 / prescaler;

    // Pre-calculate periods for faster timer counter to time conversion use later
    vnsPeriod = 1000000000 / vFreq;     // Period in nsec

    vpReg->TASKS_START = 1;

    return vFreq;
}

uint64_t TimerLFnRF5x::TickCount()
{
	return (uint64_t)vpReg->COUNTER + s_nRF5xRTCData[vDevNo].OvrCnt;
}

uint32_t TimerLFnRF5x::EnableTimerTrigger(int TrigNo, uint32_t nsPeriod, TIMER_TRIG_TYPE Type)
{
    if (TrigNo < 0 || TrigNo >= TIMER_NRF5X_RTC_MAX_TRIGGER_EVT)
        return 0;

    uint32_t cc = nsPeriod / vnsPeriod;

    if (cc <= 0)
    {
        return 0;
    }

    s_nRF5xRTCData[vDevNo].TrigType[TrigNo] = Type;
    s_nRF5xRTCData[vDevNo].CC[TrigNo] = cc;
    vpReg->EVTENSET = 1 << (TrigNo + RTC_INTENSET_COMPARE0_Pos);

    if (s_nRF5xRTCData[vDevNo].EvtHandler)
    {
        vpReg->INTENSET = 1 << (TrigNo + RTC_INTENSET_COMPARE0_Pos);
    }

    vpReg->CC[TrigNo] = s_nRF5xRTCData[vDevNo].CC[TrigNo] + vpReg->COUNTER;

    return vnsPeriod * cc; // Return real period in nsec
}

void TimerLFnRF5x::DisableTimerTrigger(int TrigNo)
{
    if (TrigNo < 0 || TrigNo >= TIMER_NRF5X_RTC_MAX_TRIGGER_EVT)
        return;

    s_nRF5xRTCData[vDevNo].TrigType[TrigNo] = TIMER_TRIG_TYPE_SINGLE;
    s_nRF5xRTCData[vDevNo].CC[TrigNo] = 0;
    vpReg->CC[TrigNo] = 0;
    vpReg->EVTENCLR = 1 << (TrigNo + RTC_INTENSET_COMPARE0_Pos);
    vpReg->INTENCLR = 1 << (TrigNo + RTC_INTENSET_COMPARE0_Pos);
}
