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
    TimerRTCnRF5x *pDevObj;
    uint64_t OvrCnt;
    TIMER_EVTCB EvtHandler;
    uint32_t CC[TIMER_RTC_MAX_TIMER_TRIGGER];
    TIMER_TRIG_TYPE TrigType[TIMER_RTC_MAX_TIMER_TRIGGER];
} NRF5X_RTC_DATA;

static NRF5X_RTC_DATA s_nRF5xRTCData[TIMER_RTC_MAX] = {
    { NRF_RTC0, },
    { NRF_RTC1, },
    { NRF_RTC2, }
};

static void RTCnRF5xInterrupt(int TimerNo)
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

    for (int i = 0; i < TIMER_RTC_MAX_TIMER_TRIGGER; i++)
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
	RTCnRF5xInterrupt(0);
}

void RTC1_IRQHandler()
{
	RTCnRF5xInterrupt(1);
}

void RTC2_IRQHandler()
{
	RTCnRF5xInterrupt(2);
}

}

TimerRTCnRF5x::TimerRTCnRF5x()
{

}

TimerRTCnRF5x::~TimerRTCnRF5x()
{

}

bool TimerRTCnRF5x::Init(const TIMER_CFG &Cfg)
{
    if (Cfg.DevNo < 0 || Cfg.DevNo >= TIMER_RTC_MAX)
        return false;

    vpReg = s_nRF5xRTCData[Cfg.DevNo].pReg;

    vpReg->TASKS_STOP = 1;
    vpReg->TASKS_CLEAR = 1;

    s_nRF5xRTCData[Cfg.DevNo].pDevObj = this;
    s_nRF5xRTCData[Cfg.DevNo].EvtHandler = Cfg.EvtHandler;

    vDevNo = Cfg.DevNo;

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


    uint32_t prescaler = 32768 / Cfg.Freq - 1;
    vpReg->PRESCALER = prescaler;

    vFreq = 32768 / (prescaler + 1);

    // Pre-calculate periods for faster timer counter to time conversion use later
    vnsPeriod = 1000000000 / vFreq;     // Period in nsec
    vusPeriod = 1000000 / vFreq;        // Period in usec

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

bool TimerRTCnRF5x::Enable()
{
    return true;
}

void TimerRTCnRF5x::Disable()
{

}

void TimerRTCnRF5x::Reset()
{
    vpReg->TASKS_CLEAR = 1;
}

bool TimerRTCnRF5x::Frequency(uint32_t Freq)
{
    vpReg->TASKS_STOP = 1;

    uint32_t prescaler = 32768 / Freq - 1;
    vpReg->PRESCALER = prescaler;

    vFreq = 32768 / (prescaler + 1);

    // Pre-calculate periods for faster timer counter to time conversion use later
    vnsPeriod = 1000000000 / vFreq;     // Period in nsec
    vusPeriod = 1000000 / vFreq;        // Period in usec

    vpReg->TASKS_START = 1;
}

uint64_t TimerRTCnRF5x::TickCount()
{
	return (uint64_t)vpReg->COUNTER + s_nRF5xRTCData[vDevNo].OvrCnt;
}

bool TimerRTCnRF5x::EnableTimerTrigger(int TimerNo, uint32_t Freq, TIMER_TRIG_TYPE Type)
{
    if (TimerNo < 0 || TimerNo >= TIMER_RTC_MAX_TIMER_TRIGGER)
        return false;

    s_nRF5xRTCData[vDevNo].TrigType[TimerNo] = Type;
    s_nRF5xRTCData[vDevNo].CC[TimerNo] = vFreq / Freq;
    vpReg->CC[TimerNo] = s_nRF5xRTCData[vDevNo].CC[TimerNo] + vpReg->COUNTER;
    vpReg->EVTENSET = 1 << (TimerNo + RTC_INTENSET_COMPARE0_Pos);

    if (s_nRF5xRTCData[vDevNo].EvtHandler)
    {
        vpReg->INTENSET = 1 << (TimerNo + RTC_INTENSET_COMPARE0_Pos);
    }

    return true;
}

void TimerRTCnRF5x::DisableTimerTrigger(int TimerNo)
{
    if (TimerNo < 0 || TimerNo >= TIMER_RTC_MAX_TIMER_TRIGGER)
        return;

    s_nRF5xRTCData[vDevNo].TrigType[TimerNo] = TIMER_TRIG_TYPE_SINGLE;
    s_nRF5xRTCData[vDevNo].CC[TimerNo] = 0;
    vpReg->CC[TimerNo] = 0;
    vpReg->EVTENCLR = 1 << (TimerNo + RTC_INTENSET_COMPARE0_Pos);
    vpReg->INTENCLR = 1 << (TimerNo + RTC_INTENSET_COMPARE0_Pos);
}
