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
    uint32_t OvrCnt;
    TIMER_EVTCB EvtHandler;
} NRF5X_RTC_DATA;

static NRF5X_RTC_DATA s_nRF5xRTCData[TIMER_RTC_MAX] = {
    { NRF_RTC0, },
    { NRF_RTC1, },
    { NRF_RTC2, }
};

static void RTCnRF5xInterrupt(int TimerNo)
{
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

bool TimerRTCnRF5x::Init(TIMER_CFG &Cfg)
{
    if (Cfg.DevNo < 0 || Cfg.DevNo >= TIMER_RTC_MAX)
        return false;

    NRF_RTC_Type *reg = s_nRF5xRTCData[Cfg.DevNo].pReg;

    s_nRF5xRTCData[Cfg.DevNo].pDevObj = this;
    s_nRF5xRTCData[Cfg.DevNo].EvtHandler = Cfg.EvtHandler;

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
    reg->PRESCALER = prescaler;

    vFreq = 32768 / (prescaler + 1);
    vPeriod = 1000000000 / vFreq;   // Period in nsec

    return true;
}

uint32_t TimerRTCnRF5x::TickCount()
{
    return NRF_RTC0->COUNTER;
}

uint32_t TimerRTCnRF5x:: uSecond()
{
    return NRF_RTC0->COUNTER * vPeriod / 1000;
}

uint32_t TimerRTCnRF5x::nSecond()
{
    return NRF_RTC0->COUNTER * vPeriod;
}


