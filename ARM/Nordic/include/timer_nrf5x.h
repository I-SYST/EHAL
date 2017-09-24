/*--------------------------------------------------------------------------
File   : timer_nrf5x.h

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

#ifndef __TIMER_NRF5x_H__
#define __TIMER_NRF5x_H__

#include <stdint.h>
#include "timer.h"

// *****
// Low frequency timer using Real Time Counter (RTC) 32768 Hz clock source
//
#define TIMER_NRF5X_RTC_MAX                 3           // 3 RTC available

#define TIMER_NRF5X_RTC_MAX_TRIGGER_EVT     4           // Max number of supported counter trigger event

class TimerLFnRF5x : public Timer {
public:
	TimerLFnRF5x();
    virtual ~TimerLFnRF5x();

	virtual bool Init(const TIMER_CFG &Cfg);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual uint32_t Frequency(uint32_t Freq);
	virtual uint32_t Frequency(void) { return vFreq; }
	virtual uint64_t TickCount();
    int MaxTriggerTimer() { return TIMER_NRF5X_RTC_MAX_TRIGGER_EVT; }
    virtual uint32_t EnableTimerTrigger(int TrigNo, uint32_t nsPeriod, TIMER_TRIG_TYPE Type);
    virtual void DisableTimerTrigger(int TrigNo);

protected:
private:
    NRF_RTC_Type *vpReg;
};

// *****
// High frequency timer using Timer 16MHz clock source
//
#define TIMER_NRF5X_HF_PCLK1M_FREQ      1000000
#define TIMER_NRF5X_HF_XTAL_FREQ        16000000
#define TIMER_NRF5X_HF_MAX              5           // 5 high frequency timer available

#define TIMER_NRF5X_HF_MAX_TRIGGER_EVT  6           // Max number of supported counter trigger event

class TimerHFnRF5x : public Timer {
public:
    TimerHFnRF5x();
    virtual ~TimerHFnRF5x();

    virtual bool Init(const TIMER_CFG &Cfg);
    virtual bool Enable();
    virtual void Disable();
    virtual void Reset();
    virtual uint32_t Frequency(uint32_t Freq);
    virtual uint64_t TickCount();
    int MaxTriggerTimer() { return TIMER_NRF5X_HF_MAX_TRIGGER_EVT; }
    virtual uint32_t EnableTimerTrigger(int TrigNo, uint32_t nsPeriod, TIMER_TRIG_TYPE Type);
    virtual void DisableTimerTrigger(int TrigNo);

    void IRQHandler();
protected:
private:

    NRF_TIMER_Type *vpReg;
    int vMaxNbTrigEvt;
    uint32_t vCC[TIMER_NRF5X_HF_MAX_TRIGGER_EVT];
    TIMER_TRIG_TYPE vTrigType[TIMER_NRF5X_HF_MAX_TRIGGER_EVT];
};

#endif // __TIMER_NRF5x_H__
