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

#define TIMER_RTC_NRF5X_RC_FREQ         4000000
#define TIMER_RTC_NRF5X_XTAL_32K_FREQ   32768
#define TIMER_RTC_MAX                   3           // 3 RTC available

#define TIMER_RTC_MAX_TIMER_TRIGGER     4           // Counter compare

//
// nRF5x Real Time Counter using fixed 32768 Hz frequency
//
class TimerRTCnRF5x : public Timer {
public:
    TimerRTCnRF5x();
    virtual ~TimerRTCnRF5x();

	virtual bool Init(const TIMER_CFG &Cfg);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool Frequency(uint32_t Freq);
	virtual uint64_t TickCount();
    int MaxTriggerTimer() { return TIMER_RTC_MAX_TIMER_TRIGGER; }
    virtual bool EnableTimerTrigger(int TimerNo, uint32_t Freq, TIMER_TRIG_TYPE Type);
    virtual void DisableTimerTrigger(int TimerNo);

protected:
private:
    NRF_RTC_Type *vpReg;
};


#endif // __TIMER_NRF5x_H__
