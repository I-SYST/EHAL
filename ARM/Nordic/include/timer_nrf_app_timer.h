/**-------------------------------------------------------------------------
@file	timer_nrf_app_timer.h

@brief	Timer class implementation on Nordic nRF51 & nRF52 series
		Implementing using Nordic SDK app_timer

@author	Hoang Nguyen Hoan
@date	Feb. 5, 2018

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
#ifndef __TIMER_NRF_APP_TIMER_H__
#define __TIMER_NRF_APP_TIMER_H__

#include "timer.h"
#include "nrf_peripherals.h"
#include "app_timer.h"

/// App Timer implementation class
///
class TimerAppTimer : public Timer {
public:
	TimerAppTimer() : vAppTimerId(&vAppTimerData) {}
    virtual ~TimerAppTimer() {}

    virtual bool Init(const TIMER_CFG &Cfg);
    virtual bool Enable() { return true; }
    virtual void Disable() {}
    virtual void Reset() {}
    virtual uint32_t Frequency(uint32_t Freq) { vFreq = Freq; return vFreq;}
    virtual uint64_t TickCount() { return app_timer_cnt_get(); }
    int MaxTimerTrigger() { return 1; }

    /**
	 * @brief	Enable millisecond timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to enable. Index value starting at 0
	 * @param   msPeriod : Trigger period in msec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 * @param   pContext : Optional pointer to user private data to be passed
	 *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  real period in nsec based on clock calculation
	 */
    virtual uint32_t EnableTimerTrigger(int TrigNo, uint32_t msPeriod, TIMER_TRIG_TYPE Type,
                                        TIMER_TRIGCB Handler, void *pContext = NULL);

	/**
	 * @brief	Enable a specific nanosecond timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to enable. Index value starting at 0
	 * @param   nsPeriod : Trigger period in nsec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 * @param   pContext : Optional pointer to user private data to be passed
	 *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  real period in nsec based on clock calculation
	 */
    virtual uint64_t EnableTimerTrigger(int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                                        TIMER_TRIGCB Handler, void *pContext = NULL);
    virtual void DisableTimerTrigger(int TrigNo);

    int FindAvailTimerTrigger(void) { return 0; }

protected:
private:
    app_timer_t vAppTimerData;
    const app_timer_id_t vAppTimerId;// = &vAppTimerData;
    TIMER_TRIGCB vTrigHandler;
    void *vpTrigHandlerContext;

    static void TimerAppTimerHandler(void* pContext);
};



#endif // __TIMER_NRF_APP_TIMER_H__

