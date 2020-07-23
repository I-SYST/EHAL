/**-------------------------------------------------------------------------
@file	timer_nrfx.h

@brief	Timer class implementation on Nordic nRF5x & nRF91 series


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

#ifndef __TIMER_NRFX_H__
#define __TIMER_NRFX_H__

#include <stdint.h>

#include "nrf.h"
#include "nrf_peripherals.h"

#include "coredev/timer.h"

/// Low frequency timer using Real Time Counter (RTC) 32768 Hz clock source.
///
#define TIMER_NRFX_RTC_BASE_FREQ   			32768
#define TIMER_NRFX_RTC_MAX                 	RTC_COUNT	//!< Number RTC available
#define TIMER_NRFX_RTC_MAX_TRIGGER_EVT     	RTC1_CC_NUM	//!< Max number of supported counter trigger event

/// Low frequency timer implementation class.
///
class TimerLFnRFx : public Timer {
public:
	TimerLFnRFx();
    virtual ~TimerLFnRFx();

	virtual bool Init(const TIMER_CFG &Cfg);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual uint32_t Frequency(uint32_t Freq);
	virtual uint32_t Frequency(void) { return vFreq; }
	virtual uint64_t TickCount();
    int MaxTimerTrigger() { return vMaxNbTrigEvt; }
    /**
	 * @brief	Enable millisecond timer trigger event.
	 *
	 * Note: this must be implemented in each class. Otherwise it won't be
	 * visible to compiler from base class.  This is due to limitation of
	 * polymorphism of C++
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
                                        TIMER_TRIGCB const Handler = NULL, void * const pContext = NULL) {
    	return (uint32_t)(EnableTimerTrigger(TrigNo, (uint64_t)msPeriod * 1000000ULL, Type, Handler, pContext) / 1000000ULL);
    }

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
                                        TIMER_TRIGCB const Handler = NULL, void * const pContext = NULL);
    virtual void DisableTimerTrigger(int TrigNo);

    int FindAvailTimerTrigger(void);

    void IRQHandler();

protected:
private:
    NRF_RTC_Type *vpReg;
    int vMaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
    uint32_t vCC[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];
    TIMER_TRIGGER vTrigger[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];
};

/// High frequency timer using Timer 16MHz clock source.
///
#define TIMER_NRFX_HF_BASE_FREQ   			16000000
#define TIMER_NRFX_HF_MAX              		TIMER_COUNT		//!< Number high frequency timer available
#if TIMER_NRFX_HF_MAX < 4
#define TIMER_NRFX_HF_MAX_TRIGGER_EVT  		TIMER2_CC_NUM	//!< Max number of supported counter trigger event
#else
#define TIMER_NRFX_HF_MAX_TRIGGER_EVT  		TIMER3_CC_NUM	//!< Max number of supported counter trigger event
#endif

/// High frequency timer implementation class
///
class TimerHFnRFx : public Timer {
public:
    TimerHFnRFx();
    virtual ~TimerHFnRFx();

    virtual bool Init(const TIMER_CFG &Cfg);
    virtual bool Enable();
    virtual void Disable();
    virtual void Reset();
    virtual uint32_t Frequency(uint32_t Freq);
    virtual uint64_t TickCount();
    int MaxTimerTrigger() { return vMaxNbTrigEvt; }

    /**
	 * @brief	Enable millisecond timer trigger event.
	 *
	 * Note: this must be implemented in each class. Otherwise it won't be
	 * visible to compiler from base class.  This is due to limitation of
	 * polymorphism of C++
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
                                        TIMER_TRIGCB Handler = NULL, void *pContext = NULL) {
    	return (uint32_t)(EnableTimerTrigger(TrigNo, (uint64_t)msPeriod * 1000000ULL, Type, Handler, pContext) / 1000000ULL);
    }

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
                                        TIMER_TRIGCB Handler = NULL, void *pContext = NULL);
    virtual void DisableTimerTrigger(int TrigNo);

    int FindAvailTimerTrigger(void);

    void IRQHandler();

protected:
private:

    NRF_TIMER_Type *vpReg;
    int vMaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
    uint32_t vCC[TIMER_NRFX_HF_MAX_TRIGGER_EVT];
    TIMER_TRIGGER vTrigger[TIMER_NRFX_HF_MAX_TRIGGER_EVT];
};

// For backward compatibility, uncomment the lines bellow
//typedef TimerLFnRFx		TimerLFnRF5x;
//typedef TimerHFnRFx		TimerHFnRF5x;

#endif // __TIMER_NRFX_H__
