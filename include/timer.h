/**-------------------------------------------------------------------------
@file	timer.h

@brief	Generic timer class

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

#ifndef __TIMER_H__
#define __TIMER_H__

#include "device.h"

/// Clock source used for the timer
typedef enum __Timer_Clock_Src {
	TIMER_CLKSRC_DEFAULT,	//!< Device default clock source
    TIMER_CLKSRC_INTERNAL,	//!< Internal RC oscillator
    TIMER_CLKSRC_LFXTAL,	//!< Low frequency crystal
    TIMER_CLKSRC_HFXTAL		//!< High frequency crystal
} TIMER_CLKSRC;

/// Timer interrupt enable type
typedef enum __Timer_Interrupt_Enable {
	TIMER_INTEN_NONE,		//!< No interrupt
	TIMER_INTEN_TICK,		//!< Enable tick count interrupt
	TIMER_INTEN_OVR			//!< Enable tick count overflow interrupt
} TIMER_INTEN;

/// Timer trigger type
typedef enum __Timer_Trigger_Type {
    TIMER_TRIG_TYPE_SINGLE,		//!< Single shot trigger
    TIMER_TRIG_TYPE_CONTINUOUS	//!< Continuous trigger
} TIMER_TRIG_TYPE;

#define TIMER_EVT_TICK                          (1<<0)   //!< Timer tick counter event
#define TIMER_EVT_COUNTER_OVR                   (1<<1)   //!< Timer overflow event
#define TIMER_EVT_TRIGGER0                		(1<<2)   //!< Periodic timer event start at this bit

#define TIMER_EVT_TRIGGER(n)              		(1<<(n+2))	//!< Trigger event id

class Timer;

/**
 * @brief	Timer event handler type.
 *
 * @param	Timer	: Pointer reference to Timer class generating the event
 * @param	Evt		: Event ID for which this callback is activated
 */
typedef void (*TIMER_EVTCB)(Timer *pTimer, uint32_t Evt);

/**
 * @brief	Timer trigger handler type
 *
 * @param	Timer	: Pointer reference to Timer class generating the event
 * @param	TrigNo	: Trigger ID for which this callback is activated
 */
typedef void (*TIMER_TRIGCB)(Timer *pTimer, int TrigNo);

#pragma pack(push, 4)

typedef struct __Timer_Trigger_Info {
	TIMER_TRIG_TYPE Type;	//!< Trigger type
	uint64_t nsPeriod;		//!< Trigger period in nanosecond
	TIMER_TRIGCB Handler;	//!< Trigger event callback
} TIMER_TRIGGER;

/// @brief	Timer configuration data.
///
/// NOTE : Interrupt priority should be as high as possible
/// Timing precision would be lost if other interrupt preempt
/// the timer interrupt.  Adjust IntPrio base on requirement
///
typedef struct __Timer_Config {
    int             DevNo;      //!< Device number.  Usually is the timer number indexed at 0
    TIMER_CLKSRC    ClkSrc;     //!< Clock source.  Not all timer allows user select clock source
    uint32_t        Freq;       //!< Frequency in Hz, 0 - to auto select max timer frequency
    int             IntPrio;    //!< Interrupt priority. recommended to use highest
    							//!< priority if precision timing is required
    TIMER_EVTCB     EvtHandler; //!< Interrupt handler
} TIMER_CFG;

#pragma pack(pop)

/// Timer base class
class Timer {
public:

    /**
     * @brief   Timer initialization.
     *
     * This is specific to each architecture.
     *
     * @param	Cfg	: Timer configuration data.
     *
     * @return
     * 			- true 	: Scuccess
     * 			- false : Otherwise
     */
    virtual bool Init(const TIMER_CFG &Cfg) = 0;

    /**
     * @brief   Turn on timer.
     *
     * This is used to re-enable timer after it was disabled for power
     * saving.  It normally does not go through full initialization sequence
     */
    virtual bool Enable() = 0;

    /**
     * @brief   Turn off timer.
     *
     * This is used to disable timer for power saving. Call Enable() to
     * re-enable timer instead of full initialization sequence
     */
    virtual void Disable() = 0;

    /**
     * @brief   Reset timer.
     */
    virtual void Reset() = 0;

    /**
     * @brief   Get the current tick count.
     *
     * This function read the tick count with compensated overflow roll over
     *
     * @return  Total tick count since last reset
     */
	virtual uint64_t TickCount() = 0;

	/**
	 * @brief	Set timer main frequency.
	 *
	 * This function allows dynamically changing the timer frequency.  Timer
	 * will be reset and restarted with new frequency
	 *
	 * @param 	Freq : Frequency in Hz
	 *
	 * @return  Real frequency
	 */
	virtual uint32_t Frequency(uint32_t Freq) = 0;

	/**
	 * @brief	Get maximum available timer trigger event for the timer.
	 *
	 * @return	count
	 */
	virtual int MaxTimerTrigger() = 0;

	/**
	 * @brief	Enable a specific nanosecond timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to enable. Index value starting at 0
	 * @param   nsPeriod : Trigger period in nsec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 *
	 * @return  real period in nsec based on clock calculation
	 */
	virtual uint64_t EnableTimerTrigger(int TrigNo, uint64_t nsPeriod,
										TIMER_TRIG_TYPE Type, TIMER_TRIGCB Handler = NULL) = 0;

	/**
	 * @brief	Enable nanosecond timer trigger event.
	 *
	 * @param   nsPeriod : Trigger period in nsec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 *
	 * @return  Timer trigger ID on success
	 * 			-1 : Failed
	 */
	virtual int EnableTimerTrigger(uint64_t nsPeriod, TIMER_TRIG_TYPE Type, TIMER_TRIGCB Handler = NULL);

	/**
	 * @brief	Enable millisecond timer trigger event.
	 *
	 * @param   msPeriod : Trigger period in msec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 *
	 * @return  Timer trigger ID on success
	 * 			-1 : Failed
	 */
	int EnableTimerTrigger(uint32_t msPeriod, TIMER_TRIG_TYPE Type, TIMER_TRIGCB Handler = NULL);

	/**
	 * @brief   Disable timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to disable. Index value starting at 0
	 */
    virtual void DisableTimerTrigger(int TrigNo) = 0;

    /**
     * @brief   Get current timer counter in millisecond.
     *
     * This function return the current timer in msec since last reset.
     *
     * @return  Counter in millisecond
     */
	virtual uint32_t mSecond() { return TickCount() * vnsPeriod / 1000000LL; }

	/**
	 * @brief   Convert tick count to millisecond.
	 *
	 * @param   Count : Timer tick count value
	 *
	 * @return  Converted count in millisecond
	 */
	virtual uint32_t mSecond(uint64_t Count) { return Count * vnsPeriod / 1000000LL; }

	/**
     * @brief   Get current timer counter in microsecond.
     *
     * This function return the current timer in usec since last reset.
     *
     * @return  Counter in microsecond
     */
	virtual uint32_t uSecond() { return TickCount() * vnsPeriod / 1000LL; }

	/**
	 * @brief   Convert tick count to microsecond.
	 *
	 * @param   Count : Timer tick count value
	 *
	 * @return  Converted count in microsecond
	 */
	virtual uint32_t uSecond(uint64_t Count) { return Count * vnsPeriod / 1000LL; }

	/**
     * @brief   Get current timer counter in nanosecond.
     *
     * This function return the current timer in nsec since last reset.
     *
     * @return  Counter in nanosecond
     */
	virtual uint32_t nSecond() { return TickCount() * vnsPeriod; }

	/**
     * @brief   Convert tick count to nanosecond
     *
     * @param   Count : Timer tick count value
     *
     * @return  Converted count in nanosecond
     */
	virtual uint32_t nSecond(uint64_t Count) { return Count * vnsPeriod; }

	/**
	 * @brief   Get current timer frequency
	 *
	 * @return  Timer frequency in Hz
	 */
	virtual uint32_t Frequency(void) { return vFreq; }

	/**
	 * @brief	Get first available timer trigger index.
	 *
	 * This function returns the first available timer trigger to be used to with
	 * EnableTimerTrigger
	 *
	 * @return	success : Timer trigger index
	 * 			fail : -1
	 */
	virtual int FindAvailTimerTrigger(void) = 0;

protected:

	TIMER_EVTCB vEvtHandler;//!< Pointer to user event handler callback

    int      vDevNo;		//!< Timer device number
	uint32_t vFreq;			//!< Frequency in Hz
	uint64_t vnsPeriod;		//!< Period in nsec
	uint64_t vRollover;     //!< Rollover counter adjustment
	uint32_t vLastCount;	//!< Last counter read value
private:
};

#endif // __TIMER_H__
