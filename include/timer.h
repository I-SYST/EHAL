/*--------------------------------------------------------------------------
File   : timer.h

Author : Hoang Nguyen Hoan          				Sep. 7, 2017

Desc   : Generic timer class

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

#ifndef __TIMER_H__
#define __TIMER_H__

#include "device.h"

typedef enum __Timer_Clock_Src {
    TIMER_CLKSRC_INTERNAL,
    TIMER_CLKSRC_LFXTAL,
    TIMER_CLKSRC_HFXTAL
} TIMER_CLKSRC;

typedef enum __Timer_Interrupt_Enable {
	TIMER_INTEN_NONE,
	TIMER_INTEN_TICK,		// Enable tick count interrupt
	TIMER_INTEN_OVR			// Enable tick count overflow interrupt
} TIMER_INTEN;

typedef enum __Timer_Trigger_Type {
    TIMER_TRIG_TYPE_SINGLE,
    TIMER_TRIG_TYPE_CONTINUOUS
} TIMER_TRIG_TYPE;

#define TIMER_EVT_TICK                          (1<<0)   // Timer tick counter event
#define TIMER_EVT_COUNTER_OVR                   (1<<1)   // Timer overflow event
#define TIMER_EVT_TIMER_TRIGGER0                (1<<2)   // Periodic timer event start at this bit

#define TIMER_EVT_TIMER_TRIGGER(n)              (1<<(n+2))

class Timer;

typedef void (*TIMER_EVTCB)(Timer *pTimer, uint32_t Evt);

#pragma pack(push, 4)

//
// NOTE : Interrupt priority should be as high as possible
// Timing precision would be lost if other interrupt preempt
// the timer interrupt.  Adjust IntPrio base on requirement
//
typedef struct __Timer_Config {
    int             DevNo;      // Device number
    TIMER_CLKSRC    ClkSrc;     // Clock source
    uint32_t        Freq;       // Frequency in Hz
    int             IntPrio;    // Interrupt priority
    TIMER_EVTCB     EvtHandler; // Interrupt handler
} TIMER_CFG;

#pragma pack(pop)

class Timer {
public:

    /**
     * @brief   Timer initialization
     *      This is specific to each architecture.
     */
    virtual bool Init(const TIMER_CFG &Cfg) = 0;

    /**
     * @brief   Enable timer
     *      This is used to re-enable timer after it was disabled for power
     * saving.  It normally does not go through full initialization sequence
     */
    virtual bool Enable() = 0;

    /**
     * @brief   Disable timer
     *      This is used to disable timer for power saving. Call Enable() to
     * re-enable timer instead of full initialization sequence
     */
    virtual void Disable() = 0;

    /**
     * @brief   Reset timer
     */
    virtual void Reset() = 0;

    /**
     * @brief   Get
     */
	virtual uint64_t TickCount() = 0;

	/**
	 * @brief	Set timer main frequency
	 *
	 * @param 	Freq : Frequency in Hz
	 *
	 * @return
	 */
	virtual bool Frequency(uint32_t Freq) = 0;

	/**
	 * @brief	Get maximum available timer trigger event for the timer
	 *
	 * @return	count
	 */
	virtual int MaxTriggerTimer() = 0;

	/**
	 * @brief	Enable timer trigger event
	 *
	 * @param TimerNo
	 * @param nsPeriod
	 * @param Type
	 * @return
	 */
	virtual bool EnableTimerTrigger(int TimerNo, uint32_t nsPeriod, TIMER_TRIG_TYPE Type) = 0;
    virtual void DisableTimerTrigger(int TimerNo) = 0;

	virtual uint32_t uSecond() { return TickCount() * vusPeriod; }
	virtual uint32_t uSecond(uint64_t Count) { return Count * vusPeriod; }
	virtual uint32_t nSecond() { return TickCount() * vnsPeriod; }
	virtual uint32_t nSecond(uint64_t Count) { return Count * vnsPeriod; }
	virtual uint32_t Frequency() { return vFreq; }

protected:

    int      vDevNo;
	uint32_t vFreq;			// Frequency in Hz
	uint32_t vnsPeriod;		// Period in nsec
    uint32_t vusPeriod;     // Period in usec
	uint32_t vOvrCnt;       // Overflow counter

private:
};

#endif // __TIMER_H__
