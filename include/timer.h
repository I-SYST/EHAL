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

typedef enum __Timer_Event {
    TIMER_EVT_TICK,
    TIMER_EVT_COUNTER_OVR,      // Timer overflow
    TIMER_EVT_COMPARATOR
} TIMER_EVT;

class Timer;

typedef void (*TIMER_EVTCB)(Timer *pTimer, TIMER_EVT Evt, uint32_t CompNo);

#pragma pack(push, 4)

typedef struct __Timer_Config {
    int             DevNo;      // Device number
    TIMER_CLKSRC    ClkSrc;     // Clock source
    uint32_t        Freq;       // Frequency in Hz
    TIMER_EVTCB     EvtHandler;
} TIMER_CFG;

#pragma pack(pop)

class Timer : public Device {
public:

	virtual uint32_t TickCount() = 0;
	virtual uint32_t uSecond() = 0;
	virtual uint32_t nSecond() = 0;

protected:

	uint32_t vFreq;			// Frequency in Hz
	uint32_t vPeriod;		// Period in nsec
	uint32_t vOvrCnt;       // Overflow counter

private:
};

#endif // __TIMER_H__
