/**-------------------------------------------------------------------------
@file	timer.cpp

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

#include "coredev/timer.h"

int Timer::EnableTimerTrigger(uint64_t nsPeriod, TIMER_TRIG_TYPE Type, TIMER_TRIGCB const Handler, void * const pContext)
{
	int idx = FindAvailTimerTrigger();
	if (idx < 0)
		return 0;

	EnableTimerTrigger(idx, nsPeriod, Type, Handler, pContext);

	return idx;
}

int Timer::EnableTimerTrigger(uint32_t msPeriod, TIMER_TRIG_TYPE Type, TIMER_TRIGCB const Handler, void * const pContext)
{
	int idx = FindAvailTimerTrigger();
	if (idx < 0)
		return 0;

	EnableTimerTrigger(idx, msPeriod, Type, Handler, pContext);

	return idx;
}
