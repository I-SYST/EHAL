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
#ifdef NRF51
#include "app_timer_appsh.h"
#else
#include "app_timer.h"
#endif
#include "app_error.h"

#include "timer_nrf_app_timer.h"

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */

void TimerAppTimer::TimerAppTimerHandler(void* pContext)
{
	TimerAppTimer *timer = (TimerAppTimer*)pContext;

	timer->vTrigHandler(timer, 0, timer->vpTrigHandlerContext);
}

bool TimerAppTimer::Init(const TIMER_CFG &Cfg)
{
#ifdef NRF51
	APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, 10, true);
#else
	app_timer_init();
#endif
	return true;
}

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
uint32_t TimerAppTimer::EnableTimerTrigger(int TrigNo, uint32_t msPeriod, TIMER_TRIG_TYPE Type,
										  TIMER_TRIGCB Handler, void *pContext)
{
    // Create timers.
    uint32_t err_code = app_timer_create(&vAppTimerId,
    										Type == TIMER_TRIG_TYPE_CONTINUOUS ? APP_TIMER_MODE_REPEATED:APP_TIMER_MODE_SINGLE_SHOT,
    										TimerAppTimerHandler);
    APP_ERROR_CHECK(err_code);

    vTrigHandler = Handler;
    vpTrigHandlerContext = pContext;

#ifdef NRF51
    err_code = app_timer_start(vAppTimerId, APP_TIMER_TICKS(msPeriod, APP_TIMER_PRESCALER), this);
#else
    err_code = app_timer_start(vAppTimerId, APP_TIMER_TICKS(msPeriod), this);
#endif

    APP_ERROR_CHECK(err_code);

	return msPeriod;
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
uint64_t TimerAppTimer::EnableTimerTrigger(int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
										  TIMER_TRIGCB Handler, void *pContext)
{
	uint32_t period = nsPeriod / 1000000ULL;

	if (period == 0)
		return 0;

	return (uint64_t)EnableTimerTrigger(TrigNo, period, Type, Handler, pContext) * 1000000ULL;
}

void TimerAppTimer::DisableTimerTrigger(int TrigNo)
{
	app_timer_stop(vAppTimerId);
}
