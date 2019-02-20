/**-------------------------------------------------------------------------
@example	TimerDemo.cpp


@brief	Timer class usage demo.


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

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"

#ifdef NORDIC_SDK
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
#else
#include "timer_nrf5x.h"
#endif

#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "board.h"
#include "stddev.h"

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_TxBuff[FIFOSIZE];

static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
static const UARTCFG s_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPINCFG),
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	15, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	NULL, //nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_TxBuff,
};

UART g_Uart;

uint64_t g_TickCount = 0;
uint32_t g_Diff = 0;


#ifdef NORDIC_SDK
const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    static uint32_t i;
    uint32_t led_to_invert = ((i++) % LEDS_NUMBER);

    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
        	// Flip GPIO for oscilloscope measurement
        	IOPinToggle(0, 22);
            break;

        default:
            //Do nothing.
            break;
    }
}
#else

void TimerHandler(Timer *pTimer, uint32_t Evt);

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 7,
	.EvtHandler = TimerHandler
};

#if 0
// Using RTC
TimerLFnRF5x g_Timer;
#else
// Using Timer
TimerHFnRF5x g_Timer;
#endif

void TimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// Flip GPIO for oscilloscope measurement
    	IOPinToggle(0, 22);
#if 0
    	uint64_t c = pTimer->nSecond();
    	g_Diff = c - g_TickCount;
    	g_TickCount = c;
#endif
    }
}

#endif

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    g_Uart.Init(s_UartCfg);

    UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	IOPinConfig(0, 22, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

#ifdef NORDIC_SDK
    uint32_t time_ms = 500; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

//    bsp_board_leds_init();

	//Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, 500UL);

    nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_LED);
#else
    g_Timer.Init(s_TimerCfg);
	uint64_t period = g_Timer.EnableTimerTrigger(0, 500000000ULL, TIMER_TRIG_TYPE_CONTINUOUS);

	//printf("Period = %u\r\n", (uint32_t)period);
#endif
    while (1)
    {
        __WFE();
//        printf("Count = %u, Diff = %u\r\n", (uint32_t)g_TickCount, g_Diff);
    }
}
/** @} */
