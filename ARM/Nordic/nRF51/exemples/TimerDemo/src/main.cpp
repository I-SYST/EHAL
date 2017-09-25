/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */

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

#include "iopincfg.h"
#include "iopinctrl.h"
#include "uart.h"
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
        	IOPinToggle(0, 23);
            break;

        default:
            //Do nothing.
            break;
    }
}
#else

void HFTimerHandler(Timer *pTimer, uint32_t Evt);
void TimerHandler(Timer *pTimer, uint32_t Evt);

const static TIMER_CFG s_HFTimerCfg = {
    0, TIMER_CLKSRC_DEFAULT, 0, 7, HFTimerHandler
};

TimerHFnRF5x g_HFTimer;

void TimerHandler(Timer *pTimer, uint32_t Evt);
/*
const static TIMER_CFG s_TimerCfg = {
    0, TIMER_CLKSRC_DEFAULT, 0, 7, TimerHandler
};
TimerLFnRF5x g_Timer;
*/
void HFTimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// Flip GPIO for oscilloscope measurement
    	IOPinToggle(0, 23);
//    	uint64_t c = pTimer->nSecond();
//    	g_Diff = c - g_TickCount;
//    	g_TickCount = c;
//        printf("Count = %u, Diff = %u\r\n", (uint32_t)g_TickCount, g_Diff);
    }
}
/*
void TimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// Flip GPIO for oscilloscope measurement
    	IOPinToggle(0, 23);
//    	uint64_t c = pTimer->nSecond();
//    	g_Diff = c - g_TickCount;
//    	g_TickCount = c;
//        printf("Count = %u, Diff = %u\r\n", (uint32_t)g_TickCount, g_Diff);
    }
}
*/
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
	IOPinConfig(0, 23, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

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
   // g_Timer.Init(s_TimerCfg);
    g_HFTimer.Init(s_HFTimerCfg);
//	uint64_t period = g_Timer.EnableTimerTrigger(0, 500000000ULL, TIMER_TRIG_TYPE_CONTINUOUS);
	uint64_t period = g_HFTimer.EnableTimerTrigger(0, 500000000ULL, TIMER_TRIG_TYPE_CONTINUOUS);

	//printf("Period = %u\r\n", (uint32_t)period);
#endif
    while (1)
    {
        __WFE();
//#ifndef NORDIC_SDK
//        printf("Count = %u, Diff = %u\r\n", (uint32_t)g_TickCount, g_Diff);
//#endif
    }
}
/** @} */
