/**-------------------------------------------------------------------------
@file	nrf_log_backend_uart.c

@brief	Implementation of UART backend for NRF_LOG

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2018

@license

MIT License

Copyright (c) 2018-2020 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include "sdk_common.h"
#include "nrf_log_backend_interface.h"
#include "nrf_log_backend_serial.h"
#include "nrf_log_internal.h"
#include "app_error.h"

#include "coredev/uart.h"
#include "nrf_log_backend_uart.h"

UARTDEV *g_pUart = NULL;

#define BACKEND_BUFFER_SIZE 255//NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE

static uint8_t g_UartLogBuff[BACKEND_BUFFER_SIZE];

void nrf_log_uart_init(UARTDEV * const pUart)
{
	if (pUart)
	{
		g_pUart = pUart;
	}
}

void nrf_log_backend_uart_init()
{
	if (g_pUart == NULL)
	{
		g_pUart = UARTGetInstance(0);
	}
}

static void serial_tx(void const * p_context, char const * p_buffer, size_t len)
{
	if (g_pUart)
    {
		UARTTx(g_pUart, (uint8_t *)p_buffer, len);
    }
}

static void nrf_log_backend_uart_put(nrf_log_backend_t const * p_backend,
                                     nrf_log_entry_t * p_msg)
{
    nrf_log_backend_serial_put(p_backend, p_msg, g_UartLogBuff, BACKEND_BUFFER_SIZE, serial_tx);
}

static void nrf_log_backend_uart_flush(nrf_log_backend_t const * p_backend)
{
}

static void nrf_log_backend_uart_panic_set(nrf_log_backend_t const * p_backend)
{
}

const nrf_log_backend_api_t nrf_log_backend_uart_api = {
        .put       = nrf_log_backend_uart_put,
        .flush     = nrf_log_backend_uart_flush,
        .panic_set = nrf_log_backend_uart_panic_set,
};
