/**-------------------------------------------------------------------------
@example	UartSdkPrbsTx.c


@brief	UART PRBS transmit test

Demo code using Nordic nrfx_uarte to do PRBS transmit test using UART


@author	Hoang Nguyen Hoan
@date	Dec. 18, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#include <stdio.h>
#include <memory.h>

#include "app_uart.h"
#include "app_error.h"
#include "nrf.h"
#include "nrfx_uarte.h"

#include "prbs.h"

// This include contain i/o definition the board in use
#include "board.h"

#define SDK_NRFX	// Use nrfx_uarte, comment out this define to use app_uart_fifo

#ifndef SDK_NRFX
#define UART_TX_BUF_SIZE 512                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 512                         /**< UART RX buffer size. */

const app_uart_comm_params_t comm_params = {
		UART_RX_PIN,
		UART_TX_PIN,
		UART_RTS_PIN,
		UART_CTS_PIN,
		APP_UART_FLOW_CONTROL_ENABLED,//APP_UART_FLOW_CONTROL_DISABLED,
      false,
	  NRF_UARTE_BAUDRATE_1000000
};

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

#else

nrfx_uarte_t g_Uarte = NRFX_UARTE_INSTANCE(0);
nrfx_uarte_config_t g_UarteCfg = NRFX_UARTE_DEFAULT_CONFIG;


void uart_error_handle(nrfx_uarte_event_t const * p_event, void *ctx)
{
//	printf("Uarte Handler\r\n");
}
#endif


int main()
{
    uint32_t err_code;
	uint8_t d = 0xff;

#ifdef SDK_NRFX
    g_UarteCfg.pselrxd = UART_RX_PIN;
    g_UarteCfg.pseltxd = UART_TX_PIN;
    g_UarteCfg.hwfc = NRF_UARTE_HWFC_DISABLED;
    g_UarteCfg.baudrate = NRF_UARTE_BAUDRATE_1000000;
    g_UarteCfg.interrupt_priority = 1;

     err_code = nrfx_uarte_init(&g_Uarte, &g_UarteCfg, uart_error_handle);
    if (err_code != NRF_SUCCESS)
    {
    	//printf("Error %x\n\r", err_code);
    }


	while(1)
	{
		if (nrfx_uarte_tx(&g_Uarte, &d, 1) == NRF_SUCCESS)
		{
			// If success send next code
			d = Prbs8(d);
		}
	}
#else

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

    while (1)
    {
    	if (app_uart_put(d) == NRF_SUCCESS);
    	{
			// If success send next code
			d = Prbs8(d);
    	}
    }
#endif
	return 0;
}
