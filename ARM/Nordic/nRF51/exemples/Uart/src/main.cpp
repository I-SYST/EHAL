/*--------------------------------------------------------------------------
File   : main.cpp

Author : Hoang Nguyen Hoan          					Nov. 14, 2015

Desc   : This project shows how to use the EHAL UART implementation
		 on nRF5x


Copyright (c) 2015, I-SYST, all rights reserved

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
#include <stdio.h>


#include "coredev/iopincfg.h"
#include "uart_nrf5x.h"
#include "idelay.h"

#define TEST_INTERRUPT

//#define NEB
//#define NORDIC_DK

#ifdef NORDIC_DK
#define UART_TX_PIN			9//7
#define UART_RX_PIN			11//8
#define UART_RTS_PIN		8//11
#define UART_CTS_PIN		10//12
#elif defined(NEB)

#define UART_RX_PIN  	 	12//8 // 13
#define UART_TX_PIN  		13//7 // 12

#else
#define UART_TX_PIN			7
#define UART_RX_PIN			8
#define UART_RTS_PIN		11
#define UART_CTS_PIN		12
#endif

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// UART configuration data
const UARTCFG g_UartCfg = {
	0,
	{
		{0, UART_RX_PIN, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
		{0, UART_TX_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
#ifdef NEB
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
#else
		{0, UART_CTS_PIN, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
		{0, UART_RTS_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
#endif
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// DCD
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// DTE
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// DTR
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RI
	},
	//115200,	// Rate
	//230400,
	1000000,
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
#ifdef NEB
	UART_FLWCTRL_NONE,
#else
	UART_FLWCTRL_HW,
#endif

#ifdef TEST_INTERRUPT
	true,
#else
	false,		// Interrupt mode
#endif
	APP_IRQ_PRIORITY_LOW,
	nRFUartEvthandler,
};

#define C_CODE
#ifdef C_CODE
// For C
UARTDEV g_UartDev;
#else
// For C++
// UART object instance
UART g_Uart;
#endif

int g_DelayCnt = 0;

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			{
/*				g_DelayCnt++;

				if (g_DelayCnt > 20)
				{
					g_DelayCnt = 0;
					cnt = UARTRx(&g_UartDev, buff, 20);
					if (cnt > 0)
						UARTTx(&g_UartDev, buff, cnt);
				}*/
				//cnt = UARTRx(&g_UartDev, buff, 1);
				//if (cnt > 0)
				//	UARTTx(&g_UartDev, buff, cnt);
			}

			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			cnt = UARTRx(&g_UartDev, pBuffer, 6);
			if (cnt > 0)
				UARTTx(&g_UartDev, pBuffer, cnt);
			break;
	}

	return cnt;
}



//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.g
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	bool res;
char *data = "nRF UART Hello World\r\n";
	uint8_t buff[20];

/*	nrf_gpio_cfg_output(9);
	while (1)
	{
		nrf_gpio_pin_toggle(9);
		usDelay(1000);
	}*/
	//NRF_MPU->PERR0 &= ~(1 << 2);
	//uart_init();
	//simple_uart_config(UART_RTS_PIN, UART_TX_PIN, UART_CTS_PIN, UART_RX_PIN, false);

#ifdef C_CODE
	res = UARTInit(&g_UartDev, &g_UartCfg);
	UARTprintf(&g_UartDev, data);
#else
	res = g_Uart.Init(g_UartCfg);
	g_Uart.printf(data);dfg
#endif
//    nrf_drv_gpiote_in_config_t cts_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
//    nrf_drv_gpiote_in_init(p_comm_params->cts_pin_no, &cts_config, gpiote_uart_event_handler);

	//for (int i = 0; i < 20; i++)
	//	simple_uart_put(data[i]);
	//simple_uart_putstring((const uint8_t*)data);

	while(1) {
		uint8_t d;
		//usDelay(1000);
		//d = simple_uart_get();
		//simple_uart_put(d);
#ifndef TEST_INTERRUPT
		if (UARTRx(&g_UartDev, &d, 1) > 0)
		{
			UARTTx(&g_UartDev, &d, 1);
		//	UARTTx(&g_UartDev, (uint8_t*)data, 22);
			//usDelay(1000);
		}
#else
		int cnt = UARTRx(&g_UartDev, buff, 20);
		if (cnt > 0)
		{
			uint8_t *p = buff;
			while (cnt > 0)
			{
				int l = UARTTx(&g_UartDev, p, cnt);
				cnt -= l;
				p += l;
			}
		//	UARTTx(&g_UartDev, (uint8_t*)data, 22);
			//usDelay(1000);
		}

#endif
		//__WFI();
	}
	return 0;
}
