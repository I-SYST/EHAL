/*--------------------------------------------------------------------------
File   : main.cpp

Author : Hoang Nguyen Hoan          Aug. 31, 2016

Desc   : UART PRBS test
		 Demo code using EHAL library to do PRBS transmit test using UART

Copyright (c) 2016, I-SYST inc., all rights reserved

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

#include "iopincfg.h"
#include "uart.h"
#include "prbs.h"

#define UART_RXD_PORT				0
#define UART_RXD_PIN				18
#define UART_RXD_PINOP				1
#define UART_TXD_PORT				0
#define UART_TXD_PIN				19
#define UART_TXD_PINOP				1
#define UART_CTS_PORT				0
#define UART_CTS_PIN				7
#define UART_CTS_PINOP				1
#define UART_RTS_PORT				0
#define UART_RTS_PIN				17
#define UART_RTS_PINOP				1

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define CFIFOMEMSIZE		(256)

uint8_t g_RxBuff[CFIFOMEMSIZE];
uint8_t g_TxBuff[CFIFOMEMSIZE];

static const IOPINCFG s_UartPins[] = {
	{UART_RXD_PORT, UART_RXD_PIN, UART_RXD_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TXD_PORT, UART_TXD_PIN, UART_TXD_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
#ifdef FLOWCTRL
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_FOLLOW, IOPINTYPE_NORMAL},	// RTS
#endif
};

// UART configuration data
const UARTCFG g_UartCfg = {
	0,
	&s_UartPins,
	sizeof(s_UartPins)/sizeof(IOPINCFG),
	1000000,	// Rate
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, //  use APP_IRQ_PRIORITY_LOW with Softdevice
	nRFUartEvthandler,
	CFIFOMEMSIZE,
	g_RxBuff,
	CFIFOMEMSIZE,
	g_TxBuff,
};

#ifdef DEMO_C
// For C
UARTDEV g_UartDev;
#else
// For C++
// UART object instance
UART g_Uart;
#endif

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:

			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
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

#ifdef DEMO_C
	res = UARTInit(&g_UartDev, &g_UartCfg);
#else
	res = g_Uart.Init(g_UartCfg);
#endif

	uint8_t d = 0xff;

	while(1)
	{
#ifdef DEMO_C
		if (UARTTx(&g_UartDev, &d, 1) > 0)
#else
		if (g_Uart.Tx(&d, 1) > 0)
#endif
		{
			// If success send next code
			d = Prbs8(d);
		}
	}
	return 0;
}
