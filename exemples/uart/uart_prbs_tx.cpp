/**-------------------------------------------------------------------------
@example	uart_prbs_tx.cpp

@brief	UART PRBS transmit test

Demo code using EHAL library to do PRBS transmit test using UART


@author	Hoang Nguyen Hoan
@date	Aug. 31, 2016

@license

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

----------------------------------------------------------------------------*/

#include <stdio.h>

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "prbs.h"

// This include contain i/o definition the board in use
#include "board.h"

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define FIFOSIZE			CFIFO_MEMSIZE(64)

uint8_t g_TxBuff[FIFOSIZE];

static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
const UARTCFG g_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPINCFG),
	.Rate = 1000000,			// Rate
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,					// Stop bit
	.FlowControl = UART_FLWCTRL_HW,
	.bIntMode = true,
	.IntPrio = 1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	.EvtCallback = nRFUartEvthandler,
	.bFifoBlocking = true,				// fifo blocking mode
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = 0,//FIFOSIZE,
	.pTxMem = NULL,//g_TxBuff,
	.bDMAMode = false,
};

#ifdef DEMO_C
// For C programming
UARTDEV g_UartDev;
#else
// For C++ object programming
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
//			g_Uart.Tx((uint8_t*)"0123456789abcdefghijklmnopqrstuvwxyz\r\n", 38);
		if (g_Uart.Tx(&d, 1) > 0)
#endif
		{
			// If success send next code
			d = Prbs8(d);
		}
	}
	return 0;
}
