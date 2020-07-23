/**-------------------------------------------------------------------------
@example	uart_slip_prbs_tx.cpp

@brief	UART PRBS transmit test over SLIP protocol

This example sends PRBS byte though UART over SLIP protocole. The example
shows UART & SLIP interface use in both C and C++.

To compile in C, rename the file to .c and uncomment the line #define DEMO_C


@author	Hoang Nguyen Hoan
@date	Oct. 7, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#include "coredev/uart.h"
#include "prbs.h"
#include "slip_intrf.h"

// This include contain i/o definition the board in use
#include "board.h"

#define DEMO_C

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// Allocate FIFO memory
#define SLIPTEST_BUFSIZE		92
#define FIFOSIZE				CFIFO_MEMSIZE(256)
uint8_t g_TxBuff[FIFOSIZE];

// This defines the s_UartPortPins map and pin count.
// See board.h for target device specific definitions
static const UART_PORTPINS;

// UART configuration data
const UARTCFG g_UartCfg = {
	.DevNo = UART_NO,
	.pIOPinMap = s_UartPortPins,
	.NbIOPins = UART_PORTPIN_COUNT,
	.Rate = 460800,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = NULL,//nRFUartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = FIFOSIZE,
	.pTxMem = g_TxBuff,
	.bDMAMode = false,
};

#ifdef DEMO_C
// For C programming
UARTDEV g_UartDev;
SLIPDEV g_SlipDev;
#else
// For C++ object programming
// UART object instance
UART g_Uart;
Slip g_Slip;
#endif

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
//	uint8_t buff[SLIPTEST_BUFSIZE];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
//			UARTRx(pDev, buff, BufferLen);
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

	printf("UART Slip PRBS Demo\n");

#ifdef DEMO_C
	res = UARTInit(&g_UartDev, &g_UartCfg);
	SlipInit(&g_SlipDev, &g_UartDev.DevIntrf, true);
#else
	res = g_Uart.Init(g_UartCfg);
	g_Slip.Init(&g_Uart);
#endif

	uint8_t d = 0xff;
	uint8_t buff[SLIPTEST_BUFSIZE];
	uint32_t lcnt = 0;
	
	while(1)
	{
		for (int i = 0; i < SLIPTEST_BUFSIZE; i++)
		{
			d = Prbs8(d);
			buff[i] = d;
		}
#ifdef DEMO_C
		SlipTx(&g_SlipDev, buff, SLIPTEST_BUFSIZE);
#else
		g_Slip.Tx(0, buff, SLIPTEST_BUFSIZE);
#endif
		lcnt++;
		if ((lcnt & 0xff) == 0)
		{
			printf("cnt %u\n", lcnt);
		}
	}
	return 0;
}
