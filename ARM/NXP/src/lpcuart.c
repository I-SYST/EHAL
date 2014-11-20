/*--------------------------------------------------------------------------
File   : lpcuart.c

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : LPC UART implementation

Copyright (c) 2012, I-SYST inc., all rights reserved

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
#include <stdarg.h>
#include <stdio.h>

#include "lpcuart.h"

extern uint32_t SystemCoreClock;

int LpcUARTGetRate(UARTDEV *pDev)
{
	return pDev->Cfg.Rate;
}

int LpcUARTSetRate(UARTDEV *pDev, int Rate)
{
	uint32_t pclk = SystemCoreClock >> 2;
	uint32_t rate16 = Rate << 4;
	uint32_t dval, mval;
	uint32_t dl;
//	int32_t diff;
//	float err;

	// Note : PCLK is hardcoded default to CCLK / 4.  See LpcUARTInit
	// Baurate clock must be 16 * Baurate
	// Baudrate = PCLK / (16 * DL)
	// DL : clock divider.
	// DL = PCLK / (16 * Baudrate)
	// The division is not integer, adjustment is requirde for the fractional part
	// by the DivVal & MulVal. Hence
	// DL = PCLK / (16 * Baudrate * (1 + DivVal/MulVal))
	//
	// The fractional part is calculated as
	// (PCLK  % (16 * Baudrate)) / (16 * Baudrate)
	// Let's make it to be the ratio
	// DivVal / MulVal
	// DivVal = Reminder of PCLK / (16 * Baurate)
	// MulVal = 16 * Baurate
	// **** Note : do not pre-divide PCLK/16. We keep it in the multiplication
	// to prserve precision for integer division
	dval = pclk % rate16;

	if (dval > 0)
	{
		// The PCLK / (16 * Baudrate) is fractional
		// => dval = pclk % rate16;
		// mval = rate16;
		// now bring down equivalent ratio
		// dval / mval = 1 / new_mval;
		// new_mval = mval / dval
		// new_dval = 1
		mval = rate16 / dval;
		dval = 1;
		if (mval > 13)
			dval = 0;
	}
	dval &= 0xf;
	mval &= 0xf;

	dl = pclk / (rate16 + rate16 * dval / mval);

	pDev->pUartReg->LCR |= LPCUART_LCR_DLAB; 	// Enable Divisor Access
	pDev->pUartReg->DLL = dl & 0xff;
	pDev->pUartReg->DLM = (dl >> 8) & 0xff;
	pDev->pUartReg->FDR = dval | (mval << 4);
	pDev->pUartReg->LCR &= ~LPCUART_LCR_DLAB;	// Disable Divisor Access

	// Recalculate actual rate
	dl <<= 4;	// Mul by 16

	// recalculate real data rate
	pDev->Cfg.Rate = pclk / (dl + dl * dval / mval);

	//diff = pDev->Cfg.Rate - Rate;
	//err = (float)diff * 100.0 / Rate;

	return pDev->Cfg.Rate;
}

int LpcUARTRxData(UARTDEV *pDev, uint8_t *pBuff, int Bufflen)
{
	int idx = 0;

	while (idx < Bufflen)
	{
		if (!LpcUARTWaitForRxFifo(pDev, 10000))
			break;
		pBuff[idx] = pDev->pUartReg->RBR & 0xff;
		idx++;
	}

	return idx;
}

bool LpcUARTStartTx(UARTDEV *pDev)
{
	pDev->pUartReg->TER = LPCUART_TER_TXEN;

	return true;
}

int LpcUARTTxData(UARTDEV *pDev, uint8_t *pData, int Datalen)
{
	int idx = 0;

	while (idx < Datalen)
	{
		if (!LpcUARTWaitForTxFifo(pDev, 10000))
			break;
		pDev->pUartReg->THR = pData[idx] & 0xff;
		idx++;
	}

	return idx;
}

void LpcUARTStopTx(UARTDEV *pDev)
{
	pDev->pUartReg->TER = 0;
}

bool LpcUARTWaitForRxFifo(UARTDEV *pDev, int Timeout)
{
	do
	{
		if (pDev->pUartReg->LSR & LPCUART_LSR_RDR)
			return true;
	} while (--Timeout > 0);

	return false;
}

bool LpcUARTWaitForTxFifo(UARTDEV *pDev, int Timeout)
{
	do
	{
		if (pDev->pUartReg->LSR & LPCUART_LSR_THRE)
			return true;
	} while (--Timeout > 0);

	return false;
}

UART_STATUS LpcUARTGetStatus(UARTDEV *pDev)
{
	return pDev->pUartReg->LSR;
}

#define UARTPRINTF_BUFFMAX		256
char g_UARTPrintfBuffer[UARTPRINTF_BUFFMAX] = {0,};// __attribute__ ((section(".RAMAHB"))) = { 0, };

void LpcUARTprintf(UARTDEV *pDev, char *pFormat, ...)
{
	va_list vl;
    va_start(vl, pFormat);
    LpcUARTvprintf(pDev, pFormat, vl);
    va_end(vl);
}

void LpcUARTvprintf(UARTDEV *pDev, char *pFormat, va_list vl)
{
    vsnprintf(g_UARTPrintfBuffer, UARTPRINTF_BUFFMAX, pFormat, vl);
    LpcUARTStartTx(pDev);
    LpcUARTTxData(pDev, (uint8_t*)g_UARTPrintfBuffer, strlen(g_UARTPrintfBuffer));
    LpcUARTStopTx(pDev);
}
