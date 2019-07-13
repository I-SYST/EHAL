/*--------------------------------------------------------------------------
File   : uart_lpcxx.c

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
#include <stdbool.h>

#include "cmsis_gcc.h"
#include "istddef.h"
#include "uart_lpcxx.h"
#include "interrupt.h"
#include "idelay.h"

extern uint32_t SystemCoreClock;
extern uint32_t SystemClkFreq;

void UARTSetCtrlLineState(UARTDEV *pDev, uint32_t LineState)
{
	LPCUARTDEV *dev = (LPCUARTDEV*)pDev->DevIntrf.pDevData;

	dev->pUartReg->MCR &= ~3;
	dev->pUartReg->MCR |= LineState & 3;
}

int LpcUARTGetRate(DEVINTRF *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((LPCUARTDEV*)pDev->pDevData)->pUartDev->Rate;

	return rate;
}

#if 0
int LpcUARTSetRate(DEVINTRF *pDev, int Rate)
{
	uint32_t pclk = LpcGetUartClk();//SystemClkFreq  / g_UartClkDiv;// >> 2;
	uint32_t rate16 = Rate << 4;
	uint32_t dval, mval;
	uint32_t dl;
	LPCUARTDEV *dev = (LPCUARTDEV*)pDev->pDevData;

	if (Rate <= 0)
	{
		dev->pUartReg->ACR = 7;	// Auto rate
		return 0;
	}

	// The fractional is calculated as
	// (PCLK  % (16 * Baudrate)) / (16 * Baudrate)
	// Let's make it to be the ratio
	// DivVal / MulVal
	//
	dval = pclk % rate16;
	mval = rate16;
	// The PCLK / (16 * Baudrate) is fractional
	// => dval = pclk % rate16;
	// mval = rate16;
	// now mormalize the ratio
	// dval / mval = 1 / new_mval;
	// new_mval = mval / dval
	// new_dval = 1
	if (dval > 0)
	{
		mval = rate16 / dval;
		dval = 1;

		// in case mval still bigger then 4 bits
		// no adjustment require
		if (mval > 15)
			dval = 0;
	}
	dval &= 0xf;
	mval &= 0xf;

	dl = pclk / (rate16 + rate16 * dval / mval);

	//dl = pclk / rate16;

	dev->pUartReg->LCR |= LPCUART_LCR_DLAB; 	// Enable Divisor Access
	dev->pUartReg->DLL = dl & 0xff;
	dev->pUartReg->DLM = (dl >> 8) & 0xff;
	dev->pUartReg->LCR &= ~LPCUART_LCR_DLAB;	// Disable Divisor Access
	dev->pUartReg->FDR = dval | (mval << 4);

	// Recalculate actual rate
	dl <<= 4;	// Mul by 16

	// recalculate real data rate
	dev->pUartDev->Rate = pclk / (dl + dl * dval / mval);

	uint32_t diff = dev->pUartDev->Rate - Rate;
	float err = (float)diff * 100.0 / Rate;

	//printf("%d Rate : %d, %d\r\n", pclk, dev->pUartDev->Rate, Rate);
	return dev->pUartDev->Rate;
}
#else
int LpcUARTSetRate(DEVINTRF *pDev, int Rate)
{
	uint32_t pclk = LpcGetUartClk();
	uint32_t rate16 = Rate << 4;
	uint32_t dval, mval;
	uint32_t dl;
	LPCUARTDEV *dev = (LPCUARTDEV*)pDev->pDevData;

	// Return current rate if out of range
	if (Rate < 110 || Rate > 3000000)
		return dev->pUartDev->Rate;

	// The fractional is calculated as
	// (PCLK  % (16 * Baudrate)) / (16 * Baudrate)
	// Let's make it to be the ratio
	// DivVal / MulVal
	//
	dval = pclk % rate16;
	mval = rate16;
	// The PCLK / (16 * Baudrate) is fractional
	// => dval = pclk % rate16;
	// mval = rate16;
	// now mormalize the ratio
	// dval / mval = 1 / new_mval;
	// new_mval = mval / dval
	// new_dval = 1
	if (dval > 0)
	{
		// Find closest fractional
		int diff = 0x7FFFFFF;
		for (uint32_t dv = 1; dv <= 15; dv++)
		{
			for (uint32_t mv = 1; mv <= 15; mv++)
			{
				uint32_t div = (rate16 + rate16 * dv / mv);
				uint32_t x = (pclk + (div >> 1)) / div;
				x <<= 4;
				// recalculate real data rate
				div = (x + x * dv / mv);
				int r = (pclk + (div >> 1)) / div;
				int rd = Rate < r ? r - Rate : Rate - r;
				if (rd < diff)
				{
					mval = mv;
					dval = dv;
					diff = rd;
				}
			}
		}
	}
	dval &= 0xf;
	mval &= 0xf;

	int div = (rate16 + rate16 * dval / mval);
	dl = (pclk + (div >> 1)) / div;
	//dl = pclk / div;

	rate16 = dl << 4;
	Rate = pclk / (rate16 + rate16 * dval / mval);

	if (Rate == dev->pUartDev->Rate)
		return dev->pUartDev->Rate;

//	printf("rate = %d\r\n", Rate);

	dev->pUartReg->TER = 0;

	int tout = 10000;

	do {
		__NOP();
	} while ((dev->pUartReg->LSR & LPCUART_LSR_THRE) == 0 && tout-- > 0);

	dev->bTxReady = true;

	tout = 10000;
	uint32_t lcr = dev->pUartReg->LCR & ~LPCUART_LCR_DLAB;

	do {
		dev->pUartReg->LCR = lcr | LPCUART_LCR_DLAB; 	// Enable Divisor Access
	} while ((dev->pUartReg->LCR & LPCUART_LCR_DLAB) == 0 && tout-- > 0);

	dev->pUartReg->DLL = dl & 0xff;
	dev->pUartReg->DLM = (dl >> 8) & 0xff;
	dev->pUartReg->FDR = dval | (mval << 4);

	tout = 10000;
	do {
		dev->pUartReg->LCR = lcr;	// Disable Divisor Access
	} while ((dev->pUartReg->LCR & LPCUART_LCR_DLAB) && tout-- > 0);

	usDelay(10000);


	// Recalculate actual rate
	dl <<= 4;	// Mul by 16

	// recalculate real data rate
	dev->pUartDev->Rate = pclk / (dl + dl * dval / mval);

	//uint32_t diff = dev->pUartDev->Rate - Rate;
	//float err = (float)diff * 100.0 / Rate;
	//printf("calc Rate : %d\r\n", dev->pUartDev->Rate);

	dev->pUartReg->TER = LPCUART_TER_TXEN;

	return dev->pUartDev->Rate;
}
#endif

int LpcUARTRxData(DEVINTRF *pDev, uint8_t *pBuff, int Bufflen)
{
	LPCUARTDEV *dev = (LPCUARTDEV*)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen > 0)
	{
		int l = Bufflen;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &l);
		if (p)
		{
			memcpy(pBuff, p, l);
			Bufflen -= l;
			pBuff += l;
			cnt += l;
		}
		else
		{
			break;
		}
	}
	EnableInterrupt(state);

	while (Bufflen > 0)
	{
		if (!LpcUARTWaitForRxFifo(dev, 1))
			break;
		*pBuff = (uint8_t)(dev->pUartReg->RBR & 0xff);
		Bufflen--;
		pBuff++;
		cnt++;
	}

	return cnt;
}

int LpcUARTTxData(DEVINTRF *pDev, uint8_t *pData, int Datalen)
{
	LPCUARTDEV *dev = (LPCUARTDEV*)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Datalen > 0)
	{
		int l = Datalen;
		uint8_t *p = CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
		if (p == NULL)
			break;
		memcpy(p, pData, l);
		Datalen -= l;
		pData += l;
		cnt += l;
	}
	EnableInterrupt(state);
	//while (l < Datalen)
	if (dev->bTxReady)
	{
		if (LpcUARTWaitForTxFifo(dev, 1000))
		{
			uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
			if (p)
			{
				dev->bTxReady = false;
				dev->pUartReg->THR = *p;
			}
		}
	}

	return cnt;
}

bool LpcUARTWaitForRxFifo(LPCUARTDEV *pDev, uint32_t Timeout)
{
	do
	{
		if (pDev->pUartReg->LSR & LPCUART_LSR_RDR)
			return true;
	} while (Timeout-- > 0);

	return false;
}

bool LpcUARTWaitForTxFifo(LPCUARTDEV *pDev, uint32_t Timeout)
{
	do
	{
		//uint32_t lsr = pDev->pUartReg->LSR & LPCUART_LSR_THRE;
		if (pDev->pUartReg->LSR & (LPCUART_LSR_TEMT | LPCUART_LSR_THRE))
			return true;
	} while (Timeout-- > 0);

	return false;
}

UART_STATUS LpcUARTGetStatus(LPCUARTDEV *pDev)
{
	return pDev->pUartReg->LSR;
}
