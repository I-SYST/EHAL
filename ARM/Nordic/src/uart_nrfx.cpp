/**-------------------------------------------------------------------------
@file	uart_nrf5x.c

@brief	nRF5x UART implementation

@author	Hoang Nguyen Hoan
@date	Aug. 30, 2015

@license

Copyright (c) 2015, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include "nrf.h"
#include "nrf_peripherals.h"

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "interrupt.h"
#include "coredev/shared_irq.h"

// There is no indication in the datasheet about how many hardware fifo
// this value seems to produce best performance
#define NRFX_UART_HWFIFO_SIZE		4

#define NRFX_UART_MAXDEV			UARTE_COUNT
#define NRFX_UART_DMA_MAXCNT		((1<<UARTE0_EASYDMA_MAXCNT_SIZE)-1)

// Default fifo size if one is not provided is not provided in the config.
#define NRFX_UART_BUFF_SIZE			(4 * NRFX_UART_HWFIFO_SIZE)

#define NRFX_UART_CFIFO_SIZE		CFIFO_MEMSIZE(NRFX_UART_BUFF_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct _nRF_UART_Dev {
	int DevNo;				// UART interface number
	union {
#ifdef UART_PRESENT
		NRF_UART_Type *pReg;		// UART registers
#endif
#ifdef UARTE_PRESENT
		NRF_UARTE_Type *pDmaReg;	// UART registers
#endif
	};
	UARTDEV	*pUartDev;				// Pointer to generic UART dev. data
	uint32_t RxPin;
	uint32_t TxPin;
	uint32_t CtsPin;
	uint32_t RtsPin;
	uint8_t RxFifoMem[NRFX_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[NRFX_UART_CFIFO_SIZE];
	uint8_t TxDmaCache[NRFX_UART_BUFF_SIZE];
	uint8_t RxDmaCache;
} NRFX_UARTDEV;

typedef struct {
	int Baud;
	int RegVal;
} NRFX_UARTRATE;

#pragma pack(pop)

static const NRFX_UARTRATE s_nRFxBaudrate[] = {
#ifdef UART_PRESENT
	{1200, UART_BAUDRATE_BAUDRATE_Baud1200},
	{2400, UART_BAUDRATE_BAUDRATE_Baud2400},
	{4800, UART_BAUDRATE_BAUDRATE_Baud4800},
	{9600, UART_BAUDRATE_BAUDRATE_Baud9600},
	{14400, UART_BAUDRATE_BAUDRATE_Baud14400},
	{19200, UART_BAUDRATE_BAUDRATE_Baud19200},
	{28800, UART_BAUDRATE_BAUDRATE_Baud28800},
	{38400, UART_BAUDRATE_BAUDRATE_Baud38400},
	{57600, UART_BAUDRATE_BAUDRATE_Baud57600},
	{76800, UART_BAUDRATE_BAUDRATE_Baud76800},
	{115200, UART_BAUDRATE_BAUDRATE_Baud115200},
	{230400, UART_BAUDRATE_BAUDRATE_Baud230400},
	{250000, UART_BAUDRATE_BAUDRATE_Baud250000},
	{460800, UART_BAUDRATE_BAUDRATE_Baud460800},
	{921600, UART_BAUDRATE_BAUDRATE_Baud921600},
	{1000000, UART_BAUDRATE_BAUDRATE_Baud1M}
#else
	{1200, UARTE_BAUDRATE_BAUDRATE_Baud1200},
	{2400, UARTE_BAUDRATE_BAUDRATE_Baud2400},
	{4800, UARTE_BAUDRATE_BAUDRATE_Baud4800},
	{9600, UARTE_BAUDRATE_BAUDRATE_Baud9600},
	{14400, UARTE_BAUDRATE_BAUDRATE_Baud14400},
	{19200, UARTE_BAUDRATE_BAUDRATE_Baud19200},
	{28800, UARTE_BAUDRATE_BAUDRATE_Baud28800},
	{38400, UARTE_BAUDRATE_BAUDRATE_Baud38400},
	{57600, UARTE_BAUDRATE_BAUDRATE_Baud57600},
	{76800, UARTE_BAUDRATE_BAUDRATE_Baud76800},
	{115200, UARTE_BAUDRATE_BAUDRATE_Baud115200},
	{230400, UARTE_BAUDRATE_BAUDRATE_Baud230400},
	{250000, UARTE_BAUDRATE_BAUDRATE_Baud250000},
	{460800, UARTE_BAUDRATE_BAUDRATE_Baud460800},
	{921600, UARTE_BAUDRATE_BAUDRATE_Baud921600},
	{1000000, UARTE_BAUDRATE_BAUDRATE_Baud1M}
#endif
};

static const int s_NbBaudrate = sizeof(s_nRFxBaudrate) / sizeof(NRFX_UARTRATE);

static NRFX_UARTDEV s_nRFxUARTDev[] = {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	{
		0,
		{ .pDmaReg = NRF_UARTE0_NS, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#else
	{
		0,
		{ .pDmaReg = NRF_UARTE0_S, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
	{
		1,
		{ .pDmaReg = NRF_UARTE1_S, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
	{
		2,
		{ .pDmaReg = NRF_UARTE2_S, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
	{
		3,
		{ .pDmaReg = NRF_UARTE3_S, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#endif
#else
	{
		0,
		{ .pReg = NRF_UART0, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#if NRFX_UART_MAXDEV > 1
	{
		1,
		{ .pDmaReg = NRF_UARTE1, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#endif
#if NRFX_UART_MAXDEV > 2
	{
		2,
		{ .pDmaReg = NRF_UARTE2, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#endif
#if NRFX_UART_MAXDEV > 3
	{
		3,
		{ .pDmaReg = NRF_UARTE3, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#endif
#endif
};

static const int s_NbUartDev = sizeof(s_nRFxUARTDev) / sizeof(NRFX_UARTDEV);

bool nRFUARTWaitForRxReady(NRFX_UARTDEV * const pDev, uint32_t Timeout)
{
#ifdef UART_PRESENT
	NRF_UART_Type *reg = pDev->pReg;
#else
	NRF_UARTE_Type *reg = pDev->pDmaReg;
#endif

	do {
		if (reg->EVENTS_RXDRDY || pDev->pUartDev->bRxReady)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRFUARTWaitForTxReady(NRFX_UARTDEV * const pDev, uint32_t Timeout)
{
#ifdef UART_PRESENT
	NRF_UART_Type *reg = pDev->pReg;
#else
	NRF_UARTE_Type *reg = pDev->pDmaReg;
#endif

	do {
		if (reg->EVENTS_TXDRDY || pDev->pUartDev->bTxReady == true)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

//static void UartIrqHandler(NRFX_UARTDEV * const pDev)
static void UartIrqHandler(int DevNo, DEVINTRF * const pDev)
{
	NRFX_UARTDEV *dev = (NRFX_UARTDEV *)pDev-> pDevData;
	int len = 0;
	int cnt = 0;

#ifdef UART_PRESENT
	NRF_UART_Type *reg = dev->pReg;
#else
	NRF_UARTE_Type *reg = dev->pDmaReg;
#endif

	uint8_t rxto = reg->EVENTS_RXTO;

	if (rxto)
	{
#ifdef UARTE_PRESENT
		dev->pDmaReg->TASKS_FLUSHRX = 1;
#endif
		reg->EVENTS_RXTO = 0;
	}

	if (reg->EVENTS_RXDRDY)
	{
#ifdef UART_PRESENT
		uint8_t *d;

		cnt = 0;
		do {
			reg->EVENTS_RXDRDY = 0;
			dev->pUartDev->bRxReady = false;

			d = CFifoPut(dev->pUartDev->hRxFifo);
			if (d == NULL)
			{
				dev->pUartDev->bRxReady = true;
				dev->pUartDev->RxDropCnt++;// g_nRF51RxDropCnt++;
				break;
			}
			*d = reg->RXD;
			cnt++;
		} while (reg->EVENTS_RXDRDY && cnt < NRFX_UART_HWFIFO_SIZE) ;

		if (dev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(dev->pUartDev->hRxFifo);
			if (dev->pReg->EVENTS_RXTO)
			{
				dev->pReg->EVENTS_RXTO = 0;
				dev->pUartDev->bRxReady = false;
				cnt = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXTIMEOUT, NULL, len);
			}
			else
				cnt = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, len);
		}
#else
		reg->EVENTS_RXDRDY = 0;
#endif
	}

#ifdef UARTE_PRESENT
	if (dev->pDmaReg->EVENTS_ENDRX)
	{
		// No DMA support for RX, just clear the event
		dev->pDmaReg->EVENTS_ENDRX = 0;
#ifndef UART_PRESENT
		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p)
		{
			*p = dev->RxDmaCache;
		}
		else
		{
			dev->pUartDev->RxDropCnt++;
		}

		// We need to transfer only 1 byte at a time for Rx. Otherwise, it will not interrupt
		// until buffer is filled. It will be blocked.
		// The RX timeout logic of the nRF series is implemented wrong. We cannot use it.
		dev->pUartDev->bRxReady = false;
		dev->pDmaReg->RXD.MAXCNT = 1;
		dev->pDmaReg->RXD.PTR = (uint32_t)&dev->RxDmaCache;
		dev->pDmaReg->TASKS_STARTRX = 1;

		if (dev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(dev->pUartDev->hRxFifo);
			cnt = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, len);
		}
#endif
	}
#endif

	if (reg->EVENTS_TXDRDY)
	{

		reg->EVENTS_TXDRDY = 0;
		cnt = 0;

#ifdef UART_PRESENT
		if (dev->pUartDev->DevIntrf.bDma == false)
		{
			do {
				dev->pReg->EVENTS_TXDRDY = 0;

				uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
				if (p == NULL)
				{
					dev->pUartDev->bTxReady = true;
					break;
				}
				dev->pUartDev->bTxReady = false;
				dev->pReg->TXD = *p;
				cnt++;
			} while (dev->pReg->EVENTS_TXDRDY && cnt < NRFX_UART_HWFIFO_SIZE);

			if (dev->pUartDev->EvtCallback)
			{
				//uint8_t buff[NRFUART_CFIFO_SIZE];

				//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFxUARTDev.pUartDev->hTxFifo));
				len = CFifoAvail(dev->pUartDev->hTxFifo);
				len = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, len);
				if (len > 0)
				{
					//s_nRFxUARTDev.bTxReady = false;
					//nRFUARTTxData(&s_nRFxUARTDev.pUartDev->SerIntrf, buff, len);
				}
			}
		}
#endif
	}

#ifdef UARTE_PRESENT
	if (dev->pDmaReg->EVENTS_ENDTX || dev->pDmaReg->EVENTS_TXSTOPPED)
	{
		dev->pDmaReg->EVENTS_ENDTX = 0;
		dev->pDmaReg->EVENTS_TXSTOPPED = 0;

		int l = NRFX_UART_BUFF_SIZE;//min(CFifoUsed(pDev->pUartDev->hTxFifo), NRF52_UART_DMA_MAX_LEN);
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
		if (p)
		{
			dev->pUartDev->bTxReady = false;

			// Transfer to tx cache before sending as CFifo will immediately make the memory
			// block available for reuse in the Put request. This could cause an overwrite
			// if uart tx has not completed in time.
			memcpy(&dev->TxDmaCache, p, l);

			dev->pDmaReg->TXD.MAXCNT = l;
			dev->pDmaReg->TXD.PTR = (uint32_t)dev->TxDmaCache;
			dev->pDmaReg->TASKS_STARTTX = 1;
		}
		else
		{
			dev->pUartDev->bTxReady = true;
		}
		if (dev->pUartDev->EvtCallback)
		{
			//uint8_t buff[NRFUART_CFIFO_SIZE];

			//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFxUARTDev.pUartDev->hTxFifo));
			len = CFifoAvail(dev->pUartDev->hTxFifo);
			len = dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_TXREADY, NULL, len);
			if (len > 0)
			{
				//s_nRFxUARTDev.bTxReady = false;
				//nRFUARTTxData(&s_nRFxUARTDev.pUartDev->SerIntrf, buff, len);
			}
		}
	}
#endif

	if (reg->EVENTS_ERROR)
	{
		uint32_t err = reg->ERRORSRC;
		reg->EVENTS_ERROR = 0;
#ifdef UART_PRESENT
		if (err & UART_ERRORSRC_OVERRUN_Msk)
		{
			dev->pUartDev->RxOvrErrCnt++;//g_nRF51RxErrCnt++;
			len = 0;
			cnt = 0;
			//int l = 0;
			uint8_t *d;
			do {
				dev->pReg->EVENTS_RXDRDY = 0;
				d = CFifoPut(dev->pUartDev->hRxFifo);
				if (d == NULL)
				{
					dev->pUartDev->bRxReady = true;
					break;
				}
				dev->pUartDev->bRxReady = false;
				*d = dev->pReg->RXD;
				cnt++;
			} while (dev->pReg->EVENTS_RXDRDY && cnt < NRFX_UART_HWFIFO_SIZE);

			if (dev->pUartDev->EvtCallback)
			{
				len = CFifoUsed(dev->pUartDev->hRxFifo);
				dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_RXDATA, NULL, len);
			}
		}
		if (err & UART_ERRORSRC_FRAMING_Msk)
		{
			dev->pUartDev->FramErrCnt++;
		}
		if (err & UART_ERRORSRC_PARITY_Msk)
		{
			dev->pUartDev->ParErrCnt++;
		}
#else
#endif
		reg->ERRORSRC = reg->ERRORSRC;
		len = 0;
		if (dev->pUartDev->EvtCallback)
		{
			dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, NULL, len);
		}
		reg->TASKS_STARTRX = 1;
	}

	if (reg->EVENTS_CTS)
	{
		reg->EVENTS_CTS = 0;
		dev->pUartDev->LineState &= ~UART_LINESTATE_CTS;
        if (dev->pUartDev->EvtCallback)
        {
            uint8_t buff = 0;
            len = 1;
            dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, &buff, len);
        }
		//NRF_UART0->TASKS_STARTTX = 1;
		//s_nRFxUARTDev.bTxReady = true;
	}

	if (reg->EVENTS_NCTS)
	{
		reg->EVENTS_NCTS = 0;
		dev->pUartDev->LineState |= UART_LINESTATE_CTS;
		//NRF_UART0->TASKS_STOPTX = 1;
        if (dev->pUartDev->EvtCallback)
        {
            uint8_t buff = UART_LINESTATE_CTS;
            len = 1;
            dev->pUartDev->EvtCallback(dev->pUartDev, UART_EVT_LINESTATE, &buff, len);
        }
	}
}

static uint32_t nRFUARTGetRate(DEVINTRF * const pDev)
{
	return ((NRFX_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static uint32_t nRFUARTSetRate(DEVINTRF * const pDev, uint32_t Rate)
{
	NRFX_UARTDEV *dev = (NRFX_UARTDEV *)pDev->pDevData;

	int rate = 0;

	for (int i = 0; i < s_NbBaudrate; i++)
	{
		if (s_nRFxBaudrate[i].Baud >= Rate)
		{
#ifdef UARTE_PRESENT
		    dev->pDmaReg->BAUDRATE = s_nRFxBaudrate[i].RegVal;
#else
		    dev->pReg->BAUDRATE = s_nRFxBaudrate[i].RegVal;
#endif
		    rate = s_nRFxBaudrate[i].Baud;
		    break;
		}
	}

	return rate;
}

static bool nRFUARTStartRx(DEVINTRF * const pSerDev, uint32_t DevAddr)
{
	return true;
}

static int nRFUARTRxData(DEVINTRF * const pDev, uint8_t *pBuff, int Bufflen)
{
	NRFX_UARTDEV *dev = (NRFX_UARTDEV *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen)
	{
		int l  = Bufflen;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &l);
		if (p == NULL)
			break;
		memcpy(pBuff, p, l);
		cnt += l;
		pBuff += l;
		Bufflen -= l;
	}
	EnableInterrupt(state);

	if (dev->pUartDev->bRxReady)
	{
#ifdef UART_PRESENT
		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p)
		{
			dev->pReg->EVENTS_RXDRDY = 0;
			dev->pUartDev->bRxReady = false;
			*p = dev->pReg->RXD;
		}
#else
		dev->pUartDev->bRxReady = false;
		dev->pDmaReg->RXD.MAXCNT = 1;
		dev->pDmaReg->RXD.PTR = (uint32_t)&dev->RxDmaCache;
		dev->pDmaReg->TASKS_STARTRX = 1;
#endif
	}

	return cnt;
}

static void nRFUARTStopRx(DEVINTRF * const pDev)
{
}

static bool nRFUARTStartTx(DEVINTRF * const pDev, uint32_t DevAddr)
{
	return true;
}

static int nRFUARTTxData(DEVINTRF * const pDev, uint8_t *pData, int Datalen)
{
	NRFX_UARTDEV *dev = (NRFX_UARTDEV *)pDev->pDevData;
    int cnt = 0;
    int rtry = pDev->MaxRetry;

    while (Datalen > 0 && rtry-- > 0)
    {
        uint32_t state = DisableInterrupt();

        while (Datalen > 0)
        {
            int l = Datalen;
            uint8_t *p = CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
            if (p == NULL)
            {
//            	dev->pUartDev->TxDropCnt++;
            	break;
            }
            memcpy(p, pData, l);
            Datalen -= l;
            pData += l;
            cnt += l;
        }
        EnableInterrupt(state);

        if (dev->pUartDev->bTxReady)
        {
#ifdef UARTE_PRESENT
        	if (pDev->bDma == true)
        	{
        		int l = NRFX_UART_BUFF_SIZE;//min(CFifoUsed(dev->pUartDev->hTxFifo), NRF52_UART_DMA_MAX_LEN);
        		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
        		if (p)
        		{
        			dev->pUartDev->bTxReady = false;

        			// Transfer to tx cache before sending as CFifo will immediately make the memory
        			// block available for reuse in the Put request. This could cause an overwrite
        			// if uart tx has not completed in time.
        			memcpy(dev->TxDmaCache, p, l);

					dev->pDmaReg->TXD.MAXCNT = l;
					dev->pDmaReg->TXD.PTR = (uint32_t)dev->TxDmaCache;
					dev->pDmaReg->TASKS_STARTTX = 1;
        		}
        	}
        	else
#endif
            //if (nRFUARTWaitForTxReady(dev, 1000))
            {
#ifdef UART_PRESENT
                dev->pReg->EVENTS_TXDRDY = 0;
                dev->pUartDev->bTxReady = true;
                uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
                if (p)
                {
                    dev->pUartDev->bTxReady = false;
                    dev->pReg->TXD = *p;
                }
#endif
            }
        }
    }

    if (rtry <= 0)
    {
    	dev->pUartDev->TxDropCnt += Datalen- cnt;
    }

    return cnt;
}

static void nRFUARTStopTx(DEVINTRF * const pDev)
{
}

static void nRFUARTDisable(DEVINTRF * const pDev)
{
	NRFX_UARTDEV *dev = (NRFX_UARTDEV *)pDev->pDevData;
#ifdef UART_PRESENT
	NRF_UART_Type *reg = dev->pReg;
	reg->TASKS_STOPRX = 1;
	reg->TASKS_STOPTX = 1;
	reg->PSELRXD = -1;
	reg->PSELTXD = -1;
	reg->PSELRTS = -1;
	reg->PSELCTS = -1;
#else
	NRF_UARTE_Type *reg = dev->pDmaReg;
	reg->TASKS_STOPRX = 1;
	reg->TASKS_STOPTX = 1;
	reg->PSEL.RXD = -1;
	reg->PSEL.TXD = -1;
	reg->PSEL.RTS = -1;
	reg->PSEL.CTS = -1;
#endif
	reg->ENABLE = 0;
}

static void nRFUARTEnable(DEVINTRF * const pDev)
{
	NRFX_UARTDEV *dev = (NRFX_UARTDEV *)pDev->pDevData;

	dev->pUartDev->RxOvrErrCnt = 0;
	dev->pUartDev->ParErrCnt = 0;
	dev->pUartDev->FramErrCnt = 0;
	dev->pUartDev->RxDropCnt = 0;
	dev->pUartDev->TxDropCnt = 0;

	CFifoFlush(dev->pUartDev->hTxFifo);

#ifdef UARTE_PRESENT
	if (pDev->bDma == true)
	{
		dev->pDmaReg->PSEL.RXD = dev->RxPin;
		dev->pDmaReg->PSEL.TXD = dev->TxPin;
		dev->pDmaReg->PSEL.CTS = dev->CtsPin;
		dev->pDmaReg->PSEL.RTS = dev->RtsPin;

		dev->pDmaReg->ENABLE |= (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);
		// Not using DMA transfer on Rx. It is useless on UART as we need to process 1 char at a time
#ifndef UART_PRESENT
		dev->pDmaReg->RXD.PTR = (uint32_t)&dev->RxDmaCache;
#endif
		dev->pDmaReg->EVENTS_ENDRX = 0;
		dev->pDmaReg->TASKS_STARTRX = 1;
	}
	else
#endif
	{
#ifdef UART_PRESENT
		dev->pReg->PSELRXD = dev->RxPin;
		dev->pReg->PSELTXD = dev->TxPin;
		dev->pReg->PSELCTS = dev->CtsPin;
		dev->pReg->PSELRTS = dev->RtsPin;

		dev->pReg->ENABLE |= (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
		dev->pReg->TASKS_STARTRX = 1;
		dev->pReg->TASKS_STARTTX = 1;
#endif
	}

	dev->pUartDev->bTxReady = true;
}

void nRFUARTPowerOff(DEVINTRF * const pDev)
{
	NRFX_UARTDEV *dev = (NRFX_UARTDEV *)pDev->pDevData;
#ifdef UART_PRESENT
	NRF_UART_Type *reg = dev->pReg;
#else
	NRF_UARTE_Type *reg = dev->pDmaReg;
#endif

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 0;
}

static void apply_workaround_for_enable_anomaly(NRFX_UARTDEV * const pDev)
{
#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF9160_XXAA)
    // Apply workaround for anomalies:
    // - nRF9160 - anomaly 23
    // - nRF5340 - anomaly 44
    volatile uint32_t const * rxenable_reg =
        (volatile uint32_t *)(((uint32_t)pDev->pDmaReg) + 0x564);
    volatile uint32_t const * txenable_reg =
        (volatile uint32_t *)(((uint32_t)pDev->pDmaReg) + 0x568);

    if (*txenable_reg == 1)
    {
    	pDev->pDmaReg->TASKS_STOPTX = 1;
    }

    if (*rxenable_reg == 1)
    {
    	pDev->pDmaReg->ENABLE = UARTE_ENABLE_ENABLE_Msk;
    	pDev->pDmaReg->TASKS_STOPRX = 1;

        while (*rxenable_reg) {}

        pDev->pDmaReg->ERRORSRC = pDev->pDmaReg->ERRORSRC;

        pDev->pDmaReg->ENABLE = 0;
    }
#endif // defined(NRF5340_XXAA_APPLICATION) || defined(NRF5340_XXAA_NETWORK) || defined(NRF9160_XXAA)
}

bool UARTInit(UARTDEV * const pDev, const UARTCFG *pCfg)
{
	// Config I/O pins
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pIOPinMap == NULL || pCfg->NbIOPins <= 0)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev)
	{
		return false;
	}

	int devno = pCfg->DevNo;
#ifdef UART_PRESENT
	NRF_UART_Type *reg = s_nRFxUARTDev[devno].pReg;
#else
	NRF_UARTE_Type *reg = s_nRFxUARTDev[devno].pDmaReg;
#endif

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_nRFxUARTDev[devno].RxFifoMem, NRFX_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_nRFxUARTDev[devno].TxFifoMem, NRFX_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIOPinMap;

	IOPinSet(pincfg[UARTPIN_TX_IDX].PortNo, pincfg[UARTPIN_TX_IDX].PinNo);
	IOPinCfg(pincfg, pCfg->NbIOPins);

	pDev->DevIntrf.pDevData = &s_nRFxUARTDev[devno];
	s_nRFxUARTDev[devno].pUartDev = pDev;

#ifndef UARTE_PRESENT
	pDev->DevIntrf.bDma = false;	// DMA not avail on nRF51

	s_nRFxUARTDev[devno].RxPin = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFxUARTDev[devno].TxPin = pincfg[UARTPIN_TX_IDX].PinNo;
	s_nRFxUARTDev[devno].pReg->PSELRXD = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFxUARTDev[devno].pReg->PSELTXD = pincfg[UARTPIN_TX_IDX].PinNo;

	s_nRFxUARTDev[devno].pReg->CONFIG &= ~(UART_CONFIG_PARITY_Msk << UART_CONFIG_PARITY_Pos);
	if (pCfg->Parity == UART_PARITY_NONE)
	{
		s_nRFxUARTDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos;
	}
	else
	{
		s_nRFxUARTDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos;
	}
#else
#ifndef UART_PRESENT
	// DMA mode avail only, force DMA
	pDev->DevIntrf.bDma = true;
#else
	pDev->DevIntrf.bDma = pCfg->bDMAMode;
#endif
	s_nRFxUARTDev[devno].RxPin = (pincfg[UARTPIN_RX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RX_IDX].PortNo << 5);
	s_nRFxUARTDev[devno].TxPin = (pincfg[UARTPIN_TX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_TX_IDX].PortNo << 5);
	s_nRFxUARTDev[devno].pDmaReg->PSEL.RXD = s_nRFxUARTDev[devno].RxPin;
	s_nRFxUARTDev[devno].pDmaReg->PSEL.TXD = s_nRFxUARTDev[devno].TxPin;
    s_nRFxUARTDev[devno].pDmaReg->CONFIG &= ~(UARTE_CONFIG_PARITY_Msk << UARTE_CONFIG_PARITY_Pos);

    if (pCfg->Parity == UART_PARITY_NONE)
	{
		s_nRFxUARTDev[devno].pDmaReg->CONFIG |= UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos;
	}
	else
	{
		s_nRFxUARTDev[devno].pDmaReg->CONFIG |= UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos;
	}
#endif

    // Set baud
    pDev->Rate = nRFUARTSetRate(&pDev->DevIntrf, pCfg->Rate);


	reg->EVENTS_RXDRDY = 0;
	reg->EVENTS_TXDRDY = 0;
	reg->EVENTS_ERROR = 0;
	reg->EVENTS_RXTO = 0;
	reg->ERRORSRC = reg->ERRORSRC;
	reg->EVENTS_CTS = 0;

#ifdef UARTE_PRESENT
	if (pDev->DevIntrf.bDma == true)
	{
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_RXSTARTED = 0;
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_TXSTARTED = 0;
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_TXSTOPPED = 0;
	}

#endif

#ifdef UART_PRESENT
    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_nRFxUARTDev[devno].pReg->CONFIG |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
    	s_nRFxUARTDev[devno].CtsPin = pincfg[UARTPIN_CTS_IDX].PinNo;
    	s_nRFxUARTDev[devno].RtsPin = pincfg[UARTPIN_RTS_IDX].PinNo;
    	s_nRFxUARTDev[devno].pReg->PSELCTS = pincfg[UARTPIN_CTS_IDX].PinNo;
    	s_nRFxUARTDev[devno].pReg->PSELRTS = pincfg[UARTPIN_RTS_IDX].PinNo;
	}
	else
	{
		s_nRFxUARTDev[devno].pReg->CONFIG &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
		s_nRFxUARTDev[devno].pReg->PSELRTS = -1;
		s_nRFxUARTDev[devno].pReg->PSELCTS = -1;
		s_nRFxUARTDev[devno].CtsPin = -1;
		s_nRFxUARTDev[devno].RtsPin = -1;
	}
#else
    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_nRFxUARTDev[devno].pDmaReg->CONFIG |= (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
    	s_nRFxUARTDev[devno].CtsPin = (pincfg[UARTPIN_CTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_CTS_IDX].PortNo << 5);
    	s_nRFxUARTDev[devno].RtsPin = (pincfg[UARTPIN_RTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RTS_IDX].PortNo << 5);
    	s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = s_nRFxUARTDev[devno].CtsPin;
    	s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = s_nRFxUARTDev[devno].RtsPin;

    	IOPinClear(pincfg[UARTPIN_CTS_IDX].PortNo, pincfg[UARTPIN_CTS_IDX].PinNo);
        IOPinClear(pincfg[UARTPIN_RTS_IDX].PortNo, pincfg[UARTPIN_RTS_IDX].PinNo);
	}
	else
	{
		s_nRFxUARTDev[devno].pDmaReg->CONFIG &= ~(UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
		s_nRFxUARTDev[devno].pDmaReg->PSEL.RTS = -1;
		s_nRFxUARTDev[devno].pDmaReg->PSEL.CTS = -1;
		s_nRFxUARTDev[devno].CtsPin = -1;
		s_nRFxUARTDev[devno].RtsPin = -1;
	}
#endif

	s_nRFxUARTDev[devno].pUartDev->bRxReady = false;
	s_nRFxUARTDev[devno].pUartDev->bTxReady = true;
	s_nRFxUARTDev[devno].pUartDev->RxOvrErrCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->ParErrCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->FramErrCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->RxDropCnt = 0;
	s_nRFxUARTDev[devno].pUartDev->TxDropCnt = 0;

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->bIntMode = pCfg->bIntMode;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->DevIntrf.Disable = nRFUARTDisable;
	pDev->DevIntrf.Enable = nRFUARTEnable;
	pDev->DevIntrf.GetRate = nRFUARTGetRate;
	pDev->DevIntrf.SetRate = nRFUARTSetRate;
	pDev->DevIntrf.StartRx = nRFUARTStartRx;
	pDev->DevIntrf.RxData = nRFUARTRxData;
	pDev->DevIntrf.StopRx = nRFUARTStopRx;
	pDev->DevIntrf.StartTx = nRFUARTStartTx;
	pDev->DevIntrf.TxData = nRFUARTTxData;
	pDev->DevIntrf.StopTx = nRFUARTStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = nRFUARTPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	apply_workaround_for_enable_anomaly(&s_nRFxUARTDev[devno]);

#ifdef UARTE_PRESENT
	if (pDev->DevIntrf.bDma == true)
	{
		s_nRFxUARTDev[devno].pDmaReg->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

#ifndef UART_PRESENT
		// We need to transfer only 1 byte at a time for Rx. Otherwise, it will not interrupt
		// until buffer is filled. It will be blocked.
		// The RX timeout logic of the nRF series is implemented wrong. We cannot use it.
		s_nRFxUARTDev[devno].pDmaReg->RXD.MAXCNT = 1;
		s_nRFxUARTDev[devno].pDmaReg->RXD.PTR = (uint32_t)&s_nRFxUARTDev[devno].RxDmaCache;
#endif
		s_nRFxUARTDev[devno].pDmaReg->EVENTS_ENDRX = 0;
		s_nRFxUARTDev[devno].pDmaReg->TASKS_STARTRX = 1;
	}
	else
#endif
	{
#ifdef UART_PRESENT
		s_nRFxUARTDev[devno].pReg->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
		s_nRFxUARTDev[devno].pReg->TASKS_STARTTX = 1;
		s_nRFxUARTDev[devno].pReg->TASKS_STARTRX = 1;
#endif

	}

    reg->INTENCLR = 0xffffffffUL;

	if (pCfg->bIntMode)
	{
#ifdef UARTE_PRESENT

#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
		SetSharedIntHandler(pCfg->DevNo, &pDev->DevIntrf, UartIrqHandler);
#endif

		if (pDev->DevIntrf.bDma == true)
		{
			s_nRFxUARTDev[devno].pDmaReg->INTENSET = (UARTE_INTENSET_RXDRDY_Set << UARTE_INTENSET_RXDRDY_Pos) |
							  (UARTE_INTENSET_RXTO_Set << UARTE_INTENSET_RXTO_Pos) |
							  (UARTE_INTENSET_TXDRDY_Set << UARTE_INTENSET_TXDRDY_Pos) |
							  (UARTE_INTENSET_ERROR_Set << UARTE_INTENSET_ERROR_Pos) |
							  (UARTE_INTENSET_CTS_Set << UARTE_INTENSET_CTS_Pos) |
							  (UARTE_INTENSET_NCTS_Set << UARTE_INTENSET_NCTS_Pos) |
							  (UARTE_INTENSET_ENDTX_Set << UARTE_INTENSET_ENDTX_Pos) |
							  (UARTE_INTENSET_ENDRX_Set << UARTE_INTENSET_ENDRX_Pos);
		}
		else
#endif
		{
#ifdef UART_PRESENT
			s_nRFxUARTDev[devno].pReg->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
							  (UART_INTENSET_RXTO_Set << UART_INTENSET_RXTO_Pos) |
							  (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
							  (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos) |
							  (UART_INTENSET_CTS_Set << UART_INTENSET_CTS_Pos) |
							  (UART_INTENSET_NCTS_Set << UART_INTENSET_NCTS_Pos);
#endif
		}

		switch (devno)
		{
#ifdef NRF91_SERIES
		case 0:
			NVIC_ClearPendingIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
			NVIC_SetPriority(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
			NVIC_SetPriority(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
			NVIC_SetPriority(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
			NVIC_SetPriority(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
			break;
#elif defined(NRF53_SERIES)
    		case 0:
                NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                break;
#ifdef NRF5340_XXAA_APPLICATION
    	    case 1:
                NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                NVIC_SetPriority(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                break;
    	    case 3:
                NVIC_ClearPendingIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                NVIC_SetPriority(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn, pCfg->IntPrio);
                NVIC_EnableIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                break;
#endif
#else
			case 0:
				NVIC_ClearPendingIRQ(UART0_IRQn);
				NVIC_SetPriority(UART0_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UART0_IRQn);
				break;
#ifdef NRF52840_XXAA
			case 1:
				NVIC_ClearPendingIRQ(UARTE1_IRQn);
				NVIC_SetPriority(UARTE1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UARTE1_IRQn);
				break;
#endif
#endif
        }
    }

	return true;
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState)
{
//	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->SerIntrf.pDevData;

}

UARTDEV * const UARTGetInstance(int DevNo)
{
	return s_nRFxUARTDev[DevNo].pUartDev;
}

#ifdef UART_PRESENT
extern "C" void UART0_IRQHandler()
{
	UartIrqHandler(0, &s_nRFxUARTDev[0].pUartDev->DevIntrf);
}
#endif

#ifdef NRF52840_XXAA
extern "C" void UARTE1_IRQHandler()
{
	UartIrqHandler(1, &s_nRFxUARTDev[1].pUartDev->DevIntrf);
}
#endif

