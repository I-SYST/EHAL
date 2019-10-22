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

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "interrupt.h"

// There is no indication in the datasheet about how many hardware fifo
// this value seems to produce best performance
#define NRF5X_UART_HWFIFO_SIZE		4

#ifdef NRF52840_XXAA
#define NRF52_UART_DMA_MAX_LEN		65535
#else
#define NRF52_UART_DMA_MAX_LEN		255
#endif

// Default fifo size if one is not provided is not provided in the config.
#define NRF5X_UART_BUFF_SIZE		(4 * NRF5X_UART_HWFIFO_SIZE)

#define NRF5X_UART_CFIFO_SIZE		CFIFO_MEMSIZE(NRF5X_UART_BUFF_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct _nRF_UART_Dev {
	int DevNo;				// UART interface number
	union {
		NRF_UART_Type *pReg;		// UART registers
#ifndef NRF51
		NRF_UARTE_Type *pDmaReg;	// UART registers
#endif
	};
	UARTDEV	*pUartDev;				// Pointer to generic UART dev. data
	uint32_t RxDropCnt;
	uint32_t RxTimeoutCnt;
	uint32_t TxDropCnt;
	uint32_t ErrCnt;
	uint32_t RxPin;
	uint32_t TxPin;
	uint32_t CtsPin;
	uint32_t RtsPin;
	uint8_t TxDmaCache[NRF5X_UART_BUFF_SIZE];
	uint8_t RxFifoMem[NRF5X_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[NRF5X_UART_CFIFO_SIZE];
} NRF5X_UARTDEV;

typedef struct {
	int Baud;
	int nRFBaud;
} NRFRATECVT;

#pragma pack(pop)

const NRFRATECVT s_BaudnRF[] = {
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
};

static const int s_NbBaudnRF = sizeof(s_BaudnRF) / sizeof(NRFRATECVT);

static NRF5X_UARTDEV s_nRFUartDev[] = {
	{
		0,
		{ .pReg = NRF_UART0, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#ifdef NRF52840_XXAA
	{
		1,
		{ .pDmaReg = NRF_UARTE1, },
		NULL,
		0, 0, 0,
		false,
		true,
	},
#endif
};

static const int s_NbUartDev = sizeof(s_nRFUartDev) / sizeof(NRF5X_UARTDEV);

bool nRFUARTWaitForRxReady(NRF5X_UARTDEV * const pDev, uint32_t Timeout)
{
	do {
		if (pDev->pReg->EVENTS_RXDRDY || pDev->pUartDev->bRxReady)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRFUARTWaitForTxReady(NRF5X_UARTDEV * const pDev, uint32_t Timeout)
{
	do {
		if (pDev->pReg->EVENTS_TXDRDY || pDev->pUartDev->bTxReady == true)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static void UART_IRQHandler(NRF5X_UARTDEV * const pDev)
{
	int len = 0;
	int cnt = 0;
#ifdef NRF52_SERIES
	uint8_t rxto = pDev->pUartDev->DevIntrf.bDma == true ? pDev->pDmaReg->EVENTS_RXTO : pDev->pReg->EVENTS_RXTO;

	if (rxto && pDev->pUartDev->DevIntrf.bDma == true)
	{
		pDev->RxTimeoutCnt++;
		pDev->pDmaReg->TASKS_FLUSHRX = 1;
		pDev->pDmaReg->EVENTS_RXTO = 0;
	}
#else
	uint8_t rxto = pDev->pReg->EVENTS_RXTO;
#endif

	if (pDev->pReg->EVENTS_RXDRDY || rxto)
	{
		uint8_t *d;

		cnt = 0;
		do {
			pDev->pReg->EVENTS_RXDRDY = 0;
			pDev->pUartDev->bRxReady = false;

			d = CFifoPut(pDev->pUartDev->hRxFifo);
			if (d == NULL)
			{
				pDev->pUartDev->bRxReady = true;
				pDev->RxDropCnt++;// g_nRF51RxDropCnt++;
				break;
			}
			*d = pDev->pReg->RXD;
			cnt++;
		} while (pDev->pReg->EVENTS_RXDRDY && cnt < NRF5X_UART_HWFIFO_SIZE) ;

		if (pDev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(pDev->pUartDev->hRxFifo);
			if (pDev->pReg->EVENTS_RXTO)
			{
				pDev->pReg->EVENTS_RXTO = 0;
				pDev->pUartDev->bRxReady = false;
				cnt = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXTIMEOUT, NULL, len);
			}
			else
				cnt = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, NULL, len);
		}
	}

#ifdef NRF52_SERIES
	if (pDev->pDmaReg->EVENTS_ENDRX)
	{
		// No DMA support for RX, just clear the event
		pDev->pDmaReg->EVENTS_ENDRX = 0;
#if 0
		int cnt = pDev->pDmaReg->RXD.AMOUNT;
		uint8_t *pcache = pDev->RxDmaCache;

		while (cnt > 0)
		{
			int l = cnt;
			uint8_t *p = CFifoPutMultiple(pDev->pUartDev->hRxFifo, &l);
			if (p == NULL)
			{
				break;
			}

			memcpy(p, pcache, l);
			pcache += l;
			cnt -= l;
		}

		if (cnt > 0)
		{
			memcpy(pDev->RxDmaCache, pcache, cnt);
		}
		pDev->bRxReady = false;
		pDev->pDmaReg->RXD.MAXCNT = NRF5X_UART_BUFF_SIZE - cnt;
		pDev->pDmaReg->RXD.PTR = (uint32_t)&pDev->RxDmaCache[cnt];
		pDev->pDmaReg->TASKS_STARTRX = 1;

		if (pDev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(pDev->pUartDev->hRxFifo);
			if (pDev->pReg->EVENTS_RXTO)
			{
				pDev->pReg->EVENTS_RXTO = 0;
				pDev->bRxReady = false;
				cnt = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXTIMEOUT, NULL, len);
			}
			else
				cnt = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, NULL, len);
		}
#endif
	}
#endif

	if (pDev->pReg->EVENTS_TXDRDY)
	{

		pDev->pReg->EVENTS_TXDRDY = 0;
		cnt = 0;

#ifdef NRF52_SERIES
		if (pDev->pUartDev->DevIntrf.bDma == false)
#endif
		{
			do {
				pDev->pReg->EVENTS_TXDRDY = 0;

				uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);
				if (p == NULL)
				{
					pDev->pUartDev->bTxReady = true;
					break;
				}
				pDev->pUartDev->bTxReady = false;
				pDev->pReg->TXD = *p;
				cnt++;
			} while (pDev->pReg->EVENTS_TXDRDY && cnt < NRF5X_UART_HWFIFO_SIZE);

			if (pDev->pUartDev->EvtCallback)
			{
				//uint8_t buff[NRFUART_CFIFO_SIZE];

				//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFUartDev.pUartDev->hTxFifo));
				len = CFifoAvail(pDev->pUartDev->hTxFifo);
				len = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, len);
				if (len > 0)
				{
					//s_nRFUartDev.bTxReady = false;
					//nRFUARTTxData(&s_nRFUartDev.pUartDev->SerIntrf, buff, len);
				}
			}
		}
	}

#ifdef NRF52_SERIES
	if (pDev->pDmaReg->EVENTS_ENDTX || pDev->pDmaReg->EVENTS_TXSTOPPED)
	{
		pDev->pDmaReg->EVENTS_ENDTX = 0;
		pDev->pDmaReg->EVENTS_TXSTOPPED = 0;

		int l = NRF5X_UART_BUFF_SIZE;//min(CFifoUsed(pDev->pUartDev->hTxFifo), NRF52_UART_DMA_MAX_LEN);
		uint8_t *p = CFifoGetMultiple(pDev->pUartDev->hTxFifo, &l);
		if (p)
		{
			pDev->pUartDev->bTxReady = false;

			// Transfer to tx cache before sending as CFifo will immediately make the memory
			// block available for reuse in the Put request. This could cause an overwrite
			// if uart tx has not completed in time.
			memcpy(&pDev->TxDmaCache, p, l);

			pDev->pDmaReg->TXD.MAXCNT = l;
			pDev->pDmaReg->TXD.PTR = (uint32_t)pDev->TxDmaCache;
			pDev->pDmaReg->TASKS_STARTTX = 1;
		}
		else
		{
			pDev->pUartDev->bTxReady = true;
		}
		if (pDev->pUartDev->EvtCallback)
		{
			//uint8_t buff[NRFUART_CFIFO_SIZE];

			//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFUartDev.pUartDev->hTxFifo));
			len = CFifoAvail(pDev->pUartDev->hTxFifo);
			len = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, len);
			if (len > 0)
			{
				//s_nRFUartDev.bTxReady = false;
				//nRFUARTTxData(&s_nRFUartDev.pUartDev->SerIntrf, buff, len);
			}
		}
	}
#endif

	if (pDev->pReg->EVENTS_ERROR)
	{
		pDev->pReg->EVENTS_ERROR = 0;
		if (pDev->pReg->ERRORSRC & 1)	// Overrrun
		{
			pDev->ErrCnt++;//g_nRF51RxErrCnt++;
			len = 0;
			cnt = 0;
			//int l = 0;
			uint8_t *d;
			do {
				pDev->pReg->EVENTS_RXDRDY = 0;
				d = CFifoPut(pDev->pUartDev->hRxFifo);
				if (d == NULL)
				{
					pDev->pUartDev->bRxReady = true;
					break;
				}
				pDev->pUartDev->bRxReady = false;
				*d = pDev->pReg->RXD;
				cnt++;
			} while (pDev->pReg->EVENTS_RXDRDY && cnt < NRF5X_UART_HWFIFO_SIZE);

			if (pDev->pUartDev->EvtCallback)
			{
				len = CFifoUsed(pDev->pUartDev->hRxFifo);
				pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, NULL, len);
			}
		}
		pDev->pReg->ERRORSRC = pDev->pReg->ERRORSRC;
		len = 0;
		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_LINESTATE, NULL, len);
		}
		pDev->pReg->TASKS_STARTRX = 1;
	}

	if (pDev->pReg->EVENTS_CTS)
	{
		pDev->pReg->EVENTS_CTS = 0;
		pDev->pUartDev->LineState &= ~UART_LINESTATE_CTS;
        if (pDev->pUartDev->EvtCallback)
        {
            uint8_t buff = 0;//UART_LINESTATE_CTS;
    //        buff[1] = 0;//UART_LINESTATE_CTS;
            len = 1;
            pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_LINESTATE, &buff, len);
        }
		//NRF_UART0->TASKS_STARTTX = 1;
		//s_nRFUartDev.bTxReady = true;
	}

	if (pDev->pReg->EVENTS_NCTS)
	{
		pDev->pReg->EVENTS_NCTS = 0;
		pDev->pUartDev->LineState |= UART_LINESTATE_CTS;
		//NRF_UART0->TASKS_STOPTX = 1;
        if (pDev->pUartDev->EvtCallback)
        {
            uint8_t buff = UART_LINESTATE_CTS;
//            buff[1] = UART_LINESTATE_CTS;
            len = 1;
            pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_LINESTATE, &buff, len);
        }
	}
}

#ifdef __cplusplus
extern "C" {
   void UART0_IRQHandler();
   void UARTE1_IRQHandler();
}
#endif	// __cplusplus

void UART0_IRQHandler()
{
	UART_IRQHandler(&s_nRFUartDev[0]);
}

#ifdef NRF52840_XXAA
void UARTE1_IRQHandler()
{
	UART_IRQHandler(&s_nRFUartDev[1]);
}
#endif

static inline int nRFUARTGetRate(DEVINTRF * const pDev) {
	return ((NRF5X_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static int nRFUARTSetRate(DEVINTRF * const pDev, int Rate)
{
	NRF5X_UARTDEV *dev = (NRF5X_UARTDEV *)pDev->pDevData;

	int rate = 0;

	for (int i = 0; i < s_NbBaudnRF; i++)
	{
		if (s_BaudnRF[i].Baud >= Rate)
		{
		    dev->pReg->BAUDRATE = (s_BaudnRF[i].nRFBaud << UART_BAUDRATE_BAUDRATE_Pos);

		    rate = s_BaudnRF[i].Baud;
		    break;
		}
	}

	return rate;
}

static inline bool nRFUARTStartRx(DEVINTRF * const pSerDev, int DevAddr) {
	return true;
}

static int nRFUARTRxData(DEVINTRF * const pDev, uint8_t *pBuff, int Bufflen)
{
	NRF5X_UARTDEV *dev = (NRF5X_UARTDEV *)pDev->pDevData;
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
		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p)
		{
			dev->pReg->EVENTS_RXDRDY = 0;
			dev->pUartDev->bRxReady = false;
			*p = dev->pReg->RXD;
		}
	}

	return cnt;
}

static inline void nRFUARTStopRx(DEVINTRF * const pDev) {
}

static inline bool nRFUARTStartTx(DEVINTRF * const pDev, int DevAddr) {
	return true;
}

static int nRFUARTTxData(DEVINTRF * const pDev, uint8_t *pData, int Datalen)
{
	NRF5X_UARTDEV *dev = (NRF5X_UARTDEV *)pDev->pDevData;
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
                break;
            memcpy(p, pData, l);
            Datalen -= l;
            pData += l;
            cnt += l;
        }
        EnableInterrupt(state);

        if (dev->pUartDev->bTxReady)
        {
#ifdef NRF52_SERIES
        	if (pDev->bDma == true)
        	{
        		int l = NRF5X_UART_BUFF_SIZE;//min(CFifoUsed(dev->pUartDev->hTxFifo), NRF52_UART_DMA_MAX_LEN);
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
                dev->pReg->EVENTS_TXDRDY = 0;
                dev->pUartDev->bTxReady = true;
                uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
                if (p)
                {
                    dev->pUartDev->bTxReady = false;
                    dev->pReg->TXD = *p;
                }
            }
        }
    }
    return cnt;
}

static inline void nRFUARTStopTx(DEVINTRF * const pDev) {
}

static void nRFUARTDisable(DEVINTRF * const pDev)
{
	NRF5X_UARTDEV *dev = (NRF5X_UARTDEV *)pDev->pDevData;

	dev->pReg->TASKS_STOPRX = 1;
	dev->pReg->TASKS_STOPTX = 1;

	dev->pReg->PSELRXD = -1;
	dev->pReg->PSELTXD = -1;
	dev->pReg->PSELRTS = -1;
	dev->pReg->PSELCTS = -1;

	dev->pReg->ENABLE = 0;
}

static void nRFUARTEnable(DEVINTRF * const pDev)
{
	NRF5X_UARTDEV *dev = (NRF5X_UARTDEV *)pDev->pDevData;

	dev->ErrCnt = 0;
	dev->RxTimeoutCnt = 0;
	dev->RxDropCnt = 0;
	dev->TxDropCnt = 0;

	dev->pReg->PSELRXD = dev->RxPin;
	dev->pReg->PSELTXD = dev->TxPin;
	dev->pReg->PSELCTS = dev->CtsPin;
	dev->pReg->PSELRTS = dev->RtsPin;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;
#ifdef NRF52_SERIES
	if (pDev->bDma == true)
	{
		dev->pDmaReg->ENABLE |= (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);
		// Not using DMA transfer on Rx. It is useless on UART as we need to process 1 char at a time
		dev->pDmaReg->EVENTS_ENDRX = 0;
		dev->pDmaReg->TASKS_STARTRX = 1;
	}
	else
#endif
	{
		dev->pReg->ENABLE |= (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
		dev->pReg->TASKS_STARTRX = 1;
		dev->pReg->TASKS_STARTTX = 1;
	}
}

void nRFUARTPowerOff(DEVINTRF * const pDev)
{
	NRF5X_UARTDEV *dev = (NRF5X_UARTDEV *)pDev->pDevData;

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 0;
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

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)s_nRFUartDev[devno].pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)s_nRFUartDev[devno].pReg + 0xFFC) = 1;

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_nRFUartDev[devno].RxFifoMem, NRF5X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_nRFUartDev[devno].TxFifoMem, NRF5X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIOPinMap;

	IOPinSet(pincfg[UARTPIN_TX_IDX].PortNo, pincfg[UARTPIN_TX_IDX].PinNo);
	IOPinCfg(pincfg, pCfg->NbIOPins);

	pDev->DevIntrf.pDevData = &s_nRFUartDev[devno];
	s_nRFUartDev[devno].pUartDev = pDev;

#ifdef NRF51
	pDev->DevIntrf.bDma = false;	// DMA not avail on nRF51

	s_nRFUartDev[devno].RxPin = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFUartDev[devno].TxPin = pincfg[UARTPIN_TX_IDX].PinNo;
	s_nRFUartDev[devno].pReg->PSELRXD = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFUartDev[devno].pReg->PSELTXD = pincfg[UARTPIN_TX_IDX].PinNo;
#else
	pDev->DevIntrf.bDma = devno == 0 ? pCfg->bDMAMode : true;

	s_nRFUartDev[devno].RxPin = (pincfg[UARTPIN_RX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RX_IDX].PortNo << 5);
	s_nRFUartDev[devno].TxPin = (pincfg[UARTPIN_TX_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_TX_IDX].PortNo << 5);
	s_nRFUartDev[devno].pReg->PSELRXD = s_nRFUartDev[devno].RxPin;
	s_nRFUartDev[devno].pReg->PSELTXD = s_nRFUartDev[devno].TxPin;
#endif

    // Set baud
    pDev->Rate = nRFUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

    s_nRFUartDev[devno].pReg->CONFIG &= ~(UART_CONFIG_PARITY_Msk << UART_CONFIG_PARITY_Pos);
	if (pCfg->Parity == UART_PARITY_NONE)
	{
		s_nRFUartDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos;
	}
	else
	{
		s_nRFUartDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos;
	}

	s_nRFUartDev[devno].pReg->EVENTS_RXDRDY = 0;
	s_nRFUartDev[devno].pReg->EVENTS_TXDRDY = 0;
	s_nRFUartDev[devno].pReg->EVENTS_ERROR = 0;
	s_nRFUartDev[devno].pReg->EVENTS_RXTO = 0;
	s_nRFUartDev[devno].pReg->ERRORSRC = NRF_UART0->ERRORSRC;
	s_nRFUartDev[devno].pReg->EVENTS_CTS = 0;

#ifdef NRF52_SERIES
	if (pDev->DevIntrf.bDma == true)
	{
		s_nRFUartDev[devno].pDmaReg->EVENTS_RXSTARTED = 0;
		s_nRFUartDev[devno].pDmaReg->EVENTS_TXSTARTED = 0;
		s_nRFUartDev[devno].pDmaReg->EVENTS_TXSTOPPED = 0;
	}
#endif

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_nRFUartDev[devno].pReg->CONFIG |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
#ifdef NRF51
    	s_nRFUartDev[devno].CtsPin = pincfg[UARTPIN_CTS_IDX].PinNo;
    	s_nRFUartDev[devno].RtsPin = pincfg[UARTPIN_RTS_IDX].PinNo;
    	s_nRFUartDev[devno].pReg->PSELCTS = pincfg[UARTPIN_CTS_IDX].PinNo;
    	s_nRFUartDev[devno].pReg->PSELRTS = pincfg[UARTPIN_RTS_IDX].PinNo;
#else
    	s_nRFUartDev[devno].CtsPin = (pincfg[UARTPIN_CTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_CTS_IDX].PortNo << 5);
    	s_nRFUartDev[devno].RtsPin = (pincfg[UARTPIN_RTS_IDX].PinNo & 0x1f) | (pincfg[UARTPIN_RTS_IDX].PortNo << 5);
    	s_nRFUartDev[devno].pReg->PSELCTS = s_nRFUartDev[devno].CtsPin;
    	s_nRFUartDev[devno].pReg->PSELRTS = s_nRFUartDev[devno].RtsPin;
#endif
		NRF_GPIO->OUTCLR = (1 << pincfg[UARTPIN_CTS_IDX].PinNo);
		NRF_GPIO->OUTCLR = (1 << pincfg[UARTPIN_RTS_IDX].PinNo);
	}
	else
	{
		s_nRFUartDev[devno].pReg->CONFIG &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
		s_nRFUartDev[devno].pReg->PSELRTS = -1;
		s_nRFUartDev[devno].pReg->PSELCTS = -1;
		s_nRFUartDev[devno].CtsPin = -1;
		s_nRFUartDev[devno].RtsPin = -1;
	}


	s_nRFUartDev[devno].pUartDev->bRxReady = false;
	s_nRFUartDev[devno].pUartDev->bTxReady = true;
	s_nRFUartDev[devno].ErrCnt = 0;
	s_nRFUartDev[devno].RxTimeoutCnt = 0;
	s_nRFUartDev[devno].RxDropCnt = 0;
	s_nRFUartDev[devno].TxDropCnt = 0;

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


#ifdef NRF52_SERIES
	if (pDev->DevIntrf.bDma == true)
	{
		s_nRFUartDev[devno].pDmaReg->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);
		// Not using DMA transfer on Rx. It is useless on UART as we need to process 1 char at a time
		// cannot wait until DMA buffer is filled.
#if 0
		s_nRFUartDev[devno].pDmaReg->RXD.MAXCNT = NRF5X_UART_BUFF_SIZE;
		s_nRFUartDev[devno].pDmaReg->RXD.PTR = (uint32_t)s_nRFUartDev[devno].RxDmaCache;
#endif
		s_nRFUartDev[devno].pDmaReg->EVENTS_ENDRX = 0;
		s_nRFUartDev[devno].pDmaReg->TASKS_STARTRX = 1;
	}
	else
#endif
	{
		s_nRFUartDev[devno].pReg->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
		s_nRFUartDev[devno].pReg->TASKS_STARTTX = 1;
		s_nRFUartDev[devno].pReg->TASKS_STARTRX = 1;

	}

    s_nRFUartDev[devno].pReg->INTENCLR = 0xffffffffUL;

	if (pCfg->bIntMode)
	{
#ifdef NRF52_SERIES
		if (pDev->DevIntrf.bDma == true)
		{
			s_nRFUartDev[devno].pDmaReg->INTENSET = (UARTE_INTENSET_RXDRDY_Set << UARTE_INTENSET_RXDRDY_Pos) |
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
			s_nRFUartDev[devno].pReg->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
							  (UART_INTENSET_RXTO_Set << UART_INTENSET_RXTO_Pos) |
							  (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
							  (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos) |
							  (UART_INTENSET_CTS_Set << UART_INTENSET_CTS_Pos) |
							  (UART_INTENSET_NCTS_Set << UART_INTENSET_NCTS_Pos);
		}

		switch (devno)
		{
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
	return s_nRFUartDev[DevNo].pUartDev;
}

