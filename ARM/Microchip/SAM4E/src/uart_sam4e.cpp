/**-------------------------------------------------------------------------
@file	uart_sam4e.cpp

@brief	SAM4E UART implementation


@author Hoang Nguyen Hoan
@date	July. 7, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#include <assert.h>
#include <string.h>

#include "sam4e.h"
#include "component/pdc.h"

#include "interrupt.h"
#include "coredev/iopincfg.h"		
#include "coredev/uart.h"
#include "cfifo.h"

#define SAM4_UART_CFIFO_SIZE		16
#define SAM4_UART_CFIFO_MEMSIZE		CFIFO_MEMSIZE(SAM4_UART_CFIFO_SIZE)

#pragma pack(push, 4)

typedef struct _Sam_Uart_Dev {
	int DevNo;				// UART interface number
	uint32_t SamDevId;
	union {
		Sam4eUart *pUartReg;
		Sam4eUsart *pUSartReg;
	};
	Sam4ePdc *pPdc;
	UARTDEV	*pUartDev;		// Pointer to generic UART dev. data
	uint8_t RxFifoMem[SAM4_UART_CFIFO_MEMSIZE];
	uint8_t TxFifoMem[SAM4_UART_CFIFO_MEMSIZE];
	uint8_t PdcRxByte;
	uint8_t PdcTxBuff[SAM4_UART_CFIFO_SIZE];
} SAM4_UARTDEV;

#pragma pack(pop)

static SAM4_UARTDEV s_Sam4UartDev[] = {
	{
		.DevNo = 0,
		.SamDevId = ID_UART0,
		.pUartReg = SAM4E_UART0,
		.pPdc = SAM4E_PDC_UART0,
	},
	{
		.DevNo = 1,
		.SamDevId = ID_UART1,
		.pUartReg = SAM4E_UART1,
		.pPdc = SAM4E_PDC_UART1,
	},
	{
		.DevNo = 2,
		.SamDevId = ID_USART0,
		.pUSartReg = SAM4E_USART0,
		.pPdc = SAM4E_PDC_USART0,
	},
	{
		.DevNo = 3,
		.SamDevId = ID_USART1,
		.pUSartReg = SAM4E_USART1,
		.pPdc = SAM4E_PDC_USART1,
	},
};

static const int s_NbSam4UartDev = sizeof(s_Sam4UartDev) / sizeof(SAM4_UARTDEV);

UARTDEV * const UARTGetInstance(int DevNo)
{
	if (DevNo < 0 || DevNo >= s_NbSam4UartDev)
	{
		return NULL;
	}

	return s_Sam4UartDev[DevNo].pUartDev;
}

void Sam4UartIntHandler(SAM4_UARTDEV *pDev)
{
	uint32_t status = pDev->pUartReg->UART_SR;
	bool err = false;
	int cnt = 10;

	if (status & UART_SR_RXRDY)
	{
		do
		{
			uint8_t *p = CFifoPut(pDev->pUartDev->hRxFifo);
			if (p == NULL)
			{
				break;
			}

			*p = pDev->pUartReg->UART_RHR;
			status = pDev->pUartReg->UART_SR;

		} while ((status & UART_SR_RXRDY) && (cnt-- > 0));

		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, NULL, 0);
		}
	}
	cnt = 10;

	if (pDev->pUartDev->DevIntrf.bDma == true)
	{
		if (status & UART_IER_ENDTX)
		{
			int l = SAM4_UART_CFIFO_SIZE;
			uint8_t *p = CFifoGetMultiple(pDev->pUartDev->hTxFifo, &l);
			if (p)
			{
				memcpy(pDev->PdcTxBuff, p, l);
				pDev->pPdc->PERIPH_TPR = (uint32_t)pDev->PdcTxBuff;
				pDev->pPdc->PERIPH_TCR = l;
				pDev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
			}
			else
			{
				pDev->pPdc->PERIPH_TCR = 0;
				pDev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
				pDev->pUartReg->UART_IDR = UART_IER_ENDTX;
			}

			if (pDev->pUartDev->EvtCallback)
			{
				pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, 0);
			}
		}
	}
	else if (status & UART_SR_TXRDY)
	{
		do
		{
			uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);
			if (p == NULL)
			{
				pDev->pUartDev->bTxReady = true;
				pDev->pUartReg->UART_IDR = UART_IER_TXEMPTY;//UART_IER_TXRDY;
				break;
			}
			pDev->pUartReg->UART_THR = *p;
			status = pDev->pUartReg->UART_SR;
		} while ((status & UART_SR_TXRDY) && (cnt-- > 0));

		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, 0);
		}
	}

	if (status & UART_SR_OVRE)
	{
		// Overrun
		pDev->pUartDev->RxOECnt++;
		//uart_reset_status(pDev->pUartReg);
		err = true;
	}
	if (status & UART_SR_FRAME)
	{
		// Framing error
		err = true;
	}
	if ( status & UART_SR_RXBUFF )
	{
		//pdc_rx_init( g_p_uart_pdc, &g_pdc_uart_packet, NULL );
		//pdc_tx_init( g_p_uart_pdc, &g_pdc_uart_packet, NULL );
	}
	if (err)
	{
		// Reset status
		pDev->pUartReg->UART_CR = UART_CR_RSTSTA;
	}
}

void UART0_Handler(void)
{
	Sam4UartIntHandler(&s_Sam4UartDev[0]);
}

void UART1_Handler( void )
{
	Sam4UartIntHandler(&s_Sam4UartDev[1]);
}

void Sam4USartIntHandler(SAM4_UARTDEV *pDev)
{
	uint32_t status = pDev->pUSartReg->US_CSR;
	bool err = false;
	int cnt = 10;

	if (status & UART_SR_RXRDY)
	{
		do
		{
			uint8_t *p = CFifoPut(pDev->pUartDev->hRxFifo);
			if (p == NULL)
			{
				break;
			}
			*p = pDev->pUSartReg->US_RHR & US_RHR_RXCHR_Msk;
			status = pDev->pUSartReg->US_CSR;
		} while ((status & US_CSR_RXRDY) && cnt-- > 0);

		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, NULL, 0);
		}
	}
	
	if (pDev->pUartDev->DevIntrf.bDma == true)
	{
		if (status & US_IER_ENDTX)
		{
			int l = SAM4_UART_CFIFO_SIZE;
			uint8_t *p = CFifoGetMultiple(pDev->pUartDev->hTxFifo, &l);
			if (p)
			{
				memcpy(pDev->PdcTxBuff, p, l);
				pDev->pPdc->PERIPH_TPR = (uint32_t)pDev->PdcTxBuff;
				pDev->pPdc->PERIPH_TCR = l;
				pDev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
			}
			else
			{
				pDev->pPdc->PERIPH_TCR = 0;
				pDev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
				pDev->pUartReg->UART_IDR = US_IER_ENDTX;
			}

			if (pDev->pUartDev->EvtCallback)
			{
				pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, 0);
			}
		}
	}
	else if (status & US_CSR_TXRDY)
	{
		cnt = 10;

		do
		{
			uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);
			if (p == NULL)
			{
				pDev->pUartDev->bTxReady = true;
				pDev->pUSartReg->US_IDR = US_IER_TXRDY;
				break;
			}
			pDev->pUSartReg->US_THR = US_THR_TXCHR(*p);
			status = pDev->pUSartReg->US_CSR;
		} while (status & US_CSR_TXRDY && cnt-- > 0);

		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, NULL, 0);
		}
	}

	if (status & UART_SR_OVRE)
	{
		// Overrun
		err = true;
		pDev->pUartDev->RxOECnt++;
	}

	if (status & UART_SR_FRAME)
	{
		err = true;
	}
	if (err)
	{
		pDev->pUSartReg->US_CR = US_CR_RSTSTA;
	}
}

void USART0_Handler( void )
{
	Sam4USartIntHandler(&s_Sam4UartDev[2]);
}

void USART1_Handler( void )
{
	Sam4USartIntHandler(&s_Sam4UartDev[3]);
}

static inline int Sam4UARTGetRate(DEVINTRF * const pDev) {
	return ((SAM4_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static int Sam4UARTSetRate(DEVINTRF * const pDev, int Rate)
{
	SAM4_UARTDEV *dev = (SAM4_UARTDEV *)pDev->pDevData;
	uint32_t cd = 0;
	uint32_t pclk = SystemPeriphClockGet(0);

	if (dev->DevNo < 2)
	{
		// UART
		// baud = periphclk / (16 * CD)
		// CD = periphclk / (baud * 16)
		dev->pUartReg->UART_BRGR = ((pclk + (Rate << 3UL))/ (Rate << 4UL)) & 0xFFFF;
	}
	else
	{
		// USART
		uint32_t fp = (pclk << 3UL);
		Rate <<= 4UL;

		if (pclk > Rate)
		{
			dev->pUSartReg->US_MR &= ~US_MR_OVER;
			fp /= Rate;
		}
		else
		{
			dev->pUSartReg->US_MR |= US_MR_OVER;
			fp /= (Rate >> 1UL);
		}
		cd = fp >> 3UL;
		fp &= 0x7;
		dev->pUSartReg->US_BRGR = cd | (fp << US_BRGR_FP_Pos);
	}

	return Rate;
}

static inline bool Sam4UARTStartRx(DEVINTRF * const pSerDev, int DevAddr) {
	return true;
}

static int Sam4UARTRxData(DEVINTRF * const pDev, uint8_t *pBuff, int Bufflen)
{
	SAM4_UARTDEV *dev = (SAM4_UARTDEV *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen)
	{
		int l  = Bufflen;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &l);
		if (p == NULL)
		{
			break;
		}
		memcpy(pBuff, p, l);
		cnt += l;
		pBuff += l;
		Bufflen -= l;
	}
	EnableInterrupt(state);

	if (dev->pUartDev->bRxReady)
	{
		bool rdy = false;
		if (dev->DevNo < 2)
		{
			rdy = dev->pUartReg->UART_SR & UART_SR_RXRDY;
		}
		else
		{
			rdy = dev->pUSartReg->US_CSR & US_CSR_RXRDY;
		}
		
		if (rdy == true)
		{
			uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
			if (p)
			{
				if (dev->DevNo < 2)
				{
					*p = dev->pUartReg->UART_RHR;
				}
				else
				{
					*p = dev->pUSartReg->US_RHR & US_RHR_RXCHR_Msk;
				}
				dev->pUartDev->bRxReady = false;
			}
		}
	}

	return cnt;
}

static inline void Sam4UARTStopRx(DEVINTRF * const pDev) {
}

static inline bool Sam4UARTStartTx(DEVINTRF * const pDev, int DevAddr) {
	return true;
}

static int Sam4UARTTxData(DEVINTRF * const pDev, uint8_t *pData, int Datalen)
{
	SAM4_UARTDEV *dev = (SAM4_UARTDEV *)pDev->pDevData;
	int cnt = 0;
	int rtry = pDev->MaxRetry > 0 ? pDev->MaxRetry : 5;

	while (Datalen > 0 && rtry-- > 0)
	{
		uint32_t state = DisableInterrupt();

		while (Datalen > 0)
		{
			int l = Datalen;
			uint8_t *p = CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
			if (p == NULL)
			{
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
			bool rdy = false;
			
			if (dev->DevNo < 2)
			{
				rdy = dev->pUartDev->DevIntrf.bDma == true ?
					  dev->pUartReg->UART_SR & UART_SR_ENDTX :
					  dev->pUartReg->UART_SR & UART_SR_TXRDY;
			}
			else
			{
				rdy = dev->pUartDev->DevIntrf.bDma == true ?
					  dev->pUSartReg->US_CSR & US_CSR_ENDTX :
					  dev->pUSartReg->US_CSR & US_CSR_TXRDY;
			}
			if (rdy == true)
			{
				if (dev->pUartDev->DevIntrf.bDma == true)
				{
					int l = SAM4_UART_CFIFO_SIZE;
					uint8_t *p = CFifoGetMultiple(dev->pUartDev->hTxFifo, &l);
					if (p)
					{
						memcpy(dev->PdcTxBuff, p, l);
						dev->pPdc->PERIPH_TPR = (uint32_t)dev->PdcTxBuff;
						dev->pPdc->PERIPH_TCR = l;
						dev->pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
						if (dev->DevNo < 2)
						{
							dev->pUartReg->UART_IER = UART_IER_ENDTX;
						}
						else
						{
							dev->pUSartReg->US_IER = US_IER_ENDTX;
						}
					}
				}
				else
				{
					uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
					if (p)
					{
						dev->pUartDev->bTxReady = false;

						if (dev->DevNo < 2)
						{
							dev->pUartReg->UART_THR = *p;
							dev->pUartReg->UART_IER = UART_IER_TXEMPTY;//UART_IER_TXRDY;
						}
						else
						{
							dev->pUSartReg->US_THR = US_THR_TXCHR(*p);
							dev->pUSartReg->US_IER = US_IER_TXRDY;
						}
					}
				}
			}
		}
	}
	return cnt;
}

void Sam4UARTStopTx(DEVINTRF * const pDev) 
{
}

void Sam4UARTDisable(DEVINTRF * const pDev)
{
	SAM4_UARTDEV *dev = (SAM4_UARTDEV *)pDev->pDevData;
}

void Sam4UARTEnable(DEVINTRF * const pDev)
{
	SAM4_UARTDEV *dev = (SAM4_UARTDEV *)pDev->pDevData;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;
}

void Sam4UARTPowerOff(DEVINTRF * const pDev)
{
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState)
{

}

bool UARTInit(UARTDEV * const pDev, const UARTCFG *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pIOPinMap == NULL || pCfg->NbIOPins <= 0)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbSam4UartDev)
	{
		return false;
	}

	int devno = pCfg->DevNo;
	
	if (s_Sam4UartDev[devno].SamDevId < 32)
	{
		SAM4E_PMC->PMC_PCER0 |= 1 << s_Sam4UartDev[devno].SamDevId;
	}
	else
	{
		SAM4E_PMC->PMC_PCER1 |= 1 << (s_Sam4UartDev[devno].SamDevId - 32);
	}
	
	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_Sam4UartDev[devno].RxFifoMem, SAM4_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_Sam4UartDev[devno].TxFifoMem, SAM4_UART_CFIFO_MEMSIZE, 1, pCfg->bFifoBlocking);
	}

	pDev->DevIntrf.pDevData = &s_Sam4UartDev[devno];
	s_Sam4UartDev[devno].pUartDev = pDev;

	IOPINCFG *iopins = (IOPINCFG*)pCfg->pIOPinMap;
	
	if (pCfg->NbIOPins > 2 && pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		IOPinCfg((IOPINCFG*)pCfg->pIOPinMap, pCfg->NbIOPins);
	}
	else
	{
		IOPinCfg((IOPINCFG*)pCfg->pIOPinMap, 2);
	}

	pDev->Rate = Sam4UARTSetRate(&pDev->DevIntrf, pCfg->Rate);

	if (devno < 2)
	{
		// UART
		s_Sam4UartDev[devno].pUartReg->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
				| UART_CR_RXDIS | UART_CR_TXDIS;

		uint32_t mode = UART_MR_CHMODE_NORMAL;
		
		switch (pCfg->Parity)
		{
			case UART_PARITY_ODD:
				mode |= UART_MR_PAR_ODD;
				break;
			case UART_PARITY_EVEN:
				mode |= UART_MR_PAR_EVEN;
				break;
			case UART_PARITY_MARK:
				mode |= UART_MR_PAR_MARK;
				break;
			case UART_PARITY_SPACE:
				mode |= UART_MR_PAR_SPACE;
				break;
			case UART_PARITY_NONE:
			default:
				mode |= UART_MR_PAR_NO;
				break;
		}

		if (pCfg->bDMAMode == true)
		{
			s_Sam4UartDev[devno].pUartDev->DevIntrf.bDma = true;
			s_Sam4UartDev[devno].pUartReg->UART_PTCR = UART_PTCR_TXTEN;
			s_Sam4UartDev[devno].pUartReg->UART_IER = UART_IER_RXRDY;// | UART_IER_ENDTX;//UART_IER_TXBUFE;
			s_Sam4UartDev[devno].pPdc->PERIPH_TPR = (uint32_t)s_Sam4UartDev[devno].PdcTxBuff;
			s_Sam4UartDev[devno].pPdc->PERIPH_TCR = 0;
			s_Sam4UartDev[devno].pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
		}
		else
		{
			s_Sam4UartDev[devno].pUartDev->DevIntrf.bDma = false;
			s_Sam4UartDev[devno].pUartReg->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
			s_Sam4UartDev[devno].pUartReg->UART_IER = UART_IER_RXRDY;// | UART_IER_TXEMPTY;//UART_IER_TXRDY;
		}
		s_Sam4UartDev[devno].pUartReg->UART_MR = mode;
		s_Sam4UartDev[devno].pUartReg->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
		s_Sam4UartDev[devno].pUartReg->UART_CR = UART_CR_RSTSTA;
	}
	else
	{
		// USART
		s_Sam4UartDev[devno].pUSartReg->US_WPMR = US_WPMR_WPKEY_PASSWD;
		s_Sam4UartDev[devno].pUSartReg->US_MR = 0;
		s_Sam4UartDev[devno].pUSartReg->US_RTOR = 0;
		s_Sam4UartDev[devno].pUSartReg->US_TTGR = 0;
		s_Sam4UartDev[devno].pUSartReg->US_CR = US_CR_RSTTX | US_CR_TXDIS;
		s_Sam4UartDev[devno].pUSartReg->US_CR = US_CR_RSTRX | US_CR_RXDIS;
		s_Sam4UartDev[devno].pUSartReg->US_CR = US_CR_RSTRX | US_CR_RXDIS;
		s_Sam4UartDev[devno].pUSartReg->US_CR = US_CR_RTSDIS;
		s_Sam4UartDev[devno].pUSartReg->US_CR = US_CR_DTRDIS;

		uint32_t mode = US_MR_CHMODE_NORMAL;

		switch (pCfg->Parity)
		{
			case UART_PARITY_ODD:
				mode |= US_MR_PAR_ODD;
				break;
			case UART_PARITY_EVEN:
				mode |= US_MR_PAR_EVEN;
				break;
			case UART_PARITY_MARK:
				mode |= US_MR_PAR_MARK;
				break;
			case UART_PARITY_SPACE:
				mode |= US_MR_PAR_SPACE;
				break;
			case UART_PARITY_NONE:
			default:
				mode |= US_MR_PAR_NO;
				break;
		}
		
		if (pCfg->StopBits == 2)
		{
			mode |= US_MR_NBSTOP_2_BIT;
		}
		else
		{
			mode |= US_MR_NBSTOP_1_BIT;
		}

		switch (pCfg->DataBits)
		{
			case 5:
				mode |= US_MR_CHRL_5_BIT;
				break;
			case 6:
				mode |= US_MR_CHRL_6_BIT;
				break;
			case 7:
				mode |= US_MR_CHRL_7_BIT;
				break;
			default:
				mode |= US_MR_CHRL_8_BIT;
				break;
		}
		
		s_Sam4UartDev[devno].pUSartReg->US_MR = mode;

		if (pCfg->bDMAMode == true)
		{
			s_Sam4UartDev[devno].pUartDev->DevIntrf.bDma = true;
			s_Sam4UartDev[devno].pUSartReg->US_PTCR = US_PTCR_TXTEN;
			s_Sam4UartDev[devno].pPdc->PERIPH_TPR = (uint32_t)s_Sam4UartDev[devno].PdcTxBuff;
			s_Sam4UartDev[devno].pPdc->PERIPH_TCR = 0;
			s_Sam4UartDev[devno].pPdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
		}
		else
		{
			s_Sam4UartDev[devno].pUartDev->DevIntrf.bDma = false;
			s_Sam4UartDev[devno].pUSartReg->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
		}
		s_Sam4UartDev[devno].pUSartReg->US_CR = US_CR_TXEN | US_CR_RXEN;
		s_Sam4UartDev[devno].pUSartReg->US_IER = US_IER_RXRDY;
		s_Sam4UartDev[devno].pUSartReg->US_CR = US_CR_RSTSTA;
	}

	s_Sam4UartDev[devno].pUartDev->bRxReady = false;
	s_Sam4UartDev[devno].pUartDev->bTxReady = true;

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
	pDev->DevIntrf.Disable = Sam4UARTDisable;
	pDev->DevIntrf.Enable = Sam4UARTEnable;
	pDev->DevIntrf.GetRate = Sam4UARTGetRate;
	pDev->DevIntrf.SetRate = Sam4UARTSetRate;
	pDev->DevIntrf.StartRx = Sam4UARTStartRx;
	pDev->DevIntrf.RxData = Sam4UARTRxData;
	pDev->DevIntrf.StopRx = Sam4UARTStopRx;
	pDev->DevIntrf.StartTx = Sam4UARTStartTx;
	pDev->DevIntrf.TxData = Sam4UARTTxData;
	pDev->DevIntrf.StopTx = Sam4UARTStopTx;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = Sam4UARTPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	switch (devno)
	{
		case 0:
			NVIC_ClearPendingIRQ(UART0_IRQn);
			NVIC_SetPriority(UART0_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(UART0_IRQn);
			break;
		case 1:
			NVIC_ClearPendingIRQ(UART1_IRQn);
			NVIC_SetPriority(UART1_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(UART1_IRQn);
			break;
		case 2:
			NVIC_ClearPendingIRQ(USART0_IRQn);
			NVIC_SetPriority(USART0_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(USART0_IRQn);
			break;
		case 3:
			NVIC_ClearPendingIRQ(USART1_IRQn);
			NVIC_SetPriority(USART1_IRQn, pCfg->IntPrio);
			NVIC_EnableIRQ(USART1_IRQn);
			break;
	}
		
	return true;
}
