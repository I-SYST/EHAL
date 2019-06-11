/**-------------------------------------------------------------------------
@file	uart_stm32f0x.cpp

@brief	STM32F0x UART implementation

@author	Hoang Nguyen Hoan
@date	June 7, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

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
//#include <atomic>

#include "stm32f0xx.h"

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "atomic.h"

#define SYSCFG_CFGR1_USART3_DMA_RMP		(1<<26)

#define STM32F0X_UART_HWFIFO_SIZE		6
#define STM32F0X_UART_RXTIMEOUT			15
#define STM32F0X_UART_BUFF_SIZE			16
#define STM32F0X_UART_CFIFO_SIZE		CFIFO_MEMSIZE(STM32F0X_UART_BUFF_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct _STM32F0X_UART_Dev {
	int DevNo;				// UART interface number
	USART_TypeDef *pReg;	// UART registers
	UARTDEV	*pUartDev;		// Pointer to generic UART dev. data
	uint32_t RxDropCnt;
	uint32_t RxTimeoutCnt;
	uint32_t TxDropCnt;
	uint32_t ErrCnt;
	uint32_t RxPin;
	uint32_t TxPin;
	uint32_t CtsPin;
	uint32_t RtsPin;
	uint8_t TxDmaCache[STM32F0X_UART_BUFF_SIZE];
	uint8_t RxFifoMem[STM32F0X_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[STM32F0X_UART_CFIFO_SIZE];
} STM32F0X_UARTDEV;

#pragma pack(pop)

static uint32_t s_FclkFreq = SYSTEM_CORE_CLOCK;		// FCLK frequency in Hz

static STM32F0X_UARTDEV s_Stm32f03xUartDev[] = {
	{
		.DevNo = 0,
		.pReg = USART1,
	},
#if !defined(STM32F030x4) && !defined(STM32F030x6)
	{
		.DevNo = 1,
		.pReg = USART2,
	},
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
	{
		.DevNo = 2,
		.pReg = USART3,
	},
	{
		.DevNo = 3,
		.pReg = USART4,
	},
#endif
#ifdef STM32F030xC
	{
		.DevNo = 4,
		.pReg = USART5,
	},
	{
		.DevNo = 5,
		.pReg = USART6,
	},
#endif
};

static const int s_NbUartDev = sizeof(s_Stm32f03xUartDev) / sizeof(STM32F0X_UARTDEV);

UARTDEV * const UARTGetInstance(int DevNo)
{
	return s_Stm32f03xUartDev[DevNo].pUartDev;
}


bool STM32F03xUARTWaitForRxReady(STM32F0X_UARTDEV * const pDev, uint32_t Timeout)
{
	return false;
}

bool STM32F03xUARTWaitForTxReady(STM32F0X_UARTDEV * const pDev, uint32_t Timeout)
{
	return false;
}

static void UART_IRQHandler(STM32F0X_UARTDEV * const pDev)
{
	UARTDEV *dev = (UARTDEV *)pDev->pUartDev;
	int len = 0;
	int cnt = 0;
	uint32_t iflag = pDev->pReg->ISR;

	if ((iflag & (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE)) == 0)
	{
		// no error
		if (iflag & USART_ISR_RXNE)
		{
			uint8_t *p = CFifoPut(dev->hRxFifo);
			if (p != NULL)
			{
				*p = pDev->pReg->RDR;
				dev->bRxReady = true;
				pDev->pReg->ICR = USART_ISR_RXNE;
			}
			if (dev->EvtCallback)
			{
				dev->EvtCallback(dev, UART_EVT_RXDATA, NULL, CFifoUsed(dev->hRxFifo));
			}
		}
		if (iflag & (USART_ISR_TXE | USART_ISR_TC))
		{
			uint8_t *p = CFifoGet(dev->hTxFifo);
			if (p != NULL)
			{
				pDev->pReg->TDR = *p;
				dev->bTxReady = false;
				pDev->pReg->ICR = USART_ISR_TXE;
			}
			else
			{
				dev->bTxReady = true;
			}
			if (dev->EvtCallback)
			{
				dev->EvtCallback(dev, UART_EVT_TXREADY, NULL, CFifoUsed(dev->hRxFifo));
			}
		}
		if (iflag & USART_ISR_RTOF)
		{
			if (dev->EvtCallback)
			{
				dev->EvtCallback(dev, UART_EVT_RXTIMEOUT, NULL, CFifoUsed(dev->hRxFifo));
			}
		}
	}
	else
	{
		// error
		pDev->ErrCnt++;
		//pDev->pReg->ICR = (USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE);
	}
	pDev->pReg->ICR = iflag;
}

extern "C" void USART1_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[0]);
	NVIC_ClearPendingIRQ(USART2_IRQn);
}

#if !defined(STM32F030x4) && !defined(STM32F030x6)
extern "C" void USART2_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[1]);
	NVIC_ClearPendingIRQ(USART2_IRQn);
}
#endif

#if defined(STM32F070xB) || defined(STM32F030xC)
extern "C" void UART3_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[2]);
	NVIC_ClearPendingIRQ(USART3_IRQn);
}

extern "C" void UART4_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[3]);
	NVIC_ClearPendingIRQ(USART4_IRQn);
}
#endif

#ifdef STM32F030xC
extern "C" void UART5_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[4]);
	NVIC_ClearPendingIRQ(USART5_IRQn);
}

extern "C" void UART6_IRQHandler()
{
	UART_IRQHandler(&s_Stm32f03xUartDev[5]);
	NVIC_ClearPendingIRQ(USART6_IRQn);
}
#endif

static inline int STM32F03xUARTGetRate(DEVINTRF * const pDev) {
	return ((STM32F0X_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static int STM32F03xUARTSetRate(DEVINTRF * const pDev, int Rate)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
	uint32_t fclk2 = s_FclkFreq << 1;
	uint32_t rem8 = fclk2 % Rate;
	uint32_t rem16 = s_FclkFreq % Rate;

	if (rem8 < rem16)
	{
		// /8 better
		s_Stm32f03xUartDev[dev->DevNo].pReg->CR1 |= USART_CR1_OVER8;

		uint32_t div = (fclk2 + (Rate >> 1)) / Rate;
		dev->pUartDev->Rate = (fclk2 + (div >> 1)) / div;
		s_Stm32f03xUartDev[dev->DevNo].pReg->BRR = ((div & 0xf) >> 1) | (div & 0xFFFFFFF0);
	}
	else
	{
		// /16 better
		s_Stm32f03xUartDev[dev->DevNo].pReg->CR1 &= ~USART_CR1_OVER8;

		uint32_t div = (s_FclkFreq + (Rate >> 1)) / Rate;
		dev->pUartDev->Rate = (s_FclkFreq + (div >> 1))/ div;
		s_Stm32f03xUartDev[dev->DevNo].pReg->BRR = div;
	}

	return dev->pUartDev->Rate;
}

static inline bool STM32F03xUARTStartRx(DEVINTRF * const pSerDev, int DevAddr) {
	return true;
}

static int STM32F03xUARTRxData(DEVINTRF * const pDev, uint8_t *pBuff, int Bufflen)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
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
			dev->pUartDev->bRxReady = false;
			*p = dev->pReg->RDR;
		}
	}

	return cnt;
}

static inline void STM32F03xUARTStopRx(DEVINTRF * const pDev) {
}

static inline bool STM32F03xUARTStartTx(DEVINTRF * const pDev, int DevAddr) {
	return true;
}

static int STM32F03xUARTTxData(DEVINTRF * const pDev, uint8_t *pData, int Datalen)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;
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
        	uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
        	if (p != NULL)
        	{
        		dev->pUartDev->bTxReady = false;
        		dev->pReg->TDR = *p;
        	}
        }
    }
    return cnt;
}

static inline void STM32F03xUARTStopTx(DEVINTRF * const pDev) {
}

static void STM32F03xUARTDisable(DEVINTRF * const pDev)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;

	if (AtomicDec(&pDev->EnCnt) > 0)
		return;

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
	dev->pReg->CR2 &= ~USART_CR2_RTOEN;

	RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;
}

static void STM32F03xUARTEnable(DEVINTRF * const pDev)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;

	if (AtomicInc(&pDev->EnCnt) > 1)
		return;

	dev->ErrCnt = 0;
	dev->RxTimeoutCnt = 0;
	dev->RxDropCnt = 0;
	dev->TxDropCnt = 0;
	pDev->bBusy = false;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;

	RCC->CFGR3 |= RCC_CFGR3_USART1SW_SYSCLK;
	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	dev->pReg->CR2 |= USART_CR2_RTOEN;

}

void STM32F03xUARTPowerOff(DEVINTRF * const pDev)
{
//	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->pDevData;

	STM32F03xUARTDisable(pDev);
}

bool UARTInit(UARTDEV * const pDev, const UARTCFG *pCfg)
{
	// Config I/O pins
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	if (pCfg->pIoMap == NULL || pCfg->IoMapLen <= 0)
	{
		return false;
	}

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev)
	{
		return false;
	}

	int devno = pCfg->DevNo;

	// Enable clock
	switch (devno)
	{
		case 0:
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			break;
#if !defined(STM32F030x4) && !defined(STM32F030x6)
		case 1:
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
			break;
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
		case 2:
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			break;
		case 3:
			RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
			break;
#endif
#ifdef STM32F030xC
		case 4:
			RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
			break;
		case 5:
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
			break;
#endif
	}

	msDelay(1);

	RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;
	RCC->CFGR3 |= RCC_CFGR3_USART1SW_SYSCLK;

	s_FclkFreq = SYSTEM_CORE_CLOCK;
	//s_Stm32f03xUartDev[devno].pReg->CR1 &= ~USART_CR1_OVER8;	// /16

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_Stm32f03xUartDev[devno].RxFifoMem, STM32F0X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_Stm32f03xUartDev[devno].TxFifoMem, STM32F0X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	// Configure UART pins
	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIoMap;

	IOPinCfg(pincfg, pCfg->IoMapLen);

	pDev->DevIntrf.pDevData = &s_Stm32f03xUartDev[devno];
	s_Stm32f03xUartDev[devno].pUartDev = pDev;

    // Set baud
    pDev->Rate = STM32F03xUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

	switch (pCfg->Parity)
	{
		case UART_PARITY_NONE:
			s_Stm32f03xUartDev[devno].pReg->CR1 &= ~USART_CR1_PCE;
			break;
		case UART_PARITY_EVEN:
			s_Stm32f03xUartDev[devno].pReg->CR1 |= USART_CR1_PS | USART_CR1_PCE;
			break;
		case UART_PARITY_ODD:
			s_Stm32f03xUartDev[devno].pReg->CR1 &= ~USART_CR1_PS;
			s_Stm32f03xUartDev[devno].pReg->CR1 |= USART_CR1_PCE;
			break;
	}

	s_Stm32f03xUartDev[devno].pReg->CR1 &= ~(USART_CR1_M | (1 << 28));

	if (pCfg->DataBits == 9)
	{
		s_Stm32f03xUartDev[devno].pReg->CR1 |=  USART_CR1_M;
	}
	else if (pCfg->DataBits == 7)
	{
		s_Stm32f03xUartDev[devno].pReg->CR1 |=  (1 << 28);
	}

	s_Stm32f03xUartDev[devno].pReg->CR2 &= ~USART_CR2_STOP_Msk;
	if (pCfg->StopBits == 2)
	{
		s_Stm32f03xUartDev[devno].pReg->CR2 |= 2;
	}

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_Stm32f03xUartDev[devno].pReg->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	}
	else
	{
    	s_Stm32f03xUartDev[devno].pReg->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	}


    s_Stm32f03xUartDev[devno].pUartDev->bRxReady = false;
    s_Stm32f03xUartDev[devno].pUartDev->bTxReady = true;
    s_Stm32f03xUartDev[devno].ErrCnt = 0;
    s_Stm32f03xUartDev[devno].RxTimeoutCnt = 0;
    s_Stm32f03xUartDev[devno].RxDropCnt = 0;
    s_Stm32f03xUartDev[devno].TxDropCnt = 0;

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
	pDev->DevIntrf.Disable = STM32F03xUARTDisable;
	pDev->DevIntrf.Enable = STM32F03xUARTEnable;
	pDev->DevIntrf.GetRate = STM32F03xUARTGetRate;
	pDev->DevIntrf.SetRate = STM32F03xUARTSetRate;
	pDev->DevIntrf.StartRx = STM32F03xUARTStartRx;
	pDev->DevIntrf.RxData = STM32F03xUARTRxData;
	pDev->DevIntrf.StopRx = STM32F03xUARTStopRx;
	pDev->DevIntrf.StartTx = STM32F03xUARTStartTx;
	pDev->DevIntrf.TxData = STM32F03xUARTTxData;
	pDev->DevIntrf.StopTx = STM32F03xUARTStopTx;
	pDev->DevIntrf.bBusy = false;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = STM32F03xUARTPowerOff;
	pDev->DevIntrf.EnCnt = 1;


	if (pDev->DevIntrf.bDma == true)
	{
#if defined(STM32F030x4) || defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F070x6) || defined(STM32F070xB)
		if (devno == 0)
		{
			SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
		}
#endif
#if defined(STM32F030xB)
		if (devno == 2)
		{
			// USART3_DMA_RMP
			SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART3_DMA_RMP;
		}
#endif
		// Not using DMA transfer on Rx. It is useless on UART as we need to process 1 char at a time
		// cannot wait until DMA buffer is filled.
	}
	else
	{
		if (devno == 0)
		{
			SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_USART1RX_DMA_RMP);
		}
		if (devno == 2)
		{
			SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART3_DMA_RMP);
		}
	}

	// Select full duplex mode
	s_Stm32f03xUartDev[devno].pReg->CR3 &= ~USART_CR3_HDSEL;

	uint32_t tmp = s_Stm32f03xUartDev[devno].pReg->CR1;

	// Disable all interrupts
	tmp &= ~(USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_RTOIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE);

	if (pCfg->bIntMode)
	{
		tmp |= (USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_RTOIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE);

		if (pCfg->FlowControl == UART_FLWCTRL_HW)
	    {
			s_Stm32f03xUartDev[devno].pReg->CR3 |= USART_CR3_CTSIE;
	    }
		s_Stm32f03xUartDev[devno].pReg->CR3 |= USART_CR3_EIE;

		switch (devno)
		{
			case 0:
				NVIC_ClearPendingIRQ(USART1_IRQn);
				NVIC_SetPriority(USART1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART1_IRQn);
				break;
#if !defined(STM32F030x4) && !defined(STM32F030x6)
			case 1:
				NVIC_ClearPendingIRQ(USART2_IRQn);
				NVIC_SetPriority(USART2_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART2_IRQn);
				break;
#endif
#if defined(STM32F070xB) || defined(STM32F030xC)
			case 2:
				NVIC_ClearPendingIRQ(USART3_IRQn);
				NVIC_SetPriority(USART3_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART3_IRQn);
				break;

			case 3:
				NVIC_ClearPendingIRQ(USART4_IRQn);
				NVIC_SetPriority(USART4_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART4_IRQn);
				break;
#endif
#ifdef STM32F030xC
			case 4:
				NVIC_ClearPendingIRQ(USART5_IRQn);
				NVIC_SetPriority(USART5_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART5_IRQn);
				break;

			case 5:
				NVIC_ClearPendingIRQ(USART6_IRQn);
				NVIC_SetPriority(USART6_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART6_IRQn);
				break;
#endif
		}
    }

	// Enable USART
	tmp |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	s_Stm32f03xUartDev[devno].pReg->CR1 = tmp;
	s_Stm32f03xUartDev[devno].pReg->CR2 |= USART_CR2_RTOEN;

	return true;
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState)
{
	STM32F0X_UARTDEV *dev = (STM32F0X_UARTDEV *)pDev->DevIntrf.pDevData;

}


