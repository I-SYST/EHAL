/**-------------------------------------------------------------------------
@file	uart_stm32l4xx.cpp

@brief	STM32L4x UART implementation

@author	Hoang Nguyen Hoan
@date	July 2, 2019

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

#include "stm32l4xx.h"

#include "istddef.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "atomic.h"

#define RCC_CCIPR_USARTSEL_PCLK			(0)
#define RCC_CCIPR_USARTSEL_SYSCLK		(1)
#define RCC_CCIPR_USARTSEL_HSI16		(2)
#define RCC_CCIPR_USARTSEL_LSE			(3)

#define RCC_CCIPR_USARTSEL(n, clk)		(clk<<((n-1) << 1))

#define STM32L4X_UART_HWFIFO_SIZE		6
#define STM32L4X_UART_RXTIMEOUT			15
#define STM32L4X_UART_BUFF_SIZE			16
#define STM32L4X_UART_CFIFO_SIZE		CFIFO_MEMSIZE(STM32L4X_UART_BUFF_SIZE)

#pragma pack(push, 4)
// Device driver data require by low level functions
typedef struct _STM32L4X_UART_Dev {
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
	uint8_t TxDmaCache[STM32L4X_UART_BUFF_SIZE];
	uint8_t RxFifoMem[STM32L4X_UART_CFIFO_SIZE];
	uint8_t TxFifoMem[STM32L4X_UART_CFIFO_SIZE];
} STM32L4X_UARTDEV;

#pragma pack(pop)

static uint32_t s_FclkFreq = SYSTEM_CORE_CLOCK;		// FCLK frequency in Hz

static STM32L4X_UARTDEV s_Stm32l4xUartDev[] = {
	{
		.DevNo = 0,
		.pReg = USART1,
	},
	{
		.DevNo = 1,
		.pReg = USART2,
	},
	{
		.DevNo = 2,
		.pReg = USART3,
	},
	{
		.DevNo = 3,
		.pReg = UART4,
	},
	{
		.DevNo = 4,
		.pReg = UART5,
	},
	{
		.DevNo = 5,
		.pReg = UART5,
	},
	{
		.DevNo = 6,
		.pReg = LPUART1,
	},
};

static const int s_NbUartDev = sizeof(s_Stm32l4xUartDev) / sizeof(STM32L4X_UARTDEV);

UARTDEV * const UARTGetInstance(int DevNo)
{
	return s_Stm32l4xUartDev[DevNo].pUartDev;
}

bool STM32L4xUARTWaitForRxReady(STM32L4X_UARTDEV * const pDev, uint32_t Timeout)
{
	return false;
}

bool STM32L4xUARTWaitForTxReady(STM32L4X_UARTDEV * const pDev, uint32_t Timeout)
{
	return false;
}

static void UART_IRQHandler(STM32L4X_UARTDEV * const pDev)
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
	UART_IRQHandler(&s_Stm32l4xUartDev[0]);
	NVIC_ClearPendingIRQ(USART2_IRQn);
}

extern "C" void USART2_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[1]);
	NVIC_ClearPendingIRQ(USART2_IRQn);
}

extern "C" void USART3_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[2]);
	NVIC_ClearPendingIRQ(USART3_IRQn);
}

extern "C" void UART4_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[3]);
	NVIC_ClearPendingIRQ(UART4_IRQn);
}

extern "C" void UART5_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[4]);
	NVIC_ClearPendingIRQ(UART5_IRQn);
}

extern "C" void LPUART1_IRQHandler()
{
	UART_IRQHandler(&s_Stm32l4xUartDev[5]);
	NVIC_ClearPendingIRQ(LPUART1_IRQn);
}

static inline int STM32L4xUARTGetRate(DEVINTRF * const pDev) {
	return ((STM32L4X_UARTDEV*)pDev->pDevData)->pUartDev->Rate;
}

static int STM32L4xUARTSetRate(DEVINTRF * const pDev, int Rate)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;
	uint32_t fclk2 = s_FclkFreq << 1;
	uint32_t rem8 = fclk2 % Rate;
	uint32_t rem16 = s_FclkFreq % Rate;

	if (rem8 < rem16)
	{
		// /8 better
		s_Stm32l4xUartDev[dev->DevNo].pReg->CR1 |= USART_CR1_OVER8;

		uint32_t div = (fclk2 + (Rate >> 1)) / Rate;
		dev->pUartDev->Rate = (fclk2 + (div >> 1)) / div;
		s_Stm32l4xUartDev[dev->DevNo].pReg->BRR = ((div & 0xf) >> 1) | (div & 0xFFFFFFF0);
	}
	else
	{
		// /16 better
		s_Stm32l4xUartDev[dev->DevNo].pReg->CR1 &= ~USART_CR1_OVER8;

		uint32_t div = (s_FclkFreq + (Rate >> 1)) / Rate;
		dev->pUartDev->Rate = (s_FclkFreq + (div >> 1))/ div;
		s_Stm32l4xUartDev[dev->DevNo].pReg->BRR = div;
	}

	return dev->pUartDev->Rate;
}

static inline bool STM32L4xUARTStartRx(DEVINTRF * const pSerDev, int DevAddr) {
	return true;
}

static int STM32L4xUARTRxData(DEVINTRF * const pDev, uint8_t *pBuff, int Bufflen)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;
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

static inline void STM32L4xUARTStopRx(DEVINTRF * const pDev) {
}

static inline bool STM32L4xUARTStartTx(DEVINTRF * const pDev, int DevAddr) {
	return true;
}

static int STM32L4xUARTTxData(DEVINTRF * const pDev, uint8_t *pData, int Datalen)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;
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

static inline void STM32L4xUARTStopTx(DEVINTRF * const pDev) {
}

static void STM32L4xUARTDisable(DEVINTRF * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	dev->pReg->CR1 &= ~(USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
	dev->pReg->CR2 &= ~USART_CR2_RTOEN;

	if (dev->DevNo == 0)
	{
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	}
	else if (dev->DevNo == 5)
	{
		RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
	}
	else if (dev->DevNo > 0)
	{
		RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USART2EN << (dev->DevNo - 1));
	}
}

static void STM32L4xUARTEnable(DEVINTRF * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	dev->ErrCnt = 0;
	dev->RxTimeoutCnt = 0;
	dev->RxDropCnt = 0;
	dev->TxDropCnt = 0;
	pDev->bBusy = false;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->pUartDev->bTxReady = true;

	if (dev->DevNo == 0)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}
	else if (dev->DevNo == 5)
	{
		RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	}
	else if (dev->DevNo > 0)
	{
		RCC->APB1ENR1 |= (RCC_APB1ENR1_USART2EN << (dev->DevNo - 1));
	}
	dev->pReg->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
	dev->pReg->CR2 |= USART_CR2_RTOEN;

}

static void STM32L4xUARTPowerOff(DEVINTRF * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	STM32L4xUARTDisable(pDev);

	switch (dev->DevNo)
	{
		case 0:
			RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			break;
		case 1:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			break;
		case 2:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART3EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			break;
		case 3:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART4EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			break;
		case 4:
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART5EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			break;
		case 5:
			RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPUART1EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			break;
	}

	dev->pReg->CR2 &= ~USART_CR2_CLKEN;
}

static void STM32L4xUARTReset(DEVINTRF * const pDev)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->pDevData;

	if (dev->DevNo == 0)
	{
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		usDelay(100);
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
	}
	else if (dev->DevNo == 5)
	{
		RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST;
		usDelay(100);
		RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;
	}
	else if (dev->DevNo > 0)
	{
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST << (dev->DevNo - 1);
		usDelay(100);
		RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_USART2RST << (dev->DevNo - 1));
	}
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
	USART_TypeDef *reg = s_Stm32l4xUartDev[devno].pReg;

	pDev->DevIntrf.pDevData = &s_Stm32l4xUartDev[devno];
	s_Stm32l4xUartDev[devno].pUartDev = pDev;

	STM32L4xUARTReset(&pDev->DevIntrf);

	// Disable UART first because some field can't be set is it is already enabled
	reg->CR1 &= ~USART_CR1_UE;

	// Enable clock
	switch (devno)
	{
		case 0:
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			RCC->CCIPR |= RCC_CCIPR_USARTSEL(1, RCC_CCIPR_USARTSEL_SYSCLK);
			break;
		case 1:
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			RCC->CCIPR |= RCC_CCIPR_USARTSEL(2, RCC_CCIPR_USARTSEL_SYSCLK);
			break;
		case 2:
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			RCC->CCIPR |= RCC_CCIPR_USARTSEL(3, RCC_CCIPR_USARTSEL_SYSCLK);
			break;
		case 3:
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			RCC->CCIPR |= RCC_CCIPR_USARTSEL(4, RCC_CCIPR_USARTSEL_SYSCLK);
			break;
		case 4:
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			RCC->CCIPR |= RCC_CCIPR_USARTSEL(5, RCC_CCIPR_USARTSEL_SYSCLK);
			break;
		case 5:
			RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
			RCC->CCIPR &= ~RCC_CCIPR_USART1SEL_Msk;
			RCC->CCIPR |= RCC_CCIPR_USARTSEL(6, RCC_CCIPR_USARTSEL_SYSCLK);
			break;
	}

	reg->CR2 |= USART_CR2_CLKEN;

	msDelay(1);


	s_FclkFreq = SYSTEM_CORE_CLOCK;

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_Stm32l4xUartDev[devno].RxFifoMem, STM32L4X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_Stm32l4xUartDev[devno].TxFifoMem, STM32L4X_UART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	// Configure UART pins
	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIoMap;

	IOPinCfg(pincfg, pCfg->IoMapLen);

    // Set baud
    pDev->Rate = STM32L4xUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

	switch (pCfg->Parity)
	{
		case UART_PARITY_NONE:
			reg->CR1 &= ~USART_CR1_PCE;
			break;
		case UART_PARITY_EVEN:
			reg->CR1 |= USART_CR1_PS | USART_CR1_PCE;
			break;
		case UART_PARITY_ODD:
			reg->CR1 &= ~USART_CR1_PS;
			reg->CR1 |= USART_CR1_PCE;
			break;
	}

	reg->CR1 &= ~(USART_CR1_M | (1 << 28));

	if (pCfg->DataBits == 9)
	{
		reg->CR1 |=  USART_CR1_M;
	}
	else if (pCfg->DataBits == 7)
	{
		reg->CR1 |=  (1 << 28);
	}

	reg->CR2 &= ~USART_CR2_STOP_Msk;
	if (pCfg->StopBits == 2)
	{
		reg->CR2 |= 2;
	}

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	reg->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	}
	else
	{
    	reg->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	}

    s_Stm32l4xUartDev[devno].pUartDev->bRxReady = false;
    s_Stm32l4xUartDev[devno].pUartDev->bTxReady = true;
    s_Stm32l4xUartDev[devno].ErrCnt = 0;
    s_Stm32l4xUartDev[devno].RxTimeoutCnt = 0;
    s_Stm32l4xUartDev[devno].RxDropCnt = 0;
    s_Stm32l4xUartDev[devno].TxDropCnt = 0;

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->Mode = pCfg->Mode;
	pDev->Duplex = pCfg->Duplex;
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
	pDev->DevIntrf.Reset = STM32L4xUARTReset;
	pDev->DevIntrf.Disable = STM32L4xUARTDisable;
	pDev->DevIntrf.Enable = STM32L4xUARTEnable;
	pDev->DevIntrf.GetRate = STM32L4xUARTGetRate;
	pDev->DevIntrf.SetRate = STM32L4xUARTSetRate;
	pDev->DevIntrf.StartRx = STM32L4xUARTStartRx;
	pDev->DevIntrf.RxData = STM32L4xUARTRxData;
	pDev->DevIntrf.StopRx = STM32L4xUARTStopRx;
	pDev->DevIntrf.StartTx = STM32L4xUARTStartTx;
	pDev->DevIntrf.TxData = STM32L4xUARTTxData;
	pDev->DevIntrf.StopTx = STM32L4xUARTStopTx;
	pDev->DevIntrf.bBusy = false;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;
	pDev->DevIntrf.PowerOff = STM32L4xUARTPowerOff;
	pDev->DevIntrf.EnCnt = 1;


	if (pDev->DevIntrf.bDma == true)
	{
/*		if (devno == 0)
		{
			SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART1TX_DMA_RMP;
		}
		if (devno == 2)
		{
			// USART3_DMA_RMP
			SYSCFG->CFGR1 |= SYSCFG_CFGR1_USART3_DMA_RMP;
		}*/
		// Not using DMA transfer on Rx. It is useless on UART as we need to process 1 char at a time
		// cannot wait until DMA buffer is filled.
		reg->CR3 |= USART_CR3_DMAT;
	}
	else
	{
/*		if (devno == 0)
		{
			SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART1TX_DMA_RMP | SYSCFG_CFGR1_USART1RX_DMA_RMP);
		}
		if (devno == 2)
		{
			SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART3_DMA_RMP);
		}
*/
		reg->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
	}

	// Select duplex mode
	reg->CR3 &= ~USART_CR3_HDSEL;
	if (pDev->Duplex == UART_DUPLEX_HALF)
	{
		reg->CR3 |= USART_CR3_HDSEL;
	}

	uint32_t tmp = reg->CR1;

	// Disable all interrupts
	tmp &= ~(USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_RTOIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE);

	if (pCfg->bIntMode)
	{
		tmp |= (USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_RTOIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE);

		if (pCfg->FlowControl == UART_FLWCTRL_HW)
	    {
			reg->CR3 |= USART_CR3_CTSIE;
	    }
		reg->CR3 |= USART_CR3_EIE;

		switch (devno)
		{
			case 0:
				NVIC_ClearPendingIRQ(USART1_IRQn);
				NVIC_SetPriority(USART1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART1_IRQn);
				break;
			case 1:
				NVIC_ClearPendingIRQ(USART2_IRQn);
				NVIC_SetPriority(USART2_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART2_IRQn);
				break;
			case 2:
				NVIC_ClearPendingIRQ(USART3_IRQn);
				NVIC_SetPriority(USART3_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(USART3_IRQn);
				break;

			case 3:
				NVIC_ClearPendingIRQ(UART4_IRQn);
				NVIC_SetPriority(UART4_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UART4_IRQn);
				break;
			case 4:
				NVIC_ClearPendingIRQ(UART5_IRQn);
				NVIC_SetPriority(UART5_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(UART5_IRQn);
				break;

			case 5:
				NVIC_ClearPendingIRQ(LPUART1_IRQn);
				NVIC_SetPriority(LPUART1_IRQn, pCfg->IntPrio);
				NVIC_EnableIRQ(LPUART1_IRQn);
				break;
		}
    }

	// Enable USART
	tmp |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

	reg->CR1 = tmp;
	reg->CR2 |= USART_CR2_RTOEN;

	return true;
}

void UARTSetCtrlLineState(UARTDEV * const pDev, uint32_t LineState)
{
	STM32L4X_UARTDEV *dev = (STM32L4X_UARTDEV *)pDev->DevIntrf.pDevData;

}

