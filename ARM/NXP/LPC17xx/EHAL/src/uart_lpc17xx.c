/*--------------------------------------------------------------------------
File   : lpc17xx_uart.c

Author : Hoang Nguyen Hoan          Jan. 16, 2014

Desc   : LPC17xx UART implementation

Copyright (c) 2014, I-SYST inc., all rights reserved

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
#include "LPC17xx.h"
#include "uart_lpcxx.h"
#include "system_LPC17xx.h"

#define LPC_PCONP_UART0				(1 << 3)
#define LPC_PCONP_UART1				(1 << 4)
#define LPC_PCONP_UART2				(1 << 24)
#define LPC_PCONP_UART3				(1 << 25)

#define LPC_PCLKSEL0_UART0_MASK 	(3 << 6)	// On PCLKSEL0
#define LPC_PCLKSEL0_UART0_DIV1		(1 << 6)
#define LPC_PCLKSEL0_UART0_DIV2		(2 << 6)
#define LPC_PCLKSEL0_UART0_DIV4		(0)
#define LPC_PCLKSEL0_UART0_DIV8		(3 << 6)

#define LPC_PCLKSEL0_UART1_MASK		(3 << 8)	// On PCLKSEL0
#define LPC_PCLKSEL0_UART1_DIV1		(1 << 8)
#define LPC_PCLKSEL0_UART1_DIV2		(2 << 8)
#define LPC_PCLKSEL0_UART1_DIV4		(0)
#define LPC_PCLKSEL0_UART1_DIV8		(3 << 8)

#define LPC_PCLKSEL1_UART2_MASK		(3 << 16)	// On PCLKSEL1
#define LPC_PCLKSEL1_UART2_DIV1		(1 << 16)
#define LPC_PCLKSEL1_UART2_DIV2		(2 << 16)
#define LPC_PCLKSEL1_UART2_DIV4		(0)
#define LPC_PCLKSEL1_UART2_DIV8		(3 << 16)

#define LPC_PCLKSEL1_UART3_MASK		(3 << 18)	// On PCLKSEL1
#define LPC_PCLKSEL1_UART3_DIV1		(1 << 18)
#define LPC_PCLKSEL1_UART3_DIV2		(2 << 18)
#define LPC_PCLKSEL1_UART3_DIV4		(0)
#define LPC_PCLKSEL1_UART3_DIV8		(3 << 18)

#define LPC17XX_UART_MAX_DEV		4

#define UART_RX_CFIFO_SIZE			16
#define UART_TX_CFIFO_SIZE			16

#define UART_RX_CFIFO_MEM_SIZE			CFIFO_MEMSIZE(UART_RX_CFIFO_SIZE)
#define UART_TX_CFIFO_MEM_SIZE			CFIFO_MEMSIZE(UART_TX_CFIFO_SIZE)

uint32_t s_ErrCnt = 0;

uint8_t s_UARTRxFifoMem[UART_RX_CFIFO_MEM_SIZE];
uint8_t s_UARTTxFifoMem[UART_TX_CFIFO_MEM_SIZE];

LPCUARTDEV g_LpcUartDev[LPC17XX_UART_MAX_DEV] = {
	{0, (LPCUARTREG*)LPC_UART0, },
	{1, (LPCUARTREG*)LPC_UART1, },
	{2, (LPCUARTREG*)LPC_UART2, },
	{3, (LPCUARTREG*)LPC_UART3, }
};

bool LpcUARTInit(UARTDEV *pDev, const UARTCFG *pCfg)
{
	LPCUARTREG *reg = NULL;
	int irq = 0;

	if (pCfg == NULL)
		return false;

	if (pCfg->pIoMap == NULL || pCfg->IoMapLen == 0)
		return false;

	switch (pCfg->DevNo)
	{
		case 0:
	        LPC_SC->PCONP |= LPC_PCONP_UART0;
	        reg = (LPCUARTREG*)LPC_UART0;
	        LPC_SC->PCLKSEL0 &= ~LPC_PCLKSEL0_UART0_MASK;	// CCLK/4
	        irq = UART0_IRQn;
			break;
		case 1:
			LPC_SC->PCONP |= LPC_PCONP_UART1;
			reg = (LPCUARTREG*)LPC_UART1;
	        LPC_SC->PCLKSEL0 &= ~LPC_PCLKSEL0_UART1_MASK;	// CCLK/4
	        irq = UART2_IRQn;
			break;
		case 2:
			LPC_SC->PCONP |= LPC_PCONP_UART2;
			reg = (LPCUARTREG*)LPC_UART2;
	        LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_UART2_MASK;	// CCLK/4
	        irq = UART2_IRQn;
			break;
		case 3:
			LPC_SC->PCONP |= LPC_PCONP_UART3;
			reg = (LPCUARTREG*)LPC_UART3;
	        LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_UART3_MASK;	// CCLK/4
	        irq = UART3_IRQn;
			break;
		default:
			return false;
	}

	// Configure I/O pins
	int idx = 0;
	IOPINCFG *pincfg = (IOPINCFG *)pCfg->pIoMap;

	IOPinCfg(pincfg, pCfg->IoMapLen);

	reg->TER = 0;	// Disable Tx
	reg->IER = 0;	// Disable all interrupts
	reg->ACR = 0;	// Disable auto baudrate

	// Clear all FIFO
	reg->FCR = LPCUART_FCR_RST_RXFIFO | LPCUART_FCR_RST_TXFIFO;

	if (pCfg->bDMAMode)
		reg->FCR |= LPCUART_FCR_DMA_MODE | LPCUART_FCR_RX_TRIG8;

	reg->LCR = (pCfg->DataBits - 5);
	if (pCfg->Parity != UART_PARITY_NONE)
	{
		reg->LCR |= (pCfg->Parity << 4);
	}

	if (pCfg->StopBits > 1)
		reg->LCR |= LPCUART_LCR_STOPBIT_MASK;

	reg->ICR = 0;
	if (pCfg->bIrDAMode)
	{
		if (pCfg->bIrDAInvert)
			reg->ICR |= LPCUART_ICR_IRDAINV;
		if (pCfg->bIrDAFixPulse)
			reg->ICR |= LPCUART_ICR_IRDA_FIXPULSE | ((pCfg->IrDAPulseDiv & 7) << 3);
		reg->ICR |= LPCUART_ICR_IRDAEN;
	}

	if (pCfg->Rate)
		pDev->Rate = LpcUARTSetRate(&pDev->DevIntrf, pCfg->Rate);
	else
	{
		// Auto baudrate
		reg->ACR = 7;
	}

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
//		LPC_GPIO->CLR[pCfg->PinCfg[UARTPIN_CTS_IDX].PortNo] = (1 << pCfg->PinCfg[UARTPIN_CTS_IDX].PinNo);
//		LPC_GPIO->CLR[pCfg->PinCfg[UARTPIN_RTS_IDX].PortNo] = (1 << pCfg->PinCfg[UARTPIN_RTS_IDX].PinNo);
		reg->MCR |= (3 << 6);	// Auto CTS/RTS flow control

	}
	else
	{
		reg->MCR &= ~(3 << 6);
	}

	reg->FCR = LPCUART_FCR_FIFOEN | LPCUART_FCR_RST_RXFIFO | LPCUART_FCR_RST_TXFIFO |
			   LPCUART_FCR_RX_TRIG8;

	uint32_t val = 0;

	while (reg->LSR & ~(3<<5))
	{
		val = reg->RBR;
	}

	val = reg->IIR;	// Clear interrupts
	pDev->LineState = 0;

	//LPC_USART->MCR |= (1<<4); // Loopback

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_UARTRxFifoMem, UART_RX_CFIFO_MEM_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_UARTTxFifoMem, UART_TX_CFIFO_MEM_SIZE, 1, pCfg->bFifoBlocking);
	}

	// Start tx
	reg->TER = LPCUART_TER_TXEN;


	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->DevIntrf.Disable = LpcUARTDisable;
	pDev->DevIntrf.Enable = LpcUARTEnable;
	pDev->DevIntrf.GetRate = LpcUARTGetRate;
	pDev->DevIntrf.SetRate = LpcUARTSetRate;
	pDev->DevIntrf.StartRx = LpcUARTStartRx;
	pDev->DevIntrf.RxData = LpcUARTRxData;
	pDev->DevIntrf.StopRx = LpcUARTStopRx;
	pDev->DevIntrf.StartTx = LpcUARTStartTx;
	pDev->DevIntrf.TxData = LpcUARTTxData;
	pDev->DevIntrf.StopTx = LpcUARTStopTx;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->DevIntrf.bBusy = false;

	g_LpcUartDev[pCfg->DevNo].bTxReady = true;

	pDev->LineState = 0;

	if (pCfg->bIntMode)
	{
		reg->IER = LPCUART_IER_THRE | LPCUART_IER_RBR | LPCUART_IER_RLS | LPCUART_IER_MS | (1<<7);
		NVIC_ClearPendingIRQ(irq);
		NVIC_SetPriority(irq, pCfg->IntPrio);
		NVIC_EnableIRQ(irq);
	}


	return true;
}





