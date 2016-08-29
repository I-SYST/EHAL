/*--------------------------------------------------------------------------
File   : lpc11uxx_uart.h

Author : Hoang Nguyen Hoan          Oct. 26, 2014

Desc   : LPC11Uxx UART implementation

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

#include "LPC11Uxx.h"
#include "uart_lpcxx.h"
#include "idelay.h"
#include "atomic.h"

#define LPC_SYSAHBCLKCTRL_UART0_EN		(1 << 12)
#define LPC_SYSAHBCLKCTRL_UART1_EN		(1 << 20)
#define LPC_SYSAHBCLKCTRL_UART2_EN		(1 << 21)

#define LPC11XX_UART_MAX_DEV		1

//extern int g_UartClkDiv;
extern uint32_t SystemCoreClock;
extern uint32_t SystemMainClkFreq;

LPCUARTDEV g_LpcUartDev[LPC11XX_UART_MAX_DEV] = {
	{0, (LPCUARTREG*)LPC_USART, }
};

bool LpcUARTWaitForRxFifo(LPCUARTDEV *pDev, uint32_t Timeout);
bool LpcUARTWaitForTxFifo(LPCUARTDEV *pDev, uint32_t Timeout);

#define UART_RX_CFIFO_SIZE			64
#define UART_TX_CFIFO_SIZE			16

#define UART_RX_CFIFO_MEM_SIZE			(UART_RX_CFIFO_SIZE + sizeof(CFIFOHDL))
#define UART_TX_CFIFO_MEM_SIZE			(UART_TX_CFIFO_SIZE + sizeof(CFIFOHDL))

uint32_t s_ErrCnt = 0;

uint8_t s_UARTRxFifoMem[UART_RX_CFIFO_MEM_SIZE];
uint8_t s_UARTTxFifoMem[UART_TX_CFIFO_MEM_SIZE];

void UART_IRQHandler(void)
{
	uint32_t iir = LPC_USART->IIR;
	uint32_t data;
	int cnt;
	uint32_t iid = iir & LPCUART_IIR_ID_MASK;

	if ((iir & LPCUART_IIR_STATUS) == 0)
	{
		switch (iid)//r & LPCUART_IIR_ID_MASK)
		{
			case LPCUART_IIR_ID_MS:
				s_ErrCnt++;
//				data = LPC_USART->MCR;
//				break;
			case LPCUART_IIR_ID_RLS:	// Line status
				{
					uint32_t r = LPC_USART->LSR;
					if (g_LpcUartDev->pUartDev->EvtCallback)
					{
						data = (r & LPCUART_LSR_OE) ? UART_LINESTATE_OVR : 0;
						data |= (r & LPCUART_LSR_PE) ? UART_LINESTATE_PARERR : 0;
						data |= (r & LPCUART_LSR_FE) ? UART_LINESTATE_FRMERR : 0;
						data |= (r & LPCUART_LSR_BI) ? UART_LINESTATE_BRK : 0;

						data |= (r << 16L);
						r = LPC_USART->MSR;

						if (r & LPCUART_MSR_DCTS)
						{
							g_LpcUartDev->pUartDev->LineState ^= UART_LINESTATE_CTS;
							data |= (g_LpcUartDev->pUartDev->LineState & UART_LINESTATE_CTS);
						}
						//data |= (r & LPCUART_MSR_CTS) ? UART_LINESTATE_CTS : 0;
						data |= (r & LPCUART_MSR_DDSR) ? UART_LINESTATE_DSR : 0;
						data |= (r & LPCUART_MSR_RI) ? UART_LINESTATE_RI : 0;
						data |= (r & LPCUART_MSR_DCD) ? UART_LINESTATE_DCD : 0;

						data |= (r << 24L);

						g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_LINESTATE, (uint8_t*)&data, 1);
					}
				}
				//break;

			case LPCUART_IIR_ID_CTIMOUT:
			case LPCUART_IIR_ID_RDA:
				{
					cnt = 0;
					uint32_t state = DisableInterrupt();
					while ((g_LpcUartDev->pUartReg->LSR & LPCUART_LSR_RDR) && cnt < 8) {
					//do {
						uint8_t *p = CFifoPut(g_LpcUartDev->pUartDev->hRxFifo);
						if (p == NULL)
							break;
						*p = g_LpcUartDev->pUartReg->RBR;
						cnt++;
					}
					//while ((g_LpcUartDev->pUartReg->LSR & LPCUART_LSR_RDR) && cnt < 14);
					cnt = CFifoUsed(g_LpcUartDev->pUartDev->hRxFifo);
					EnableInterrupt(state);

					if (g_LpcUartDev->pUartDev->EvtCallback)
					{
						int l = CFifoUsed(g_LpcUartDev->pUartDev->hRxFifo);
						if (iid == LPCUART_IIR_ID_CTIMOUT)
							g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXTIMEOUT, NULL, l);
						else
							g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXDATA, NULL, l);
					}
				}
				break;

			case LPCUART_IIR_ID_THRE:
				//data = LPC_USART->LSR;
			{
				cnt = 0;
				uint32_t state = DisableInterrupt();
				g_LpcUartDev->bTxReady = false;
				do {
					uint8_t *p = CFifoGet(g_LpcUartDev->pUartDev->hTxFifo);
					if (p == NULL)
					{
						g_LpcUartDev->bTxReady = true;
						break;
					}
					LPC_USART->THR = *p;
					cnt++;
				} //while (LpcUARTWaitForTxFifo(&g_LpcUartDev, 10) && cnt < 14);
				  while (g_LpcUartDev->pUartReg->LSR & (LPCUART_LSR_TEMT | LPCUART_LSR_THRE) && cnt < 14);
				EnableInterrupt(state);

				if (g_LpcUartDev->pUartDev->EvtCallback)
				{
					int len = CFifoAvail(g_LpcUartDev->pUartDev->hTxFifo);
					len = g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_TXREADY, NULL, len);
					if (len > 0)
					{
						//LpcUARTTxData(&g_LpcUartDev->pUartDev->SerIntrf, d, len);
//								LPC_USART->THR = d[0];
					}
					if (g_LpcUartDev->bTxReady)
					{
						if (g_LpcUartDev->pUartReg->LSR & (LPCUART_LSR_TEMT | LPCUART_LSR_THRE))
						{
							uint8_t *p = CFifoGet(g_LpcUartDev->pUartDev->hTxFifo);
							if (p == NULL)
							{
								g_LpcUartDev->bTxReady = true;
								break;
							}
							LPC_USART->THR = *p;
						}
					}
				}
			}
				break;
			default:
				;
		}
	}
	//NVIC_ClearPendingIRQ(UART_IRQn);
}

uint32_t LpcGetUartClk()
{
//	printf("Clock : %d %d\r\n", SystemClkFreq, SystemClkFreq / LPC_SYSCON->UARTCLKDIV);
	return SystemMainClkFreq / LPC_SYSCON->UARTCLKDIV;
}

bool UARTInit(UARTDEV *pDev, const UARTCFG *pCfg)
{
	LPCUARTREG *reg = NULL;
	//g_UartClkDiv = 1;

	switch (pCfg->DevNo)
	{
		case 0:
	        LPC_SYSCON->SYSAHBCLKCTRL |= LPC_SYSAHBCLKCTRL_UART0_EN;
	        reg = (void*)LPC_USART;
	        LPC_SYSCON->UARTCLKDIV = 2;//g_UartClkDiv; //PCLKSEL0 &= ~LPC_PCLKSEL0_UART0_MASK;	// CCLK/4
			break;
/*		case 1:
	        LPC_SYSCON->SYSAHBCLKCTRL |= LPC_SYSAHBCLKCTRL_UART1_EN;
	        pDev->pUartReg = (LPC_UART_TypeDef*)LPC_UART1;
	        LPC_SC->PCLKSEL0 &= ~LPC_PCLKSEL0_UART1_MASK;	// CCLK/4
			break;
		case 2:
	        LPC_SYSCON->SYSAHBCLKCTRL |= LPC_SYSAHBCLKCTRL_UART2_EN;
	        pDev->pUartReg = LPC_UART2;
	        LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_UART2_MASK;	// CCLK/4
			break;*/
		default:
			return false;
	}

	//LPC_USART_Type *reg = (LPC_USART_Type *)pDev->pUartReg;

	// Configure I/O pins
	int idx = 0;

	while (pCfg->PinCfg[idx].PortNo >= 0 && idx < UART_NB_PINS)
	{
		IOPinCfg(&pCfg->PinCfg[idx], 1);
		idx++;
	}
/*
	LPC_GPIO->SET[pCfg->PinCfg[UARTPIN_TX_IDX].PortNo] = (1 << pCfg->PinCfg[UARTPIN_TX_IDX].PinNo);
	if (pCfg->PinCfg[UARTPIN_CTS_IDX].PortNo >= 0)
		LPC_GPIO->CLR[pCfg->PinCfg[UARTPIN_CTS_IDX].PortNo] = (1 << pCfg->PinCfg[UARTPIN_CTS_IDX].PinNo);

	if (pCfg->PinCfg[UARTPIN_RTS_IDX].PortNo >= 0)
	{
		LPC_GPIO->CLR[pCfg->PinCfg[UARTPIN_RTS_IDX].PortNo] = (1 << pCfg->PinCfg[UARTPIN_RTS_IDX].PinNo);
	}
*/

	reg->TER = 0;	// Disable Tx
	reg->IER = 0;	// Disable all interrupts
	reg->ACR = 0;	// Disable auto baudrate

	// Clear all FIFO
	reg->FCR = LPCUART_FCR_RST_RXFIFO | LPCUART_FCR_RST_TXFIFO;


//	if (pCfg->DMAMode)
//		pDev->pUartReg->FCR |= LPCUART_FCR_DMA_MODE | LPCUART_FCR_RX_TRIG8;

	// Data bis, Parity, Stop bit
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

	g_LpcUartDev[pCfg->DevNo].DMAMode = pCfg->bDMAMode;
	g_LpcUartDev[pCfg->DevNo].pUartReg = reg;
	g_LpcUartDev[pCfg->DevNo].pUartDev = pDev;

	pDev->SerIntrf.pDevData = (void*)&g_LpcUartDev[pCfg->DevNo];

	if (pCfg->Rate)
		pDev->Rate = LpcUARTSetRate(&pDev->SerIntrf, pCfg->Rate);
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

	while (LPC_USART->LSR & ~(3<<5))
	{
		val = LPC_USART->RBR;
	}

	val = LPC_USART->IIR;	// Clear interrupts
	pDev->LineState = 0;

	//LPC_USART->MCR |= (1<<4); // Loopback

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_UARTRxFifoMem, UART_RX_CFIFO_MEM_SIZE, 1);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_UARTTxFifoMem, UART_TX_CFIFO_MEM_SIZE, 1);
	}

	// Start tx
	LPC_USART->TER = LPCUART_TER_TXEN;


	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->SerIntrf.Disable = LpcUARTDisable;
	pDev->SerIntrf.Enable = LpcUARTEnable;
	pDev->SerIntrf.GetRate = LpcUARTGetRate;
	pDev->SerIntrf.SetRate = LpcUARTSetRate;
	pDev->SerIntrf.StartRx = LpcUARTStartRx;
	pDev->SerIntrf.RxData = LpcUARTRxData;
	pDev->SerIntrf.StopRx = LpcUARTStopRx;
	pDev->SerIntrf.StartTx = LpcUARTStartTx;
	pDev->SerIntrf.TxData = LpcUARTTxData;
	pDev->SerIntrf.StopTx = LpcUARTStopTx;
	pDev->EvtCallback = pCfg->EvtCallback;

	g_LpcUartDev[pCfg->DevNo].bTxReady = true;

	pDev->LineState = 0;

	if (pCfg->bIntMode)
	{
		LPC_USART->IER = LPCUART_IER_THRE | LPCUART_IER_RBR | LPCUART_IER_RLS | LPCUART_IER_MS | (1<<7);
		NVIC_ClearPendingIRQ(UART_IRQn);
		NVIC_SetPriority(UART_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(UART_IRQn);
	}

	return true;
}

