/*--------------------------------------------------------------------------
File   : uart_lpc11uxx.h

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
#include "interrupt.h"

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

#define UART_RX_CFIFO_SIZE			16
#define UART_TX_CFIFO_SIZE			16

#define UART_RX_CFIFO_MEM_SIZE			(UART_RX_CFIFO_SIZE + sizeof(CFIFOHDR))
#define UART_TX_CFIFO_MEM_SIZE			(UART_TX_CFIFO_SIZE + sizeof(CFIFOHDR))

uint8_t s_UARTRxFifoMem[UART_RX_CFIFO_MEM_SIZE];
uint8_t s_UARTTxFifoMem[UART_TX_CFIFO_MEM_SIZE];

static int32_t s_RxFifoPeak = 0;

void UART_IRQHandler(void)
{
	uint32_t iir = LPC_USART->IIR;
	uint32_t data;
	int cnt;
	uint32_t iid = iir & LPCUART_IIR_ID_MASK;

	if ((iir & LPCUART_IIR_STATUS) == 0)
	{
		switch (iid)
		{
			case LPCUART_IIR_ID_MS:
			case LPCUART_IIR_ID_RLS:	// Line status
				{
					uint32_t r = LPC_USART->LSR;
					if (r & LPCUART_LSR_OE)
					{
						g_LpcUartDev->pUartDev->RxOECnt++;
					}
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
							g_LpcUartDev->pUartDev->LineState ^= UART_LINESTATE_DSR;
							data |= (g_LpcUartDev->pUartDev->LineState & UART_LINESTATE_DSR);
							g_LpcUartDev->pUartDev->LineState ^= UART_LINESTATE_CTS;
							data |= (g_LpcUartDev->pUartDev->LineState & UART_LINESTATE_CTS);
						}
						//data |= (r & LPCUART_MSR_CTS) ? UART_LINESTATE_CTS : 0;
						//data |= (r & LPCUART_MSR_DDSR) ? UART_LINESTATE_DSR : 0;
						data |= (r & LPCUART_MSR_RI) ? UART_LINESTATE_RI : 0;
						data |= (r & LPCUART_MSR_DCD) ? UART_LINESTATE_DCD : 0;

						data |= (r << 24L);

						g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_LINESTATE, (uint8_t*)&data, 4);
					}
				}
				break;

			case LPCUART_IIR_ID_CTIMOUT:
			case LPCUART_IIR_ID_RDA:
				cnt = 0;
				// TRG4 & 14 : works well at 1 Mbaud
				// Disable interrupt is require here, otherwise there will be drop
				// DONOT remove this
				uint32_t state = DisableInterrupt();
				while ((g_LpcUartDev->pUartReg->LSR & LPCUART_LSR_RDR) && cnt < 14)
				{
					uint8_t *p = CFifoPut(g_LpcUartDev->pUartDev->hRxFifo);
					if (p == NULL)
						break;
					*p = g_LpcUartDev->pUartReg->RBR;
					cnt++;
				}
				EnableInterrupt(state);
				cnt = CFifoUsed(g_LpcUartDev->pUartDev->hRxFifo);
				if (cnt > s_RxFifoPeak)
				{
					s_RxFifoPeak = cnt;
				}
				if (g_LpcUartDev->pUartDev->EvtCallback)
				{
					if (iid == LPCUART_IIR_ID_CTIMOUT)
						g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXTIMEOUT, NULL, cnt);
					else //if (cnt > 8)
						g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXDATA, NULL, cnt);
				}
				break;

			case LPCUART_IIR_ID_THRE:
			{
				cnt = 0;
				//uint32_t state = DisableInterrupt();
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
				} while ((g_LpcUartDev->pUartReg->LSR & (LPCUART_LSR_TEMT | LPCUART_LSR_THRE)) && cnt < 14);
				//EnableInterrupt(state);

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
	NVIC_ClearPendingIRQ(UART_IRQn);
}

uint32_t LpcGetUartClk()
{
//	printf("Clock : %d %d\r\n", SystemClkFreq, SystemClkFreq / LPC_SYSCON->UARTCLKDIV);
	return SystemMainClkFreq / LPC_SYSCON->UARTCLKDIV;
}

inline void LpcUARTDisable(DEVINTRF *pDev)
{
}

void LpcUARTEnable(DEVINTRF *pDev)
{
}

bool UARTInit(UARTDEV *pDev, const UARTCFG *pCfg)
{
	LPCUARTREG *reg = NULL;

	switch (pCfg->DevNo)
	{
		case 0:
	        LPC_SYSCON->SYSAHBCLKCTRL |= LPC_SYSAHBCLKCTRL_UART0_EN;
	        reg = (void*)LPC_USART;
	        LPC_SYSCON->UARTCLKDIV = 2;//g_UartClkDiv; //PCLKSEL0 &= ~LPC_PCLKSEL0_UART0_MASK;	// CCLK/4
			break;
		default:
			return false;
	}

	// Configure I/O pins
	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIOPinMap;
	IOPinCfg(pincfg, pCfg->NbIOPins);


	reg->TER = 0;	// Disable Tx
	reg->IER = 0;	// Disable all interrupts
	reg->ACR = 0;	// Disable auto baudrate

	// Clear all FIFO
	reg->FCR = LPCUART_FCR_RST_RXFIFO | LPCUART_FCR_RST_TXFIFO;

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

	pDev->DevIntrf.pDevData = (void*)&g_LpcUartDev[pCfg->DevNo];

	if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		reg->MCR |= (3 << 6);	// Auto CTS/RTS flow control

	}
	else
	{
		reg->MCR &= ~(3 << 6);
	}

	reg->FCR = LPCUART_FCR_FIFOEN | LPCUART_FCR_RST_RXFIFO | LPCUART_FCR_RST_TXFIFO |
			   LPCUART_FCR_RX_TRIG8;

	uint32_t val;

	while (LPC_USART->LSR & ~(3<<5))
	{
		val = LPC_USART->RBR;
	}

	val = LPC_USART->IIR;	// Clear interrupts
	pDev->LineState = 0;

	if (pCfg->Rate > 0)
	{
		pDev->Rate = LpcUARTSetRate(&pDev->DevIntrf, pCfg->Rate);
	}
	else
	{
		// Auto baudrate
		reg->ACR = 7;
	}
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

	s_RxFifoPeak = 0;

	// Start tx
	LPC_USART->TER = LPCUART_TER_TXEN;

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UART;
	pDev->RxOECnt = 0;
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
	pDev->DevIntrf.MaxRetry = 0;
	pDev->EvtCallback = pCfg->EvtCallback;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

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

