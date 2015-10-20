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
#include "lpcuart.h"

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

void UART_IRQHandler(void)
{
	uint32_t iir = LPC_USART->IIR;
	uint32_t data;
//	uint8_t d[80];
//	int idx = 0;
	int cnt;
	//uint32_t lsr = LPC_USART->LSR;
	uint32_t iid = iir & LPCUART_IIR_ID_MASK;
	//bool flushrx = false;
	static uint8_t d[20];

	if ((iir & LPCUART_IIR_STATUS) == 0)
	{
		switch (iid)//r & LPCUART_IIR_ID_MASK)
		{
			case LPCUART_IIR_ID_MS:
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

						r = LPC_USART->MSR;

						//data |= (r & LPCUART_MSR_CTS) ? UART_LINESTATE_CTS : 0;
						data |= (r & LPCUART_MSR_DSR) ? UART_LINESTATE_DSR : 0;
						data |= (r & LPCUART_MSR_RI) ? UART_LINESTATE_RI : 0;
						data |= (r & LPCUART_MSR_DCD) ? UART_LINESTATE_DCD : 0;


						g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_LINESTATE, &data, 1);
					}
				}
				break;

			case LPCUART_IIR_ID_CTIMOUT:
				//flushrx = true;
				//break;
			case LPCUART_IIR_ID_RDA:
			//case LPCUART_IIR_ID_THRE:
				cnt = 0;
				while (g_LpcUartDev->pUartReg->LSR & LPCUART_LSR_RDR)
				{
					d[cnt++] = g_LpcUartDev->pUartReg->RBR;
				}
				//data = LPC_USART->RBR;
				if (g_LpcUartDev->pUartDev->EvtCallback)
				{
					if (iid == LPCUART_IIR_ID_CTIMOUT)
						g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXTIMEOUT, d, cnt);
					else
						g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXDATA, d, cnt);
				}
				break;

			case LPCUART_IIR_ID_THRE:
				g_LpcUartDev->bSending = false;
				//data = LPC_USART->LSR;
				if (g_LpcUartDev->pUartDev->EvtCallback)
				{
					cnt = g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_TXREADY, &d, 16);
					for (int i = 0; i < cnt; i++)
					{
						LPC_USART->THR = d[i];
					}
				}
				break;
			default:
				;
		}
	}
	else
	{
		cnt = 0;
		while (g_LpcUartDev->pUartReg->LSR & LPCUART_LSR_RDR)
		{
			d[cnt++] = g_LpcUartDev->pUartReg->RBR;
		}
		if (g_LpcUartDev->pUartDev->EvtCallback)
		{
			if (iid == LPCUART_IIR_ID_CTIMOUT)
				g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXTIMEOUT, d, cnt);
			else
				g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_RXDATA, d, cnt);
		}
		if ((g_LpcUartDev->pUartReg->LSR & LPCUART_LSR_THRE) && g_LpcUartDev->pUartDev->EvtCallback)
		{
			cnt = g_LpcUartDev->pUartDev->EvtCallback(g_LpcUartDev->pUartDev, UART_EVT_TXREADY, &d, 16);
			for (int i = 0; i < cnt; i++)
			{
				LPC_USART->THR = d[i];
			}
		}

	}
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

	//LPC_USART->MCR |= (1<<4); // Loopback

	// Start tx
	LPC_USART->TER = LPCUART_TER_TXEN;


	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->IrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->IrDAInvert = pCfg->bIrDAInvert;
	pDev->IrDAMode = pCfg->bIrDAMode;
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

	if (pCfg->bIntMode)
	{
		NVIC_ClearPendingIRQ(UART_IRQn);
		NVIC_SetPriority(UART_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(UART_IRQn);
		LPC_USART->IER = LPCUART_IER_THRE | LPCUART_IER_RBR | LPCUART_IER_RLS | LPCUART_IER_MS;
	}

	return true;
}

