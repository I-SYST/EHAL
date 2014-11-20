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
#include "lpcuart.h"
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

bool LpcUARTInit(UARTDEV *pDev, UARTCFG *pCfg)
{
	switch (pCfg->DevNo)
	{
		case 0:
	        LPC_SC->PCONP |= LPC_PCONP_UART0;
	        pDev->pUartReg = (LPCUARTREG*)LPC_UART0;
	        LPC_SC->PCLKSEL0 &= ~LPC_PCLKSEL0_UART0_MASK;	// CCLK/4
			break;
		case 1:
			LPC_SC->PCONP |= LPC_PCONP_UART1;
	        pDev->pUartReg = (LPCUARTREG*)LPC_UART1;
	        LPC_SC->PCLKSEL0 &= ~LPC_PCLKSEL0_UART1_MASK;	// CCLK/4
			break;
		case 2:
			LPC_SC->PCONP |= LPC_PCONP_UART2;
	        pDev->pUartReg = (LPCUARTREG*)LPC_UART2;
	        LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_UART2_MASK;	// CCLK/4
			break;
		case 3:
			LPC_SC->PCONP |= LPC_PCONP_UART3;
	        pDev->pUartReg = (LPCUARTREG*)LPC_UART3;
	        LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_UART3_MASK;	// CCLK/4
			break;
		default:
			return false;
	}

	// Configure I/O pins
	int idx = 0;

	while (pCfg->PinCfg[idx].PortNo >= 0 && idx < LPCUART_NB_PINS)
	{
		IOPinCfg(&pCfg->PinCfg[idx], 1);
		idx++;
	}

	pDev->Cfg = *pCfg;

	pDev->pUartReg->TER = 0;	// Disable Tx
	pDev->pUartReg->IER = 0;	// Disable all interrupts
	pDev->pUartReg->ACR = 0;	// Disable auto baudrate

	// Clear all FIFO
	pDev->pUartReg->FCR = LPCUART_FCR_RST_RXFIFO | LPCUART_FCR_RST_TXFIFO;

	if (pCfg->DMAMode)
		pDev->pUartReg->FCR |= LPCUART_FCR_DMA_MODE | LPCUART_FCR_RX_TRIG8;

	pDev->pUartReg->LCR = (pCfg->DataBits - 5);
	if (pCfg->Parity != UART_PARITY_NONE)
	{
		pDev->pUartReg->LCR |= (pCfg->Parity << 4);
	}

	if (pCfg->StopBits > 1)
		pDev->pUartReg->LCR |= LPCUART_LCR_STOPBIT_MASK;

	pDev->pUartReg->ICR = 0;
	if (pCfg->IrDAMode)
	{
		if (pCfg->IrDAInvert)
			pDev->pUartReg->ICR |= LPCUART_ICR_IRDAINV;
		if (pCfg->IrDAFixPulse)
			pDev->pUartReg->ICR |= LPCUART_ICR_IRDA_FIXPULSE | ((pCfg->IrDAPulseDiv & 7) << 3);
		pDev->pUartReg->ICR |= LPCUART_ICR_IRDAEN;
	}

	if (pCfg->Rate)
		pDev->Cfg.Rate = LpcUARTSetRate(pDev, pCfg->Rate);
	else
	{
		// Auto baudrate
		pDev->pUartReg->ACR = 7;
	}

	pDev->pUartReg->FCR |= LPCUART_FCR_FIFOEN;

	return true;
}





