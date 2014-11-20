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

bool LpcUARTInit(UARTDEV *pDev, const UARTCFG *pCfg)
{

	switch (pCfg->DevNo)
	{
		case 0:
	        LPC_SYSCON->SYSAHBCLKCTRL |= LPC_SYSAHBCLKCTRL_UART0_EN;
	        pDev->pUartReg = (void*)LPC_USART;
	        LPC_SYSCON->UARTCLKDIV = 4; //PCLKSEL0 &= ~LPC_PCLKSEL0_UART0_MASK;	// CCLK/4
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

	LPC_USART_Type *reg = (LPC_USART_Type *)pDev->pUartReg;

	// Configure I/O pins
	int idx = 0;

	while (pCfg->PinCfg[idx].PortNo >= 0 && idx < LPCUART_NB_PINS)
	{
		IOPinCfg(&pCfg->PinCfg[idx], 1);
		idx++;
	}

	pDev->Cfg = *pCfg;

	reg->TER = 0;	// Disable Tx
	reg->IER = 0;	// Disable all interrupts
	reg->ACR = 0;	// Disable auto baudrate

	// Clear all FIFO
	reg->FCR = LPCUART_FCR_RST_RXFIFO | LPCUART_FCR_RST_TXFIFO;

//	if (pCfg->DMAMode)
//		pDev->pUartReg->FCR |= LPCUART_FCR_DMA_MODE | LPCUART_FCR_RX_TRIG8;

	reg->LCR = (pCfg->DataBits - 5);
	if (pCfg->Parity != UART_PARITY_NONE)
	{
		reg->LCR |= (pCfg->Parity << 4);
	}

	if (pCfg->StopBits > 1)
		reg->LCR |= LPCUART_LCR_STOPBIT_MASK;

	reg->ICR = 0;
	if (pCfg->IrDAMode)
	{
		if (pCfg->IrDAInvert)
			reg->ICR |= LPCUART_ICR_IRDAINV;
		if (pCfg->IrDAFixPulse)
			reg->ICR |= LPCUART_ICR_IRDA_FIXPULSE | ((pCfg->IrDAPulseDiv & 7) << 3);
		reg->ICR |= LPCUART_ICR_IRDAEN;
	}

	if (pCfg->Rate)
		pDev->Cfg.Rate = LpcUARTSetRate(pDev, pCfg->Rate);
	else
	{
		// Auto baudrate
		reg->ACR = 7;
	}

	reg->FCR |= LPCUART_FCR_FIFOEN;

	return true;
}

