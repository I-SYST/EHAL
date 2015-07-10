//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "LPC11Uxx.h"
#include "lpcuart.h"

#define SYSAHBCLKCTRL_GPIO		(1UL << 6)	// Enables clock for GPIO port registers.
#define SYSAHBCLKCTRL_USB		(1UL << 14)	// Enables clock to the USB register interface
#define SYSAHBCLKCTRL_USBRAM	(1UL << 27)	// Enables USB SRAM block at address 0x20004000
#define SYSAHBCLKCTRL_SRAM1		(1UL << 26)

#define UART_RXD_PORT				0
#define UART_RXD_PIN				18
#define UART_TXD_PORT				0
#define UART_TXD_PIN				19

const UARTCFG g_UartCfg = {
	0,
	{
		{UART_RXD_PORT, UART_RXD_PIN, 1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
		{UART_TXD_PORT, UART_TXD_PIN, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	},
	115200,
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
	UART_FLWCTRL_NONE,
};

// UART device instance
UART g_Uart;

//UARTDEV *g_pUartDev = g_Uart;

uint8_t bbb[100];

extern "C" void UART_IRQHandler(void)
{
	uint32_t iir = LPC_USART->IIR;

	if ((iir & LPCUART_IIR_STATUS) == 0)
	{
		switch (iir & LPCUART_IIR_ID_MASK)
		{
		case 6: // Line Status
		{
			uint32_t ls = LPC_USART->LSR;
			if (ls & 1)
				bbb[0] = LPC_USART->RBR;
		}
		case LPCUART_IIR_ID_RDA:
			bbb[0] = LPC_USART->RBR;
			break;
		}
	}
}

void HardwareInit()
{
	// Power on require subsystems
	LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_GPIO | SYSAHBCLKCTRL_USB |
								 SYSAHBCLKCTRL_USBRAM | SYSAHBCLKCTRL_SRAM1;

	g_Uart.Init(g_UartCfg);
}

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
/*#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
*/
int
main (int argc, char* argv[])
{
	uint32_t len = 10;
	uint8_t buffer[100];
	uint8_t data[] = {"1234567890"};

	HardwareInit();

	while(1)
	{
		memset(buffer, 0, 100);
		len = g_Uart.Tx(data, 10);
		len = g_Uart.Rx(buffer, 100);
		if (len > 0)
		{
			uint8_t a = buffer[0];
		}
	}
}

//#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
