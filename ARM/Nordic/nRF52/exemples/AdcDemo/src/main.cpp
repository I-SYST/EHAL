//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2017, I-SYST
// Description : Hello World in C++
//============================================================================

#include "adc_nrf52.h"
#include "uart.h"
#include "stddev.h"

// This include contain i/o definition the board in use
#include "board.h"

//#define ADC_DEMO_SINGLE_SHOT
#define ADC_DEMO_INTERRUPT_ENABLE

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void ADVEventHandler(ADCDevice *pDevObj, ADC_EVT Evt);

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_TxBuff[FIFOSIZE];

static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
static const UARTCFG s_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPINCFG),
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_TxBuff,
};

UART g_Uart;

// Define available voltage sources
static const ADC_REFVOLT s_RefVolt[] = {
	{.Type = ADC_REFVOLT_TYPE_INTERNAL, .Voltage = 0.6 },
	{.Type = ADC_REFVOLT_TYPE_SUPPLY, .Voltage = 3.3 / 4.0},
};

static const int s_NbRefVolt = sizeof(s_RefVolt) / sizeof(ADC_REFVOLT);

#define ADC_CFIFO_SIZE		CFIFO_TOTAL_MEMSIZE(200, sizeof(ADC_DATA))

static uint8_t s_AdcFifoMem[ADC_CFIFO_SIZE];

// Define ADC device
static const ADC_CFG s_AdcCfg = {
#ifdef ADC_DEMO_SINGLE_SHOT
	.Mode = ADC_CONV_MODE_SINGLE,
#else
	.Mode = ADC_CONV_MODE_CONTINUOUS,
#endif
	.pRefVolt = s_RefVolt,
	.NbRefVolt = s_NbRefVolt,
	.DevAddr = 0,
	.Resolution = 10,
	.Rate = 8000,
	.OvrSample = 0,
#ifdef ADC_DEMO_INTERRUPT_ENABLE
	.bInterrupt = true,
#else
	.bInterrupt = false,
#endif
	.IntPrio = 6,
	.EvtHandler = ADVEventHandler,
	ADC_CFIFO_SIZE,
	s_AdcFifoMem
};

ADCnRF52 g_Adc;

// Define ADC channel
static const ADC_CHAN_CFG s_ChanCfg[] = {
	{
		.Chan = 0,
		.RefVoltIdx = 1,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 4,//1 << 8,
		.AcqTime = 0,
		.BurstMode = false,
		.PinP = { .PinNo = 0, .Conn = ADC_PIN_CONN_VDD },
	},
	{
		.Chan = 1,
		.RefVoltIdx = 1,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 4,//1 << 8,
		.AcqTime = 0,
		.BurstMode = false,
		.PinP = { .PinNo = 1, .Conn = ADC_PIN_CONN_NONE },
	}
};

static const int s_NbChan = sizeof(s_ChanCfg) / sizeof(ADC_CHAN_CFG);

volatile bool g_bDataReady = false;

void ADVEventHandler(ADCDevice *pAdcDev, ADC_EVT Evt)
{
	if (Evt == ADC_EVT_DATA_READY)
	{
		g_bDataReady = true;
#if 0 //def ADC_DEMO_INTERRUPT_ENABLE
		int cnt = 0;

		do {
			ADC_DATA df[2];
			cnt = g_Adc.Read(df, 2);
			if (cnt > 0)
				g_Uart.printf("%d ADC[0] = %.2f V, ADC[1] = %.2f V\r\n", df[0].Timestamp, df[0].Data, df[1].Data);
		} while (cnt > 0);
#ifdef ADC_DEMO_SINGLE_SHOT
			g_Adc.StartConversion();
#endif
#endif
	}
}

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:

			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	printf("Init ADC\r\n");

	g_Adc.Init(s_AdcCfg, NULL);
	g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
}
//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	HardwareInit();
	g_Adc.StartConversion();

	while (1)
	{
		__WFE();
//		if (g_bDataReady == true)
		{
//			g_bDataReady = false;
#if 1// !defined(ADC_DEMO_INTERRUPT_ENABLE)
			int cnt = 0;

			do {
				ADC_DATA df[2];
				memset(df, 0, sizeof(df));
				cnt = g_Adc.Read(df, 2);
				if (cnt > 0)
					g_Uart.printf("%d ADC[0] = %.2f V, ADC[1] = %.2f V\r\n", df[0].Timestamp, df[0].Data, df[1].Data);
			} while (cnt > 0);
#ifdef ADC_DEMO_SINGLE_SHOT
			g_Adc.StartConversion();
#endif
#endif
		}
	}

	return 0;
}
