//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2018, I-SYST inc.
// Description : Hello World in C++
//============================================================================

#include "uart.h"
#include "spi.h"
#include "stddev.h"
#include "board.h"

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_UarTxBuff[FIFOSIZE];

// Assign UART pins
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
	NULL,//nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_UarTxBuff,
};

UART g_Uart;

//********** SPI Master **********
static const IOPINCFG s_SpiMasterPins[] = {
    {0, 22, 1,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// SCK
    {0, 23, 1,
     IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// MISO
    {0, 24, 1,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MOSI
    {0, 25, 0,
     IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// CS
};

static const SPICFG s_SpiMasterCfg = {
    0,//SPI_DEVNO,
    SPIMODE_MASTER,
	s_SpiMasterPins,
    sizeof( s_SpiMasterPins ) / sizeof( IOPINCFG ),
    1000000,   // Speed in Hz
    8,      // Data Size
    5,      // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,         // clock polarity
    SPICSEL_AUTO,
	true,
    6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    NULL
};

SPI g_SpiMaster;

//********** SPI Slave **********

int SpiSlaveHandler(DEVINTRF * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

static const IOPINCFG s_SpiSlavePins[] = {
    {0, 26, 1,
     IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// SCK
    {0, 27, 1,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// MISO
    {0, 28, 1,
     IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MOSI
    {0, 29, 0,
     IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const SPICFG s_SpiSlaveCfg = {
    1,//SPI_DEVNO,
    SPIMODE_SLAVE,
	s_SpiSlavePins,
    sizeof( s_SpiSlavePins ) / sizeof( IOPINCFG ),
    1000000,   // Speed in Hz
    8,      // Data Size
    5,      // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,         // clock polarity
    SPICSEL_AUTO,
	true,
    6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
	SpiSlaveHandler
};

SPI g_SpiSlave;

uint8_t g_SpiSlaveTxBuff[512];
uint8_t g_SpiSlaveRxBuff[512];

int SpiSlaveHandler(DEVINTRF * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	switch (EvtId)
	{
		case DEVINTRF_EVT_STATECHG:
			// Update new RX/TX buffer here
			g_SpiSlave.SetSlaveRxBuffer(0, g_SpiSlaveRxBuff, 10);
			g_SpiSlave.SetSlaveTxData(0, g_SpiSlaveTxBuff, 10);
			break;

		case DEVINTRF_EVT_COMPLETED:
			printf("transfer completed %d bytes in %p\r\n", Len, pBuffer);

			// just copy the received data to the tx buffer for next read from master
			memcpy(g_SpiSlaveTxBuff, pBuffer, Len);

			break;
	}
}

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	printf("Init SPI Master/Slave demo\r\n");
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

	g_SpiMaster.Init(s_SpiMasterCfg);
	g_SpiSlave.Init(s_SpiSlaveCfg);

	uint8_t data[100];
	uint8_t buff[100];

	// Fill dummy data
	for (int i = 0; i < 100; i++)
	{
		data[i] = i;
	}

	memset(buff, 0xff, 100);

	g_SpiMaster.Write(0, data, 1, data, 10);
	int cnt = g_SpiMaster.Read(0, data, 1, buff, 8);

	printf("Received : ");
	for (int i = 0; i < cnt; i++)
	{
		printf("%x ", buff[i]);
	}
	printf("\r\n");

	while(1)
	{
		__WFE();
	}

	return 0;
}
