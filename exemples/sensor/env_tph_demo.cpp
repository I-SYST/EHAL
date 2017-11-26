//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2017, I-SYST
// Description : Hello World in C++
//============================================================================

#include "i2c.h"
#include "spi.h"
#include "uart.h"
#include "stddev.h"
#include "sensors/tph_bme280.h"
#include "sensors/tphg_bme680.h"
#include "blueio_board.h"
#include "board.h"
#include "idelay.h"

#define TPH_I2C

//********** UART **********
//int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_TxBuff[FIFOSIZE];

static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
const UARTCFG g_UartCfg = {
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
	g_TxBuff,
};

UART g_Uart;

#ifndef TPH_I2C
//********** SPI **********
static const IOPINCFG s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP,
     IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {BMEx80_CS_PORT, BMEx80_CS_PIN, BMEx80_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const SPICFG s_SpiCfg = {
    0,//SPI_DEVNO,
    SPIMODE_MASTER,
	s_SpiPins,
    sizeof( s_SpiPins ) / sizeof( IOPINCFG ),
    1000000,   // Speed in Hz
    8,      // Data Size
    5,      // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,         // clock polarity
    SPICSEL_AUTO,
    6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    nullptr
};

SPI g_Spi;
#endif

//********** I2C **********
static const I2CCFG s_I2cCfg = {
	0,			// I2C device number
	{
		{I2C0_SDA_PORT, I2C0_SDA_PIN, I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// RX
		{I2C0_SCL_PORT, I2C0_SCL_PIN, I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// TX
	},
	100000,	// Rate
	I2CMODE_MASTER,
	0,			// Slave address
	5,			// Retry
	7,			// Interrupt prio
	NULL		// Event callback
};

I2C g_I2c;

#ifdef	TPH_I2C
DeviceIntrf *g_pIntrf = &g_I2c;
#else
DeviceIntrf *g_pIntrf = &g_Spi;
#endif

//********** SENSOR **********
static const GASSENSOR_HEAT s_HeaterProfile[] = {
	{ 320, 150 },
};

static const GASSENSOR_CFG s_GasSensorCfg = {
#ifdef	TPH_I2C
	BME680_I2C_DEV_ADDR0,		// Device address
#else
	0,
#endif
	SENSOR_OPMODE_SINGLE,	// Operating mode
	100,
	sizeof(s_HeaterProfile) / sizeof(GASSENSOR_HEAT),
	s_HeaterProfile
};

static const TPHSENSOR_CFG s_TphSensorCfg = {

#ifdef	TPH_I2C
	BME680_I2C_DEV_ADDR0,		// I2C Device address
#else
	0,		// SPI CS index
#endif
	SENSOR_OPMODE_SINGLE,	// Operating mode
	100,						// Sampling frequency in Hz
	1,
	1,
	1,
	1
};

//TphBme280 g_EnvSensor;
TphgBme680	g_EnvSensor;

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
	TPHSENSOR_DATA tphdata;
	GASSENSOR_DATA gdata;
	bool res = true;

	res = g_Uart.Init(g_UartCfg);

	// retarget print to UART
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

#ifdef	TPH_I2C
	g_I2c.Init(s_I2cCfg);
#else
	g_Spi.Init(s_SpiCfg);
#endif
	res = g_EnvSensor.Init(s_TphSensorCfg, g_pIntrf, NULL);
	g_EnvSensor.Init(s_GasSensorCfg, g_pIntrf, NULL);

	g_EnvSensor.Read(tphdata);
	//g_EnvSensor.Read(gdata);
	g_EnvSensor.StartSampling();

 	while (res == true) {
 		// Gas sensor takes a long time to sample
		usDelay(200000);
		g_EnvSensor.Read(tphdata);
		//g_EnvSensor.Read(gdata);
		g_EnvSensor.StartSampling();

		printf("%u : Temp : %.2f C, Press : %.3f KPa, Humi : %.2f %%\r\n",
				tphdata.Timestamp,
				(float)tphdata.Temperature / 100.0,
				(float)tphdata.Pressure / 1000.0,
				(float)tphdata.Humidity / 100.0
				);
 	}
	return 0;
}
