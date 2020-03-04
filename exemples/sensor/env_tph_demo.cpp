/**-------------------------------------------------------------------------
@example	env_tph_demo.cpp


@brief	Environmental sensor demo.

This application demo shows how to use environmental driver (BME280, BME680, MS8607).
Temperature, Pressure, Humidity (TPH). Support I2C or SPI interface.  Data is printed
to UART.

NOTE : The BME680 Air Quality Index is undocumented.  It requires the library
Bosch Sensortec Environmental Cluster (BSEC) Software. Download from
https://www.bosch-sensortec.com/bst/products/all_products/bsec and put in
external folder as indicated on the folder tree.

The BSEC library must be initialized in the main application prior to initializing
this driver by calling the function

bsec_library_return_t res = bsec_init();

@author Hoang Nguyen Hoan
@date	May 8, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include <inttypes.h>

#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "timer_nrf5x.h"
#include "sensors/tph_bme280.h"
#include "sensors/tphg_bme680.h"
#include "sensors/tph_ms8607.h"
#include "blueio_board.h"
#include "idelay.h"

#include "sensors/bsec_interface.h"

#define BME680		// To use Bosch BME680 with Air Quality Index
#include "board.h"

void TimerHandler(Timer *pTimer, uint32_t Evt);

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 7,
	.EvtHandler = NULL,//TimerHandler
};

TimerLFnRF5x g_Timer;

//********** UART **********
//int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_TxBuff[FIFOSIZE];

// Assign UART pins
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

//********** SPI **********
static const IOPINCFG s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP,
     IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {BMEx80_CS_PORT, BMEx80_CS_PIN, BMEx80_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const SPICFG s_SpiCfg = {
    0,//SPI_DEVNO,
	SPITYPE_NORMAL,
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
	true, //DMA
	false,
    6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    NULL
};

SPI g_Spi;

//********** I2C **********
static const I2CCFG s_I2cCfg = {
	0,			// I2C device number
	{
		{I2C0_SDA_PORT, I2C0_SDA_PIN, I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
		{I2C0_SCL_PORT, I2C0_SCL_PIN, I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	},
	100000,	// Rate
	I2CMODE_MASTER,
	5,			// Retry
	0,			// Number of slave addresses
	{0,},		// Slave addresses
	true,	// DMA
	false,		// Use interrupt
	7,			// Interrupt prio
	NULL		// Event callback
};

I2C g_I2c;

#define TPH_I2C		// To use I2C interface

#ifdef	TPH_I2C
DeviceIntrf *g_pIntrf = &g_I2c;
#else
DeviceIntrf *g_pIntrf = &g_Spi;
#endif

//********** SENSOR **********
static const GASSENSOR_HEAT s_HeaterProfile[] = {
	{ 375, 125 },
};

static const GASSENSOR_CFG s_GasSensorCfg = {
#ifdef	TPH_I2C
	BME680_I2C_DEV_ADDR0,	// I2C Device address
#else
	0,						// SPI CS index
#endif
	SENSOR_OPMODE_SINGLE,	// Operating mode
	500,
	sizeof(s_HeaterProfile) / sizeof(GASSENSOR_HEAT),
	s_HeaterProfile
};

static const TPHSENSOR_CFG s_TphSensorCfg = {

#ifdef	TPH_I2C
	BME680_I2C_DEV_ADDR0,	// I2C Device address
#else
	0,						// SPI CS index
#endif
	SENSOR_OPMODE_SINGLE,	// Operating mode
	1000,					// Sampling frequency in mHz
	2,
	1,
	1,
	1
};


#ifdef BME680
TphgBme680	g_EnvSensor;
#else
TphBme280 g_EnvSensor;
//TphMS8607	g_EnvSensor;
#endif

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
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

    g_Timer.Init(s_TimerCfg);

#ifdef	TPH_I2C
	g_I2c.Init(s_I2cCfg);
#else
	g_Spi.Init(s_SpiCfg);
#endif

#ifdef BME680
	bsec_library_return_t bsec_status;

	// NOTE : For BME680 air quality calculation, this library is require to be initialized
	// before initializing the sensor driver.
	bsec_status = bsec_init();

	if (bsec_status != BSEC_OK)
	{
		printf("BSEC init failed\r\n");

		return 1;
	}
#endif

	res = g_EnvSensor.Init(s_TphSensorCfg, g_pIntrf, &g_Timer);

	if (res == false)
	{
		printf("Init error\r\n");
	}

#ifdef BME680
	g_EnvSensor.Init(s_GasSensorCfg, g_pIntrf, &g_Timer);
#endif

	float lastiaq = -1;

	while (1)
	{
		g_EnvSensor.StartSampling();


        // Gas sensor takes a long time to sample
		usDelay(1000000);

		g_EnvSensor.Read(tphdata);
#ifdef BME680
		g_EnvSensor.Read(gdata);

#endif
		printf("T=%" PRIu64 " : Temp : %.2f C, Press : %.3f KPa, Humi : %.2f %% ",
				tphdata.Timestamp,
				(float)tphdata.Temperature / 100.0,
				(float)tphdata.Pressure / 1000.0,
				(float)tphdata.Humidity / 100.0);

#ifdef BME680
		if (lastiaq != gdata.AirQualIdx)
		{
			lastiaq = gdata.AirQualIdx;
		}
		printf("Gas = %d %.2f\r\n", gdata.GasRes[gdata.MeasIdx], gdata.AirQualIdx);
#else
		printf("\r\n");
#endif

 	}

	return 0;
}
