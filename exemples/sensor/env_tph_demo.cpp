//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2017, I-SYST
// Description : Hello World in C++
//============================================================================

#include "i2c.h"
#include "spi.h"
#include "sensors/tph_bme280.h"
#include "sensors/tphg_bme680.h"
#include "blueio_board.h"
#include "board.h"

#define TPH_I2C

static const IOPINCFG s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP,
     IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {BME280_CS_PORT, BME280_CS_PIN, BME280_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const SPICFG s_SpiCfg = {
    SPI_DEVNO,
    SPIMODE_MASTER,
	s_SpiPins,
    sizeof( s_SpiPins ) / sizeof( IOPINCFG ),
    8000000,   // Speed in Hz
    8,      // Data Size
    5,      // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,         // clock polarity
    SPICSEL_AUTO,
    6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    nullptr
};

static const I2CCFG s_I2cCfg = {
	0,			// I2C device number
	{
		{I2C0_SDA_PORT, I2C0_SDA_PIN, I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
		{I2C0_SCL_PORT, I2C0_SCL_PIN, I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	},
	100000,	// Rate
	I2CMODE_MASTER,
	0,			// Slave address
	5,			// Retry
	7,			// Interrupt prio
	NULL		// Event callback
};

I2C g_I2c;
SPI g_Spi;

#ifdef	TPH_I2C
DeviceIntrf *g_pIntrf = &g_I2c;
#else
DeviceIntrf *g_pIntrf = &g_Spi;
#endif

static const TPHSENSOR_CFG s_TphSensorCfg = {

#ifdef	TPH_I2C
	BME280_I2C_DEV_ADDR0,		// Device address
#else
	0,
#endif
	TPHSENSOR_OPMODE_SINGLE,	// Operating mode
	100,						// Sampling frequency in Hz
	1,
	1,
	1,
	0
};

//TphBme280 g_TphSensor;
TphgBme680	g_TphSensor;
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
	uint8_t cdata[41];

	g_I2c.Init(s_I2cCfg);
//	g_Spi.Init(s_SpiCfg);

	bool res = g_TphSensor.Init(s_TphSensorCfg, g_pIntrf);


	while (res == true) {
		float t = g_TphSensor.ReadTemperature();
	}
	return 0;
}
