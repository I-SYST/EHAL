//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2017, I-SYST
// Description : Hello World in C++
//============================================================================

#include "i2c.h"
#include "spi.h"
#include "sensors/pth_bme280.h"
#include "blueio_board.h"
#include "bme280.h"
#include "board.h"

#define PTH_I2C

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
		{BLUEIO_TAG_BME280_I2C_SDA_PORT, BLUEIO_TAG_BME280_I2C_SDA_PIN, BLUEIO_TAG_BME280_I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
		{BLUEIO_TAG_BME280_I2C_SCL_PORT, BLUEIO_TAG_BME280_I2C_SCL_PIN, BLUEIO_TAG_BME280_I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	},
	100000,	// Rate
	I2CMODE_MASTER,
	0,			// Slave address
	5,			// Retry
	7,			// Interrupt prio
	NULL		// Event callback
};

#ifdef PTH_I2C
I2C g_I2c;
#else
SPI g_Spi;
#endif

static const PTHSENSOR_CFG s_PthSensorCfg = {

#ifdef	PTH_I2C
	BME280_I2C_DEV_ADDR0,		// Device address
#else
	1,
#endif
	PTHSENSOR_OPMODE_CONTINUOUS,	// Operating mode
	100						// Sampling frequency in Hz
};

PthBme280 g_PthSensor;
extern "C" {
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	return g_I2c.Read(dev_addr, &reg_addr, 1, reg_data, cnt);
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	return g_I2c.Write(dev_addr, &reg_addr, 1, reg_data, cnt);
}
s32 bme280_data_readout_template(void);
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
extern struct bme280_t bme280;
extern uint8_t *pCalibData;

int main()
{
	uint8_t cdata[41];

#ifdef	PTH_I2C
	g_I2c.Init(s_I2cCfg);
	bool res = g_PthSensor.Init(s_PthSensorCfg, &g_I2c);
#else
	g_Spi.Init(s_SpiCfg);
	g_PthSensor.Init(s_PthSensorCfg, &g_Spi);
#endif

	//bme280_data_readout_template();
/*
	//if (memcmp(&bme280.cal_param, pCalibData, sizeof(bme280.cal_param)) != 0)
	uint8_t *p = (uint8_t*)&bme280.cal_param;
	for (int i = 0; i < sizeof(bme280.cal_param); i++)
	{
		if (p[i] != pCalibData[i])
		{
			uint8_t a = p[i];
			uint8_t b = pCalibData[i];
			printf("Failed %d %d %d\r\n", i, a, b);
		}
	}*/

	while (res == true) {
	float t = g_PthSensor.ReadTemperature();
	}
	return 0;
}
