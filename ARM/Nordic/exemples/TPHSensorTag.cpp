/*--------------------------------------------------------------------------
File   : PTHSensorTag.cpp

Author : Hoang Nguyen Hoan          May 8, 2017

Desc   : Environmental Sensor BLE demo
		 This application demo shows BLE connectionless using EHAL library
		 It advertises Barometric Pressure, Temperature & Humidity data
		 in manufacturer specific data

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <string.h>

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"
#include "nrf_power.h"

#include "blueio_board.h"
#include "uart.h"
#include "i2c.h"
#include "spi.h"
#include "custom_board.h"
#include "iopincfg.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "tph_bme280.h"
#include "tph_ms8607.h"
#include "tphg_bme680.h"
#include "timer_nrf5x.h"
#include "board.h"

#define DEVICE_NAME                     "TPHSensorTag"                            /**< Name of device. Will be included in the advertising data. */

//#define TPH_BME280
#define TPH_BME680

// Use timer to update data
// NOTE :	RTC timer 0 used by radio, RTC Timer 1 used by SDK
//			Only RTC timer 2 is usable with Softdevice for nRF52, not avail on nRF51
//
#ifdef NRF52
#define USE_TIMER_UPDATE
//#define NEBLINA_MODULE
#endif

// NOTE : Min advertisement interval for S130 v2 is 100 ms
#define APP_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#ifdef USE_TIMER_UPDATE
// Use timer to update date
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */
#else
// Use advertisement timeout to update data
#define APP_ADV_TIMEOUT_IN_SECONDS      1                                         /**< The advertising timeout (in units of seconds). */
#endif

void TimerHandler(Timer *pTimer, uint32_t Evt);

#pragma pack(push, 1)
typedef struct __EnvData {
	uint32_t Pressure;		// Barometric pressure in Pa no decimal
	int16_t  Temperature;	// Temperature in degree C, 2 decimals fixed point
	uint16_t Humidity;		// Relative humidity in %, 2 decimals fixed point
} ENVSENSOR_DATA;
#pragma pop(pop)

uint8_t g_AdvDataBuff[sizeof(ENVSENSOR_DATA) + 1] = {
	BLEAPP_ADV_MANDATA_TYPE_TPH,
};

BLEAPP_ADV_MANDATA &g_AdvData = *(BLEAPP_ADV_MANDATA*)g_AdvDataBuff;

// Evironmental Sensor Data to advertise
ENVSENSOR_DATA &g_TPHData = *(ENVSENSOR_DATA *)g_AdvData.Data;

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 2,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = APP_IRQ_PRIORITY_LOW,
	.EvtHandler = TimerHandler
};

TimerLFnRF5x g_Timer;

const BLEAPP_CFG s_BleAppCfg = {
	{ // Clock config nrf_clock_lf_cfg_t
#ifdef IMM_NRF51822
		NRF_CLOCK_LF_SRC_RC,	// Source RC
		1, 1, 0
#else
		NRF_CLOCK_LF_SRC_XTAL,	// Source 32KHz XTAL
		0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM// NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
#endif

	},
	0, 						// Number of central link
	0, 						// Number of peripheral link
	BLEAPP_MODE_NOCONNECT,   // Connectionless beacon type
	DEVICE_NAME,                 // Device name
	ISYST_BLUETOOTH_ID,     // PnP Bluetooth/USB vendor id
	1,                      // PnP Product ID
	0,						// Pnp prod version
	false,					// Enable device information service (DIS)
	NULL,
	(uint8_t*)&g_AdvDataBuff,   // Manufacture specific data to advertise
	sizeof(g_AdvDataBuff),      // Length of manufacture specific data
	BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	APP_ADV_INTERVAL,       // Advertising interval in msec
	APP_ADV_TIMEOUT_IN_SECONDS,	// Advertising timeout in sec
	100,                          // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	0,
	0,
	-1,    // Led port nuber
	-1,     // Led pin number
	NULL						// RTOS Softdevice handler
};

// Motsai Neblina V2 module uses SPI interface
#ifdef NEBLINA_MODULE
static const IOPINCFG gsSpiBoschPin[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP,
     IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_BME280_CS_PORT, SPI_BME280_CS_PIN, SPI_BME280_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const SPICFG s_SpiCfg = {
    SPI_DEVNO,
    SPIMODE_MASTER,
    gsSpiBoschPin,
    sizeof( gsSpiBoschPin ) / sizeof( IOPINCFG ),
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

SPI g_Spi;

DeviceIntrf *g_pIntrf = &g_Spi;

#else

// Configure I2C interface
static const I2CCFG s_I2cCfg = {
	0,			// I2C device number
	{

#ifdef TPH_BME280
		{I2C0_SDA_PORT, I2C0_SDA_PIN, I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
		{I2C0_SCL_PORT, I2C0_SCL_PIN, I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
#else
		{0, 4, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
		{0, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
#endif
	},
	100000,	// Rate
	I2CMODE_MASTER,
	0,			// Slave address
	5,			// Retry
	7,			// Interrupt prio
	NULL		// Event callback
};

// I2C interface instance
I2C g_I2c;

DeviceIntrf *g_pIntrf = &g_I2c;
#endif

// Configure environmental sensor
static TPHSENSOR_CFG s_TphSensorCfg = {
#ifdef NEBLINA_MODULE
    0,      // SPI CS index 0
#else
	BME280_I2C_DEV_ADDR0,   // I2C device address
#endif
	SENSOR_OPMODE_SINGLE,
	100,						// Sampling frequency in Hz
	1,
	1,
	1,
	0
};

// Environmental sensor instance
TphgBme680 g_Bme680Sensor;
TphBme280 g_Bme280Sensor;
TphMS8607 g_MS8607Sensor;


#ifdef TPH_BME280
TPHSensor &g_TphSensor = g_Bme280Sensor;
#elif defined(TPH_BME680)
TPHSensor &g_TphSensor = g_Bme680Sensor;
#else
TPHSensor &g_TphSensor = g_MS8607Sensor;
#endif

void ReadPTHData()
{
    g_pIntrf->Enable();

	TPHSENSOR_DATA data;

	g_TphSensor.Read(data);

	// NOTE : M0 does not access unaligned data
	// use local 4 bytes align stack variable then mem copy
	// skip timestamp as advertising pack is limited in size
	memcpy(&g_TPHData, ((uint8_t*)&data) + 4, sizeof(ENVSENSOR_DATA));

	// Update advertisement data
	BleAppAdvManDataSet(g_AdvDataBuff, sizeof(g_AdvDataBuff));

	g_pIntrf->Disable();
}

void TimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	ReadPTHData();
    }
}

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
#ifndef USE_TIMER_UPDATE
    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
    {
    	// Update environmental sensor data every time advertisement timeout
    	// for re-advertisement
    	ReadPTHData();
    }
#endif
}

void HardwareInit()
{
	// Set this only if nRF is power at 2V or more
	nrf_power_dcdcen_set(true);

	// Initialize I2C
#ifdef NEBLINA_MODULE
    g_Spi.Init(s_SpiCfg);
#else
    g_I2c.Init(s_I2cCfg);
#endif

	// Inititalize sensor
    g_TphSensor.Init(s_TphSensorCfg, g_pIntrf);

    // Update sensor data
    TPHSENSOR_DATA tphdata;
	g_TphSensor.Read(tphdata);

	// Do memcpy to adv data. Due to byte alignment, cannot read directly into
	// adv data
	memcpy(g_AdvData.Data, ((uint8_t*)&tphdata) + 4, sizeof(ENVSENSOR_DATA));

	g_pIntrf->Disable();

#ifdef USE_TIMER_UPDATE
	// Only with SDK14
    g_Timer.Init(s_TimerCfg);
	uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS);
#endif
}

int main()
{
    HardwareInit();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    BleAppRun();

	return 0;
}

