/**-------------------------------------------------------------------------
@example	TPHSensorTag.cpp

@brief	Environmental Sensor BLE demo (Supports BME280, BME680, MS8607).

This application demo shows BLE non-connectable using EHAL library. It advertises
Temperature, Pressure, Humidity (TPH) data in BLE manufacturer specific data.
Support I2C and SPI interface


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

#include <string.h>

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"

#include "bsec_interface.h"

#include "blueio_board.h"
#include "coredev/uart.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "tph_bme280.h"
#include "tph_ms8607.h"
#include "tphg_bme680.h"
#include "timer_nrf5x.h"
#ifdef NRF51
#include "timer_nrf_app_timer.h"
#endif
#include "board.h"
#include "idelay.h"
#include "adc_nrf52_saadc.h"

#define DEVICE_NAME                     "EnvSensorTag"                            /**< Name of device. Will be included in the advertising data. */

#ifdef NEBLINA_MODULE
#define TPH_BME280
#else
//#define TPH_BME280
#define TPH_BME680
#endif

//#ifdef NRF52
// Use timer to update data
// NOTE :	RTC timer 0 used by radio, RTC Timer 1 used by SDK
//			Only RTC timer 2 is usable with Softdevice for nRF52, not avail on nRF51
//
//#define USE_TIMER_UPDATE
//#endif

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(1000, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#ifdef USE_TIMER_UPDATE
// Use timer to update date
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */
#else
// Use advertisement timeout to update data
#define APP_ADV_TIMEOUT_IN_SECONDS      MSEC_TO_UNITS(120000, UNIT_10_MS)           /**< The advertising timeout (in units of seconds). */
#endif

void TimerHandler(Timer *pTimer, uint32_t Evt);

uint8_t g_AdvDataBuff[10] = {
	BLEADV_MANDATA_TYPE_TPH,
};

BLEADV_MANDATA &g_AdvData = *(BLEADV_MANDATA*)g_AdvDataBuff;

// Evironmental Sensor Data to advertise
BLEADV_MANDATA_TPHSENSOR &g_TPHData = *(BLEADV_MANDATA_TPHSENSOR *)g_AdvData.Data;
BLEADV_MANDATA_GASSENSOR &g_GasData = *(BLEADV_MANDATA_GASSENSOR *)g_AdvData.Data;
BLUEIO_DATA_BAT &g_AdvBat = *(BLUEIO_DATA_BAT *)g_AdvData.Data;

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 2,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = APP_IRQ_PRIORITY_LOW,
	.EvtHandler = TimerHandler
};

#ifdef NRF51
TimerAppTimer g_Timer;
#else
TimerLFnRF5x g_Timer;
#endif

const BLEAPP_CFG s_BleAppCfg = {
	{ // Clock config nrf_clock_lf_cfg_t
#ifdef IMM_NRF51822
		NRF_CLOCK_LF_SRC_RC,	// Source RC
		1, 1, 0
#else
		NRF_CLOCK_LF_SRC_XTAL,	// Source 32KHz XTAL
		0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM
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
	NULL,
	0,
	BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	APP_ADV_INTERVAL,       // Advertising interval in msec
	APP_ADV_TIMEOUT_IN_SECONDS,	// Advertising timeout in sec
	0,//MSEC_TO_UNITS(1000, UNIT_0_625_MS) ,   // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	0,
	0,
	-1,		// Led port nuber
	-1,     // Led pin number
	0, 		// Tx power
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
    {BME280_CS_PORT, BME280_CS_PIN, BME280_CS_PINOP,
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

#if defined(TPH_BME280) || defined(TPH_BME680)
		{I2C0_SDA_PORT, I2C0_SDA_PIN, I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{I2C0_SCL_PORT, I2C0_SCL_PIN, I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#else
		// Custom board with MS8607
		{0, 4, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{0, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#endif
	},
	100000,	// Rate
	I2CMODE_MASTER,
	5,			// Retry
	0,			// Number of slave addresses
	{0,},		// Slave addresses
	true,
	false,		// Use interrupt
	APP_IRQ_PRIORITY_LOW,			// Interrupt prio
	NULL		// Event callback
};

// I2C interface instance
I2C g_I2c;

DeviceIntrf *g_pIntrf = &g_I2c;
#endif

static const IOPINCFG s_GpioPins[] = {
	{BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, BLUEIO_BUT1_PINOP,	// Button 1
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP,	// Button 2
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP, 			// MPU9250 Interrupt
	 IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_LED2_RED_PORT, BLUEIO_TAG_EVIM_LED2_RED_PIN, BLUEIO_TAG_EVIM_LED2_RED_PINOP,		// RGB LED2 Red
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_LED2_GREEN_PORT, BLUEIO_TAG_EVIM_LED2_GREEN_PIN, BLUEIO_TAG_EVIM_LED2_GREEN_PINOP,	// RGB LED2 Green
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	{BLUEIO_TAG_EVIM_LED2_BLUE_PORT, BLUEIO_TAG_EVIM_LED2_BLUE_PIN, BLUEIO_TAG_EVIM_LED2_BLUE_PINOP,	// RGB LED2 Blue
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
//	    {BLUEIO_TAG_EVIM_SPI2_SCK_PORT, BLUEIO_TAG_EVIM_SPI2_SCK_PIN, BLUEIO_TAG_EVIM_SPI2_SCK_PINOP,
//	     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const int s_NbGpioPins = sizeof(s_GpioPins) / sizeof(IOPINCFG);

// Configure environmental sensor
static TPHSENSOR_CFG s_TphSensorCfg = {
#ifdef NEBLINA_MODULE
    0,      // SPI CS index 0 connected to BME280
#else
	BME680_I2C_DEV_ADDR0,   // I2C device address
#endif
	SENSOR_OPMODE_SINGLE,
	10,						// Sampling frequency in mHz
	2,
	1,
	1,
	1
};

static const GASSENSOR_HEAT s_HeaterProfile[] = {
	{ 375, 125 },
};

static const GASSENSOR_CFG s_GasSensorCfg = {
	BME680_I2C_DEV_ADDR0,	// Device address
	SENSOR_OPMODE_SINGLE,	// Operating mode
	10,
	sizeof(s_HeaterProfile) / sizeof(GASSENSOR_HEAT),
	s_HeaterProfile
};

// Environmental sensor instance
TphgBme680 g_Bme680Sensor;
TphBme280 g_Bme280Sensor;
TphMS8607 g_MS8607Sensor;


#ifdef TPH_BME280
TphSensor &g_TphSensor = g_Bme280Sensor;
#elif defined(TPH_BME680)
TphSensor &g_TphSensor = g_Bme680Sensor;
#else
TphSensor &g_TphSensor = g_MS8607Sensor;
#endif

GasSensor &g_GasSensor = g_Bme680Sensor;

// Define available voltage sources
static const ADC_REFVOLT s_RefVolt[] = {
	{.Type = ADC_REFVOLT_TYPE_INTERNAL, .Voltage = 0.6 },
};

static const int s_NbRefVolt = sizeof(s_RefVolt) / sizeof(ADC_REFVOLT);

#define ADC_CFIFO_SIZE		CFIFO_TOTAL_MEMSIZE(200, sizeof(ADC_DATA))

void ADCEventHandler(AdcDevice *pAdcDev, ADC_EVT Evt);

static uint8_t s_AdcFifoMem[ADC_CFIFO_SIZE];

// Define ADC device
static const ADC_CFG s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_RefVolt,
	.NbRefVolt = s_NbRefVolt,
	.DevAddr = 0,
	.Resolution = 10,
	.Rate = 8000,
	.OvrSample = 0,
	.bInterrupt = true,
	.IntPrio = 6,
	.EvtHandler = ADCEventHandler
};

AdcnRF52 g_Adc;

// Define ADC channel
static const ADC_CHAN_CFG s_ChanCfg[] = {
	{
		.Chan = 0,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 5,//1 << 8,
		.AcqTime = 10,
		.BurstMode = true,
		.PinP = { .PinNo = 8, .Conn = ADC_PIN_CONN_NONE },
	},
};

static const int s_NbChan = sizeof(s_ChanCfg) / sizeof(ADC_CHAN_CFG);
volatile bool g_bDataReady = false;
BLUEIO_DATA_BAT g_BatData;

void ADCEventHandler(AdcDevice *pAdcDev, ADC_EVT Evt)
{
	if (Evt == ADC_EVT_DATA_READY)
	{
		g_bDataReady = true;
		int cnt = 0;

		ADC_DATA df[s_NbChan];
		cnt = g_Adc.Read(df, s_NbChan);
		if (cnt > 0)
		{
//			g_Uart.printf("%d ADC[0] = %.2fV, ADC[1] = %.2fV, ADC[2] = %.2fV, ADC[3] = %.2fV\r\n",
//					df[0].Timestamp, df[0].Data, df[1].Data, df[2].Data, df[3].Data);

			uint8_t level = 100 * (df->Data - 1.75)/ 1.25;
			g_BatData.Level = level;
			g_BatData.Voltage = (int32_t)(df->Data * 1000.0);
		}

		g_Adc.Disable();
	}
}

void ReadPTHData()
{
	static uint32_t gascnt = 0;
	TPHSENSOR_DATA data;
	GASSENSOR_DATA gdata;
#if 1

	g_I2c.Enable();

	g_TphSensor.Read(data);



	if (g_TphSensor.DeviceID() == BME680_ID)// && (gascnt & 0x3) == 0)
	{
		g_GasSensor.Read(gdata);
	}

	if ((gascnt & 0x7) == 0)
	{
		g_Adc.Enable();
		g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
		g_Adc.StartConversion();

		g_AdvData.Type = BLEADV_MANDATA_TYPE_BAT;

		memcpy(&g_AdvBat, &g_BatData, sizeof(BLUEIO_DATA_BAT));
	}
	else if ((gascnt & 0x3) == 0)
	{
		BLEADV_MANDATA_GASSENSOR gas;

		g_AdvData.Type = BLEADV_MANDATA_TYPE_GAS;
		gas.GasRes = gdata.GasRes[gdata.MeasIdx];
		gas.AirQIdx = gdata.AirQualIdx;

		memcpy(&g_GasData, &gas, sizeof(BLEADV_MANDATA_GASSENSOR));
	}
	else
	{
		g_AdvData.Type = BLEADV_MANDATA_TYPE_TPH;

		// NOTE : M0 does not access unaligned data
		// use local 4 bytes align stack variable then mem copy
		// skip timestamp as advertising pack is limited in size
		memcpy(&g_TPHData, ((uint8_t*)&data) + sizeof(data.Timestamp), sizeof(BLEADV_MANDATA_TPHSENSOR));
	}

	g_TphSensor.StartSampling();

	g_I2c.Disable();

	g_Adc.Enable();
	g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
	g_Adc.StartConversion();


#endif
	// Update advertisement data
	BleAppAdvManDataSet(g_AdvDataBuff, sizeof(g_AdvDataBuff), NULL, 0);
//	BleAppAdvManDataSet((uint8_t*)&gascnt, sizeof(gascnt), NULL, 0);

	gascnt++;
}

void TimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// SDK15 no longer allow updating advertisement data dynamically
    	// Have to stop and restart advertisement
		//BleAppAdvStop();
    	ReadPTHData();
		//BleAppAdvStart(BLEAPP_ADVMODE_FAST);
    }
}

void AppTimerHandler(Timer *pTimer, int TrigNo, void *pContext)
{
	if (TrigNo == 0)
	{
		ReadPTHData();
//		app_sched_event_put(pContext, sizeof(uint32_t), SchedAdvData);
	}
}

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
#ifndef USE_TIMER_UPDATE
    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
    {
    	// Update environmental sensor data every time advertisement timeout
    	// for re-advertisement
   // 	ReadPTHData();
    }
#endif
}

void BleAppAdvTimeoutHandler()
{
	ReadPTHData();
	BleAppAdvStart(BLEAPP_ADVMODE_FAST);
}

void HardwareInit()
{
	// Set this only if nRF is power at 2V or more
	//nrf_power_dcdcen_set(true);
	NRF_POWER->DCDCEN = 1;

    IOPinCfg(s_GpioPins, s_NbGpioPins);

	IOPinSet(0, BLUEIO_TAG_BME680_LED2_BLUE_PIN);
	IOPinSet(0, BLUEIO_TAG_BME680_LED2_GREEN_PIN);
	IOPinSet(0, BLUEIO_TAG_BME680_LED2_RED_PIN);

	g_Timer.Init(s_TimerCfg);

	// Initialize I2C
#ifdef NEBLINA_MODULE
    g_Spi.Init(s_SpiCfg);
#else
    g_I2c.Init(s_I2cCfg);
#endif

	bsec_library_return_t bsec_status;

	// NOTE : For BME680 air quality calculation, this library is require to be initialized
	// before initializing the sensor driver.
	bsec_status = bsec_init();

	if (bsec_status != BSEC_OK)
	{
		printf("BSEC init failed\r\n");

		return;
	}

	// Inititalize sensor
    g_TphSensor.Init(s_TphSensorCfg, g_pIntrf, &g_Timer);

//    g_TphSensor.Disable();

//	g_I2c.Disable();

//	while(1) __WFE();


	if (g_TphSensor.DeviceID() == BME680_ID)
    {
    	g_GasSensor.Init(s_GasSensorCfg, g_pIntrf, NULL);
    }


    g_TphSensor.StartSampling();

	usDelay(300000);

    // Update sensor data
    TPHSENSOR_DATA tphdata;

    g_TphSensor.Read(tphdata);

    if (g_TphSensor.DeviceID() == BME680_ID)
    {
		GASSENSOR_DATA gdata;
		g_GasSensor.Read(gdata);
    }

	g_TphSensor.StartSampling();

	g_AdvData.Type = BLEADV_MANDATA_TYPE_TPH;
	// Do memcpy to adv data. Due to byte alignment, cannot read directly into
	// adv data
	memcpy(g_AdvData.Data, ((uint8_t*)&tphdata) + sizeof(tphdata.Timestamp), sizeof(BLEADV_MANDATA_TPHSENSOR));


	g_I2c.Disable();

	g_Adc.Init(s_AdcCfg);
	g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
	g_Adc.StartConversion();

#ifdef USE_TIMER_UPDATE
	// Only with SDK14

//	uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS);
#endif
}

int main()
{
    HardwareInit();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

	//uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS, AppTimerHandler);

    BleAppRun();

	return 0;
}

