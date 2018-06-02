/**-------------------------------------------------------------------------
@example	BlueIOThingy.cpp

@brief	Environmental Sensor BLE demo (Supports BME280, BME680, MS8607).

This application demo shows Thingy compatible environmental sensor using EHAL library.
It sends Temperature, Pressure, Humidity (TPH) data in Thingy compatible format.
Support I2C and SPI interface


NOTE : The BME680 Air Quality Index is undocumented.  It requires the library
Bosch Sensortec Environmental Cluster (BSEC) Software. Download from
https://www.bosch-sensortec.com/bst/products/all_products/bsec and put in
external folder as indicated on the folder tree.

The BSEC library must be initialized in the main application prior to initializing
this driver by calling the function

bsec_library_return_t res = bsec_init();

@author Hoang Nguyen Hoan
@date	Jan 24, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"

#include "bsec_interface.h"

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
#include "timer_nrf_app_timer.h"
#include "board.h"
#include "idelay.h"

#define DEVICE_NAME                     "BlueIOThingy"                            /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */

#ifdef NRF52
#define MODEL_NAME                      "IMM-NRF52x"                        /**< Model number. Will be passed to Device Information Service. */
#else
#define MODEL_NAME                      "IMM-NRF51x"                        /**< Model number. Will be passed to Device Information Service. */
#endif

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(200, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
// Use advertisement timeout to update data
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#ifdef NEBLINA_MODULE
#define TPH_BME280
#else
#define TPH_BME680
#endif

//#ifdef NRF52
// Use timer to update data
// NOTE :	RTC timer 0 used by radio, RTC Timer 1 used by SDK
//			Only RTC timer 2 is usable with Softdevice for nRF52, not avail on nRF51
//
#define USE_TIMER_UPDATE
//#endif

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(200, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#ifdef USE_TIMER_UPDATE
// Use timer to update date
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */
#else
// Use advertisement timeout to update data
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */
#endif

/// Thingy custom service UUID
#define THINGY_BASE_UUID    {{0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF}} //!< Used vendor specific UUID

// TCS (Configuration service characteristics)

#define BLE_UUID_TCS_SERVICE				0x0100                      /**< The UUID of the Thingy Configuration Service. */

#define BLE_UUID_TCS_DEVICE_NAME_CHAR	0x0101                      /**< The UUID of the device name Characteristic. */
#define BLE_UUID_TCS_ADV_PARAMS_CHAR		0x0102                      /**< The UUID of the advertising parameters Characteristic. */
#define BLE_UUID_TCS_APPEARANCE_CHAR		0x0103                      /**< The UUID of the appearance Characteristic. */
#define BLE_UUID_TCS_CONN_PARAM_CHAR		0x0104                      /**< The UUID of the connection parameters Characteristic. */
#define BLE_UUID_TCS_BEACON_PARAM_CHAR	0x0105                      /**< The UUID of the beacon Characteristic. */
#define BLE_UUID_TCS_CLOUD_PARAM_CHAR	0x0106                      /**< The UUID of the cloud token Characteristic. */
#define BLE_UUID_TCS_FW_VERSION_CHAR		0x0107                      /**< The UUID of the FW version Characteristic. */
#define BLE_UUID_TCS_MTU_CHAR			0x0108                      /**< The UUID of the MTU Characteristic. */

#define THINGY_TCS_FW_VERSIO_CHAR_IDX   	0//5

#define BLE_TCS_MAX_DATA_LEN (NRF_BLE_MAX_MTU_SIZE - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Configuration service module. */

#pragma pack(push, 1)
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} ble_tcs_fw_version_t;
#pragma pack(pop)

const ble_tcs_fw_version_t s_ThingyVersion {
	 2, 1, 0
};

BLESRVC_CHAR g_ConfChars[] = {
    {
        // Version characteristic.  This is the minimum required for Thingy App to work
		BLE_UUID_TCS_FW_VERSION_CHAR,
        BLE_TCS_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_VARLEN,
        NULL,    					// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
		(uint8_t*)&s_ThingyVersion,                       // pointer to char default values
        3,                          // Default value length in bytes
    },
};

/// Service definition
const BLESRVC_CFG s_ConfSrvcCfg = {
    BLESRVC_SECTYPE_NONE,       	// Secure or Open service/char
    THINGY_BASE_UUID,           	// Base UUID
    BLE_UUID_TCS_SERVICE,       	// Service UUID
    sizeof(g_ConfChars) / sizeof(BLESRVC_CHAR),  // Total number of characteristics for the service
    g_ConfChars,                 	// Pointer a an array of characteristic
    NULL,                       	// pointer to user long write buffer
    0,                           	// long write buffer size
	NULL,							// Authentication event callback
};

/// TCS instance
BLESRVC g_ConfSrvc;

#define BLE_UUID_TES_SERVICE          	0x0200

#define BLE_UUID_TES_TEMPERATURE_CHAR	0x0201                      /**< The UUID of the temperature Characteristic. */
#define BLE_UUID_TES_PRESSURE_CHAR    	0x0202                      /**< The UUID of the pressure Characteristic. */
#define BLE_UUID_TES_HUMIDITY_CHAR    	0x0203                      /**< The UUID of the humidity Characteristic. */
#define BLE_UUID_TES_GAS_CHAR         	0x0204                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_COLOR_CHAR       	0x0205                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_CONFIG_CHAR      	0x0206                      /**< The UUID of the config Characteristic. */

//#define THINGY_TES_CONFIGCHAR_IDX   0
#define THINGY_TES_TEMPCHAR_IDX     0
#define THINGY_TES_PRESCHAR_IDX     1
#define THINGY_TES_HUMICHAR_IDX     2

#define BLE_TES_MAX_DATA_LEN (NRF_BLE_MAX_MTU_SIZE - 3)       /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Environment service module. */

#pragma pack(push, 1)
typedef struct {
    int8_t  integer;
    uint8_t decimal;
} ble_tes_temperature_t;

typedef struct {
    int32_t  integer;
    uint8_t  decimal;
} ble_tes_pressure_t;

typedef struct {
    uint8_t  led_red;
    uint8_t  led_green;
    uint8_t  led_blue;
} ble_tes_color_config_t;

#pragma pack(pop)

static const char s_EnvTempCharDescString[] = {
        "Temperature characteristic",
};

static const char s_EnvPresCharDescString[] = {
        "Pressure characteristic",
};

static const char s_EnvHumCharDescString[] = {
        "Humidity characteristic",
};

/// Characteristic definitions
BLESRVC_CHAR g_EnvChars[] = {
    {
        // Temperature characteristic
        BLE_UUID_TES_TEMPERATURE_CHAR,
        BLE_TES_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
        s_EnvTempCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0,                          // Default value length in bytes
    },
    {
        // Pressure characteristic
        BLE_UUID_TES_PRESSURE_CHAR, // char UUID
        BLE_TES_MAX_DATA_LEN,       // char max data length
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
        s_EnvPresCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Humidity characteristic
        BLE_UUID_TES_HUMIDITY_CHAR, // char UUID
        BLE_TES_MAX_DATA_LEN,       // char max data length
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
        s_EnvHumCharDescString,     // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0                           // Default value length in bytes
    },
};

/// Service definition
const BLESRVC_CFG s_EnvSrvcCfg = {
    BLESRVC_SECTYPE_NONE,       // Secure or Open service/char
    THINGY_BASE_UUID,           // Base UUID
    BLE_UUID_TES_SERVICE,       // Service UUID
    sizeof(g_EnvChars) / sizeof(BLESRVC_CHAR),  // Total number of characteristics for the service
    g_EnvChars,                 // Pointer a an array of characteristic
    NULL,                       // pointer to user long write buffer
    0,                          // long write buffer size
	NULL
};

BLESRVC g_EnvSrvc;


uint8_t g_AdvDataBuff[9] = {
	BLEADV_MANDATA_TYPE_TPH,
};

BLEADV_MANDATA &g_AdvData = *(BLEADV_MANDATA*)g_AdvDataBuff;

// Evironmental Sensor Data to advertise
BLEADV_MANDATA_TPHSENSOR &g_TPHData = *(BLEADV_MANDATA_TPHSENSOR *)g_AdvData.Data;
BLEADV_MANDATA_GASSENSOR &g_GasData = *(BLEADV_MANDATA_GASSENSOR *)g_AdvData.Data;

void TimerHandler(Timer *pTimer, uint32_t Evt);

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 2,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = APP_IRQ_PRIORITY_LOW,
	.EvtHandler = NULL,//TimerHandler
};

#ifdef NRF51
TimerAppTimer g_Timer;
#else
TimerLFnRF5x g_Timer;
//TimerAppTimer g_Timer;
#endif

static const ble_uuid_t  s_AdvUuids[] = {
    {BLE_UUID_TCS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};


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
	1, 						// Number of peripheral link
	BLEAPP_MODE_APPSCHED,   	// Connectionless beacon type
	DEVICE_NAME,         	// Device name
	ISYST_BLUETOOTH_ID,    	// PnP Bluetooth/USB vendor id
	1,                     	// PnP Product ID
	0,						// Pnp prod version
	false,					// Enable device information service (DIS)
	NULL,
	(uint8_t*)&g_AdvDataBuff,   // Manufacture specific data to advertise
	sizeof(g_AdvDataBuff),      // Length of manufacture specific data
	BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	s_AdvUuids,      				// Service uuids to advertise
	sizeof(s_AdvUuids) / sizeof(ble_uuid_t), 						// Total number of uuids
	APP_ADV_INTERVAL,       // Advertising interval in msec
	APP_ADV_TIMEOUT_IN_SECONDS,	// Advertising timeout in sec
	0,                          // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	MIN_CONN_INTERVAL,
	MAX_CONN_INTERVAL,
	-1,		// Led port nuber
	-1,     // Led pin number
	0, 		// Tx power
	NULL,						// RTOS Softdevice handler
	276,
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
    0,      // SPI CS index 0 connected to BME280
#else
	BME680_I2C_DEV_ADDR0,   // I2C device address
#endif
	SENSOR_OPMODE_SINGLE,
	100,						// Sampling frequency in mHz
	1,
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
	500,
	sizeof(s_HeaterProfile) / sizeof(GASSENSOR_HEAT),
	s_HeaterProfile
};

// Environmental sensor instance
TphgBme680 g_Bme680Sensor;
TphBme280 g_Bme280Sensor;

#ifdef TPH_BME280
TphSensor &g_TphSensor = g_Bme280Sensor;
#elif defined(TPH_BME680)
TphSensor &g_TphSensor = g_Bme680Sensor;
#else
TphSensor &g_TphSensor = g_MS8607Sensor;
#endif

GasSensor &g_GasSensor = g_Bme680Sensor;

void EnvSrvcNotifTemp(float Temp)
{
    ble_tes_temperature_t t;


    t.integer = (int)Temp;
    t.decimal = (uint8_t)((Temp - (float)t.integer) * 100.0);

	BleSrvcCharNotify(&g_EnvSrvc, THINGY_TES_TEMPCHAR_IDX, (uint8_t*)&t, sizeof(t));
}

void EnvSrvcNotifPressure(float Press)
{
    ble_tes_pressure_t b;

    b.integer = (int)Press;
    b.decimal = (uint8_t)((Press - (float)b.integer) * 100.0);

	BleSrvcCharNotify(&g_EnvSrvc, THINGY_TES_PRESCHAR_IDX, (uint8_t*)&b, sizeof(b));
}

void EnvSrvcNotifHumi(uint8_t Humi)
{
	BleSrvcCharNotify(&g_EnvSrvc, THINGY_TES_HUMICHAR_IDX, &Humi, sizeof(Humi));
}

void ReadPTHData()
{
	static uint32_t gascnt = 0;
	TPHSENSOR_DATA data;

	g_TphSensor.Read(data);



	if (g_TphSensor.DeviceID() == BME680_ID && (gascnt & 0x3) == 0)
	{
		GASSENSOR_DATA gdata;
		BLEADV_MANDATA_GASSENSOR gas;

		g_GasSensor.Read(gdata);

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
		memcpy(&g_TPHData, ((uint8_t*)&data) + 8, sizeof(BLEADV_MANDATA_TPHSENSOR));
	}


	g_TphSensor.StartSampling();

	// Update advertisement data
	BleAppAdvManDataSet(g_AdvDataBuff, sizeof(g_AdvDataBuff));

	EnvSrvcNotifTemp((float)data.Temperature / 100.0);

	EnvSrvcNotifPressure((float)data.Pressure / 100.0);

	EnvSrvcNotifHumi(data.Humidity / 100);


	gascnt++;
}

void SchedAdvData(void * p_event_data, uint16_t event_size)
{
	ReadPTHData();
}

void AppTimerHandler(Timer *pTimer, int TrigNo, void *pContext)
{
	if (TrigNo == 0)
	{
		app_sched_event_put(pContext, sizeof(uint32_t), SchedAdvData);
	}
}

void TimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
		// NOTE : Use app_sched is needed as Softdevice will crash if called directly
		app_sched_event_put(&Evt, sizeof(uint32_t), SchedAdvData);
    }
}

/// BLE event handler.  Need this to handle events for the services
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

	BleSrvcEvtHandler(&g_ConfSrvc, p_ble_evt);
    BleSrvcEvtHandler(&g_EnvSrvc, p_ble_evt);

}

/// Initialize all services needed for this firmware
void BleAppInitUserServices()
{
    uint32_t res = 0;

    res = BleSrvcInit(&g_ConfSrvc, &s_ConfSrvcCfg);
    res = BleSrvcInit(&g_EnvSrvc, &s_EnvSrvcCfg);
}

void HardwareInit()
{
	// Set this only if nRF is power at 2V or more
	NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos;

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

    if (g_TphSensor.DeviceID() == BME680_ID)
    {
    		g_GasSensor.Init(s_GasSensorCfg, g_pIntrf, NULL);
    }

	g_TphSensor.StartSampling();

	usDelay(200000);

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
	memcpy(g_AdvData.Data, ((uint8_t*)&tphdata) + 4, sizeof(BLEADV_MANDATA_TPHSENSOR));

//#ifdef USE_TIMER_UPDATE
	// Only with SDK14
//	uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS, AppTimerHandler);
//#endif
}

int main()
{
    HardwareInit();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

	uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS, AppTimerHandler);

    BleAppRun();

	return 0;
}

