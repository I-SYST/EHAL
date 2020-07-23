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

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "bsec_interface.h"

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "sensors/tph_bme280.h"
#include "sensors/tph_ms8607.h"
#include "sensors/tphg_bme680.h"
#include "sensors/agm_mpu9250.h"
#include "sensors/accel_adxl362.h"
#include "timer_nrfx.h"
#include "board.h"
#include "idelay.h"
#include "seep.h"
#include "diskio_flash.h"

#include "BlueIOThingy.h"
#include "BlueIOMPU9250.h"

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

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
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




uint8_t g_AdvDataBuff[10] = {
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
TimerLFnRFx g_Timer;
//TimerAppTimer g_Timer;
#endif

static const ble_uuid_t  s_AdvUuids[] = {
    {BLE_UUID_TCS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};

const BLEAPP_CFG s_BleAppCfg = {
#ifdef IMM_NRF51822
		.ClkCfg = { NRF_CLOCK_LF_SRC_RC, 1, 1, 0},
#else
		.ClkCfg = { NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM},
#endif
	.CentLinkCount = 0, 				// Number of central link
	.PeriLinkCount = 1, 				// Number of peripheral link
	.AppMode = BLEAPP_MODE_APPSCHED,	// Use scheduler
	.pDevName = DEVICE_NAME,			// Device name
	.VendorID = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.bEnDevInfoService = false,			// Enable device information service (DIS)
	.pDevDesc = NULL,
	.pAdvManData = g_AdvDataBuff,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_AdvDataBuff),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLEAPP_SECTYPE_NONE,//BLEAPP_SECTYPE_STATICKEY_MITM,//BLEAPP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BLEAPP_SECEXCHG_NONE,	// Security key exchange
	.pAdvUuids = s_AdvUuids,      			// Service uuids to advertise
	.NbAdvUuid = sizeof(s_AdvUuids) / sizeof(ble_uuid_t), 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = 0,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,// Led pin number
	.ConnLedActLevel = 0,
	.TxPower = 0,						// Tx power
	.SDEvtHandler = NULL,				// RTOS Softdevice handler
	.MaxMtu = 53,
};

static const IOPINCFG s_GpioPins[] = {
    {BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, BLUEIO_BUT1_PINOP,
     IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP,
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
    {BLUEIO_LED1_PORT, BLUEIO_LED1_PIN, BLUEIO_LED1_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_LED2_RED_PORT, BLUEIO_TAG_EVIM_LED2_RED_PIN, BLUEIO_TAG_EVIM_LED2_RED_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_LED2_GREEN_PORT, BLUEIO_TAG_EVIM_LED2_GREEN_PIN, BLUEIO_TAG_EVIM_LED2_GREEN_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_LED2_BLUE_PORT, BLUEIO_TAG_EVIM_LED2_BLUE_PIN, BLUEIO_TAG_EVIM_LED2_BLUE_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_EEP_WP_PORT, BLUEIO_TAG_EVIM_EEP_WP_PIN, BLUEIO_TAG_EVIM_EEP_WP_PINOP,				// EEP WP
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_BUZZ_PORT, BLUEIO_TAG_EVIM_BUZZ_PIN, BLUEIO_TAG_EVIM_BUZZ_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},
};

static const int s_NbGpioPins = sizeof(s_GpioPins) / sizeof(IOPINCFG);

static const IOPINCFG s_SpiPins[] = {
    {SPI2_SCK_PORT, SPI2_SCK_PIN, SPI2_SCK_PINOP,
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI2_MISO_PORT, SPI2_MISO_PIN, SPI2_MISO_PINOP,
		IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI2_MOSI_PORT, SPI2_MOSI_PIN, SPI2_MOSI_PINOP,
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {BLUEIO_TAG_EVIM_IMU_CS_PORT, BLUEIO_TAG_EVIM_IMU_CS_PIN, BLUEIO_TAG_EVIM_IMU_CS_PINOP,
		IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_FLASH_CS_PORT, BLUEIO_TAG_EVIM_FLASH_CS_PIN, BLUEIO_TAG_EVIM_FLASH_CS_PINOP,
		IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// CS
};

static const SPICFG s_SpiCfg = {
    .DevNo = SPI2_DEVNO,
    .Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
    .pIOPinMap = s_SpiPins,
    .NbIOPins = sizeof(s_SpiPins) / sizeof(IOPINCFG),
    .Rate = 1000000,   // Speed in Hz
    .DataSize = 8,      // Data Size
    .MaxRetry = 5,      // Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_SECOND_CLK, // Data phase
    .ClkPol = SPICLKPOL_LOW,         // clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,
	.bIntEn = false,
    .IntPrio = APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

//DeviceIntrf *g_pIntrf = &g_Spi;

// Configure I2C interface
static const I2CCFG s_I2cCfg = {
	.DevNo = 0,			// I2C device number
	.Pins = {
#if defined(TPH_BME280) || defined(TPH_BME680)
		{I2C0_SDA_PORT, I2C0_SDA_PIN, I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{I2C0_SCL_PORT, I2C0_SCL_PIN, I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#else
		// Custom board with MS8607
		{0, 4, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{0, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#endif
	},
	.Rate = 100000,		// Rate
	.Mode = I2CMODE_MASTER,
	.MaxRetry = 5,			// Retry
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},		// Slave addresses
	.bDmaEn = true,
	.bIntEn = false,		// use interrupt
	.IntPrio = APP_IRQ_PRIORITY_LOW,// Interrupt prio
	.EvtCB = NULL		// Event callback
};

// I2C interface instance
I2C g_I2c;

//DeviceIntrf *g_pIntrf = &g_I2c;

// Configure environmental sensor
static TPHSENSOR_CFG s_TphSensorCfg = {
#ifdef NEBLINA_MODULE
	.DevAddr = 0,      // SPI CS index 0 connected to BME280
#else
	.DevAddr = BME680_I2C_DEV_ADDR0,   // I2C device address
#endif
	.OpMode = SENSOR_OPMODE_SINGLE,
	.Freq = 100,						// Sampling frequency in mHz
	.TempOvrs = 1,
	.PresOvrs = 1,
	.HumOvrs = 1,
	.FilterCoeff = 1,
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

static const ACCELSENSOR_CFG s_AccelCfg = {
	.DevAddr = 0,	// SPI CS idx
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,	// 50Hz (in mHz)
	.Scale = 2,
	.bInter = true,
	.IntPol = DEVINTR_POL_LOW,
};


//AgmMpu9250 g_AgmSensor;
AccelAdxl362 g_AccelSensor;

static const SEEP_CFG s_SeepCfg = {
	.DevAddr = 0x50,
	.AddrLen = 2,
	.PageSize = 32,
	.Size = 128 * 32,
	.WrDelay = 5,
	.WrProtPin = {-1, -1,},
};

Seep g_Seep;

FlashDiskIO g_FlashDiskIO;

static uint8_t s_FlashCacheMem[DISKIO_SECT_SIZE];
DISKIO_CACHE_DESC g_FlashCache = {
    -1, 0xFFFFFFFF, s_FlashCacheMem
};

bool MX25U1635E_init(int pDevNo, DeviceIntrf* ppInterface);

static FLASHDISKIO_CFG s_FlashDiskCfg = {
    .DevNo = 1,
    .TotalSize = 16 * 1024 * 1024 / 8,      // 256 Mbits
    .SectSize = 4,
	.BlkSize = 32,
    .WriteSize = 128,
    .AddrSize = 3,                          // 256+ Mbits needs 4 bytes addressing
    .pInitCB = MX25U1635E_init,//mx66u51235f_init,
    .pWaitCB = NULL,//FlashWriteDelayCallback,
};

bool FlashWriteDelayCallback(int DevNo, DeviceIntrf *pInterf)
{
	return true;
}

bool MX25U1635E_init(int DevNo, DeviceIntrf* pInterface)
{
    if (pInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

    d = FLASH_CMD_READID;
    cnt = pInterface->Read(DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 3);
    if ( r != 0x25C2 )
    	return false;

    // Enable write
    d = FLASH_CMD_EN4B;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    return true;
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
		memcpy(&g_TPHData, ((uint8_t*)&data) + sizeof(data.Timestamp), sizeof(BLEADV_MANDATA_TPHSENSOR));
	}


	g_TphSensor.StartSampling();

	// Update advertisement data
	BleAppAdvManDataSet(g_AdvDataBuff, sizeof(g_AdvDataBuff), NULL, 0);

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

	BleSrvcEvtHandler(GetConfSrvcInstance(), p_ble_evt);
	BleSrvcEvtHandler(GetUISrvcInstance(), p_ble_evt);
    BleSrvcEvtHandler(GetEnvSrvcInstance(), p_ble_evt);
    BleSrvcEvtHandler(GetImuSrvcInstance(), p_ble_evt);
}

/// Initialize all services needed for this firmware
void BleAppInitUserServices()
{
    uint32_t res = 0;

    res = ConfSrvcInit();
    res = UISrvcInit();
    res = EnvSrvcInit();
    res = ImuSrvcInit();
}

void FlashTest()
{
	g_FlashDiskIO.Init(s_FlashDiskCfg, &g_Spi, &g_FlashCache, 1);

	uint8_t buff[512];
	uint8_t tmp[512];
	uint16_t *p = (uint16_t*)buff;

	memset(tmp, 0, 512);
	for (int i = 0; i < 256; i++)
	{
		p[i] = i;
	}


	printf("Erasing... Please wait\r\n");

	// Ease could take a few minutes
	g_FlashDiskIO.Erase();

	printf("Writing 2KB data...\r\n");

	g_FlashDiskIO.SectWrite(0, buff);
	g_FlashDiskIO.SectWrite(2, buff);
	g_FlashDiskIO.SectWrite(4, buff);
	g_FlashDiskIO.SectWrite(8, buff);

	printf("Validate readback...\r\n");

	g_FlashDiskIO.SectRead(0, tmp);

	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 0 verify failed\r\n");
	}
	else
	{
		printf("Sector 0 verify success\r\n");
	}

	memset(tmp, 0, 512);
	g_FlashDiskIO.SectRead(2, tmp);
	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 2 verify failed\r\n");
	}
	else
	{
		printf("Sector 2 verify success\r\n");
	}

	memset(tmp, 0, 512);
	g_FlashDiskIO.SectRead(4, tmp);
	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 4 verify failed\r\n");
	}
	else
	{
		printf("Sector 4 verify success\r\n");
	}

	memset(tmp, 0, 512);
	g_FlashDiskIO.SectRead(8, tmp);
	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 8 verify failed\r\n");
	}
	else
	{
		printf("Sector 8 verify success\r\n");
	}
}

void HardwareInit()
{
	// Set this only if nRF is power at 2V or more
	NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos;

	IOPinCfg(s_GpioPins, s_NbGpioPins);

	// Turn off all LEDs
	IOPinSet(BLUEIO_LED1_PORT, BLUEIO_LED1_PIN);
	IOPinClear(BLUEIO_TAG_EVIM_LED2_RED_PORT, BLUEIO_TAG_EVIM_LED2_RED_PIN);
	IOPinClear(BLUEIO_TAG_EVIM_LED2_GREEN_PORT, BLUEIO_TAG_EVIM_LED2_GREEN_PIN);
	IOPinClear(BLUEIO_TAG_EVIM_LED2_BLUE_PORT, BLUEIO_TAG_EVIM_LED2_BLUE_PIN);

	//IOPinSet(BLUEIO_TAG_EVIM_LED2_RED_PORT, BLUEIO_TAG_EVIM_LED2_RED_PIN);
	//IOPinSet(BLUEIO_TAG_EVIM_LED2_GREEN_PORT, BLUEIO_TAG_EVIM_LED2_GREEN_PIN);
	//IOPinSet(BLUEIO_TAG_EVIM_LED2_BLUE_PORT, BLUEIO_TAG_EVIM_LED2_BLUE_PIN);

    g_Timer.Init(s_TimerCfg);

    g_Spi.Init(s_SpiCfg);


    g_I2c.Init(s_I2cCfg);

    g_Seep.Init(s_SeepCfg, &g_I2c);

    uint8_t d = 0xa5;
    g_Seep.Write(0, &d, 1);

    d = 0;
    g_Seep.Read(0, &d, 1);

    printf("%d\r\n", d);

    uint8_t reg[2] = { 0xb, 0};
    uint8_t val[2] = {0, };

    g_Spi.Read(0, reg, 2, val, 2);

    if (MPU9250Init(&g_Spi, &g_Timer) == true)
    {

    }
    else
    {
    	if (ICM20948Init(&g_Spi, &g_Timer) == true)
    	{

    	}
    }

//    g_AgmSensor.Init(s_AccelCfg, &g_Spi);
    //g_AccelSensor.Init(s_AccelCfg, &g_Spi);

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
    g_TphSensor.Init(s_TphSensorCfg, &g_I2c, &g_Timer);

    if (g_TphSensor.DeviceID() == BME680_ID)
    {
		g_GasSensor.Init(s_GasSensorCfg, &g_I2c, NULL);
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

    //RETURN_IF_ERROR(err_code);

}

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
    HardwareInit();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

	uint32_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS, AppTimerHandler);

    BleAppRun();

	return 0;
}

extern "C" int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
	return 0;
}
