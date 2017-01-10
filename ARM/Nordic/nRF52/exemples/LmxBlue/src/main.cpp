//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2017, I-SYST
// Description : Hello World in C++
//============================================================================
#include <string.h>
#include <inttypes.h>

#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_dfu_settings.h"
#include "ble_dis.h"

#include "istddef.h"
#include "iopincfg.h"
#include "iopinctrl.h"
#include "stddev.h"
#include "custom_board.h"
#include "lmx_blueio.h"
#include "blueio_blesrvc.h"
#include "ledmxio.h"
#include "ble_periph_app.h"

#define DEVICE_NAME                     "LmxBLue"                            /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "LMX_DISPLAY"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                               /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

void LmxBlueSrvcWrCallback(BLUEIOSRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

static const IOPINCFG s_GpioPins[] = {
	{BLUEIO_CONNECT_LED_PORT, BLUEIO_CONNECT_LED_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// BLE connect
};

static const int s_NbGpioPins = sizeof(s_GpioPins) / sizeof(IOPINCFG);

static const ble_uuid_t  s_AdvUuids[] = {
	{LMXBLUE_UUID_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};

static const char s_RdCharDescString[] = {
		"LmxBlue data characteristic",
};

static const char s_WdCharDescString[] = {
		"LmxBlue write characteristic",
};

uint8_t g_ManData[8];


const BLUEIOSRVC_CFG s_LmxBlueSrvcCfg = {
	BLUEIOSRVC_SECTYPE_NONE,
	LMXBLUE_UUID_BASE,
	LMXBLUE_UUID_SERVICE,
	LMXBLUE_UUID_RDCHAR,
	20, BLUEIOSVC_CHAR_PROP_READ | BLUEIOSVC_CHAR_PROP_NOTIFY,
	s_RdCharDescString,
	LMXBLUE_UUID_WRCHAR,
	20, BLUEIOSVC_CHAR_PROP_WRITEWORESP,
	s_WdCharDescString,
	LmxBlueSrvcWrCallback
};

BLUEIOSRVC g_LmxBleSrvc;

const BLEAPP_CFG s_BleAppCfg = {
	BLEAPP_MODE_APPSCHED,
	"LMXBLUE",
	"IBB-LMXBLUE",
	"I-SYST inc.",
	ISYST_BLUETOOTH_ID,
	g_ManData,
	sizeof(g_ManData),
	BLEAPP_SECTYPE_NONE,
	BLEAPP_SECEXCHG_NONE,
	s_AdvUuids,
	sizeof(s_AdvUuids) / sizeof(ble_uuid_t),
	APP_ADV_INTERVAL,
	APP_ADV_TIMEOUT_IN_SECONDS,
	BLUEIO_CONNECT_LED_PORT,
	BLUEIO_CONNECT_LED_PIN
};

// I/O pins connection
LEDMXIOCFG g_IOCfg = {
	LMXBLUE_WR_PIN,  // WR pin
	LMXBLUE_RD_PIN,  // RD pin
	LMXBLUE_DATA_PIN,  // Data pin
	LMXBLUE_EN_PIN, // En pin
	{ LMXBLUE_AD0_PIN, LMXBLUE_AD1_PIN, LMXBLUE_AD2_PIN, LMXBLUE_AD3_PIN,}, // CS pins
	4,  // Number of CS pins
	LMXBLUE_CSTYPE
};

// Display board configuration
LEDMXCFG g_LmxCfg = {
  &g_IOCfg,
  8,  // Number of display board in daisy chain, only one in this case
  {0, 1, 2, 3, 4, 5, 6, 7}, // display board ordering
};

LEDMXDEV g_LmxDev = {0,};

void LmxBlueSrvcWrCallback(BLUEIOSRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{

}

void BlePeriphAppEvtDispatch(ble_evt_t * p_ble_evt)
{
    BlueIOBleSvcEvtHandler(&g_LmxBleSrvc, p_ble_evt);
}

void BlePeriphAppInitUserStorage()
{
}

void BlePeriphAppInitServices()
{
    uint32_t       err_code;

    err_code = BlueIOBleSrvcInit(&g_LmxBleSrvc, &s_LmxBlueSrvcCfg);
    APP_ERROR_CHECK(err_code);
}

void HardwareInit()
{
	// config Bluetooth connection state LED
    IOPinCfg(s_GpioPins, s_NbGpioPins);

	// Initialize IDM-LMX3208 series displays
	LedMxInit(&g_LmxDev, &g_LmxCfg);

	LedMxPrintLeft(&g_LmxDev, "IBB-LMXBLUE Blutooth LED matrix demo");
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

    BlePeriphAppInit(&s_BleAppCfg, true);

    // This function will not return
    BlePeriphAppRun();

	return 0;
}



