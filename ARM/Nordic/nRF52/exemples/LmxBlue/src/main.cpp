//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2017, I-SYST
// Description : Hello World in C++
//============================================================================
#include <string.h>
#include <inttypes.h>

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "stddev.h"
#include "custom_board.h"
#include "lmx_blueio.h"
#include "ledmxio.h"
#include "ble_app.h"
#include "ble_service.h"
#include "bluetooth/blueio_blesrvc.h"

#define DEVICE_NAME                     "LmxBLue"                            /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME                      "LMX_DISPLAY"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                               /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#if (NRF_SD_BLE_API_VERSION < 6)
#define APP_ADV_TIMEOUT			      	0										/**< The advertising timeout (in units of seconds). */
#else
#define APP_ADV_TIMEOUT					MSEC_TO_UNITS(0, UNIT_10_MS)		/**< The advertising timeout (in units of 10ms seconds). */
#endif

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

void LmxBlueSrvcWrCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

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

BLESRVC_CHAR g_LmxChars[] = {
	{
		.Uuid = LMXBLUE_UUID_MSGCHAR,
		.MaxDataLen = 256,
		.Property = BLESVC_CHAR_PROP_WRITEWORESP,
		.pDesc = s_WdCharDescString,
		.WrCB = LmxBlueSrvcWrCallback
	}
};

const BLESRVC_CFG s_LmxBlueSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,
	.UuidBase = LMXBLUE_UUID_BASE,
	.UuidSvc = LMXBLUE_UUID_SERVICE,
	.NbChar = 1,
	.pCharArray = g_LmxChars
};

BLESRVC g_LmxBleSrvc;

const BLEAPP_DEVDESC s_LmxBlueDevDesc = {
	MODEL_NAME,       		// Model name
	MANUFACTURER_NAME,		// Manufacturer name
	"123",					// Serial number string
	"0.0",					// Firmware version string
	"0.0",					// Hardware version string
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
	.pDevName = "LMXBLUE",
	.VendorID = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.pDevDesc = &s_LmxBlueDevDesc,
	.pAdvManData = g_ManData,
	.AdvManDataLen = sizeof(g_ManData),
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLEAPP_SECTYPE_LESC_MITM,//BLEAPP_SECTYPE_STATICKEY_MITM,//BLEAPP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BLEAPP_SECEXCHG_NONE,	// Security key exchange
	.pAdvUuids = NULL,      			// Service uuids to advertise
	.NbAdvUuid = 0, 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,// Led pin number
	.TxPower = 0,						// Tx power
	.SDEvtHandler = NULL				// RTOS Softdevice handler
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

#define LMXBLUE_LINE_MAX			16

// Display board configuration
LEDMXCFG g_LmxCfg[LMXBLUE_LINE_MAX] = {
    {
	    &g_IOCfg,
		4,  // Number of display board in daisy chain, only one in this case
		{0, 1, 2, 3,}, // display board ordering
    },
    {
	    &g_IOCfg,
		4,  // Number of display board in daisy chain, only one in this case
		{4, 5, 6, 7,}, // display board ordering
    },
    {
	    &g_IOCfg,
		4,  // Number of display board in daisy chain, only one in this case
		{8, 9, 10, 11,}, // display board ordering
    },
    {
	    &g_IOCfg,
		4,  // Number of display board in daisy chain, only one in this case
		{12, 13, 14, 15,}, // display board ordering
    },
};

LEDMXDEV g_LmxDev[LMXBLUE_LINE_MAX] = {{0,},};

void LmxBlueSrvcWrCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	LMXMSG *msg = (LMXMSG*)pData;
	msg->Text[msg->Length] = 0;
	//LedMxPrintLeft(&g_LmxDev, msg->Text);
}

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
	BleSrvcEvtHandler(&g_LmxBleSrvc, p_ble_evt);
}

void BleAppInitUserServices()
{
    uint32_t       err_code;

    err_code = BleSrvcInit(&g_LmxBleSrvc, &s_LmxBlueSrvcCfg);
    APP_ERROR_CHECK(err_code);
}

void HardwareInit()
{
	// config Bluetooth connection state LED
    IOPinCfg(s_GpioPins, s_NbGpioPins);

	// Initialize IDM-LMX3208 series displays
	//LedMxInit(&g_LmxDev, &g_LmxCfg);

	//LedMxPrintLeft(&g_LmxDev, "IBB-LMXBLUE Blutooth LED matrix demo");
}

void BlePeriphAppInitUserData()
{

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

    BleAppInit(&s_BleAppCfg, true);

    BleAppRun();

	return 0;
}



