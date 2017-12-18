//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright (c) 2017, I-SYST
// Description : Hello World in C++
//============================================================================
#include "app_util.h"

#include "istddef.h"
#include "ble_app.h"

#define DEVICE_NAME                     "Advertiser"

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT_IN_SECONDS      1

uint32_t g_AdvCnt = 0;

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
	(uint8_t*)&g_AdvCnt,   // Manufacture specific data to advertise
	sizeof(g_AdvCnt),      // Length of manufacture specific data
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
	0, 		// Tx power
	NULL						// RTOS Softdevice handler
};

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
    {
    	// for re-advertisement
    	g_AdvCnt++;
    	BleAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt));
    }
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
    //HardwareInit();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    BleAppRun();

	return 0;
}
