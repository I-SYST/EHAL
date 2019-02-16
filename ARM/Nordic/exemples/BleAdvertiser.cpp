/**-------------------------------------------------------------------------
@example	BleAdvertiser.cpp

@brief	BLE non-connectable, advertising incremental counter. Counter increments every second.

@author	Hoang Nguyen Hoan
@date	Dec. 19, 2017

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

#include "app_util.h"

#include "istddef.h"
#include "ble_app.h"
#include "iopinctrl.h"

#define DEVICE_NAME                     "Advertiser"

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(500, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT_IN_SECONDS      MSEC_TO_UNITS(1000, UNIT_10_MS)

uint32_t g_AdvCnt = 0;

const BLEAPP_CFG s_BleAppCfg = {
	{ // Clock config nrf_clock_lf_cfg_t
#ifdef IMM_NRF51822
		NRF_CLOCK_LF_SRC_RC,	// Source RC
		1, 1, 0
#else
		NRF_CLOCK_LF_SRC_XTAL,	// Source 32KHz XTAL
#ifdef NRF51
		0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
#else
		0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM
#endif
#endif
	},
	0,						// Number of central link
	0,						// Number of peripheral link
	BLEAPP_MODE_NOCONNECT,	// Connectionless beacon type
	DEVICE_NAME,			// Device name
	ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	1,                      // PnP Product ID
	0,						// Pnp prod version
	false,					// Enable device information service (DIS)
	NULL,					// Pointer device info descriptor
	(uint8_t*)&g_AdvCnt,   	// Manufacture specific data to advertise
	sizeof(g_AdvCnt),      	// Length of manufacture specific data
	NULL,
	0,
	BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	APP_ADV_INTERVAL,       	// Advertising interval in msec
	APP_ADV_TIMEOUT_IN_SECONDS,	// Advertising timeout in sec
	0,							// Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	0,		// Min. connection interval
	0,		// Max. connection interval
	-1,		// Led port nuber
	-1,		// Led pin number
	0,
	0,		// Tx power
	NULL	// RTOS Softdevice handler
};

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
    {
    		// for re-advertisement
//    		g_AdvCnt++;

//    		BleAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt));
//    		BleAppAdvStart(BLEAPP_ADVMODE_FAST);
    }
}

void BleAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BleAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);
	BleAppAdvStart(BLEAPP_ADVMODE_FAST);
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
    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    BleAppRun();

	return 0;
}
