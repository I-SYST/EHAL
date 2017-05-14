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

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"
#include "blueio_board.h"
#include "uart.h"
#include "i2c.h"
#include "custom_board.h"
#include "iopincfg.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "pth_bme280.h"

#define DEVICE_NAME                     "PTHSensorTag"                            /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      60                                         /**< The advertising timeout (in units of seconds). */


PTHSENSOR_DATA g_PTHData;

const BLEAPP_CFG s_BleAppCfg = {
	{ // Clock config nrf_clock_lf_cfg_t
#ifdef IMM_NRF51822
		NRF_CLOCK_LF_SRC_RC,	// Source RC
		1, 1, 0
#else
		NRF_CLOCK_LF_SRC_XTAL,	// Source 32KHz XTAL
		0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
#endif

	},
	0, 						// Number of central link
	1, 						// Number of peripheral link
	BLEAPP_MODE_NOCONNECT,   // Connectionless beacon type
	DEVICE_NAME,                 // Device name
	ISYST_BLUETOOTH_ID,     // PnP Bluetooth/USB vendor id
	1,                      // PnP Product ID
	0,						// Pnp prod version
	false,					// Enable device information service (DIS)
	NULL,//&s_UartBleDevDesc,
	(uint8_t*)&g_PTHData,              // Manufacture specific data to advertise
	sizeof(g_PTHData),      // Length of manufacture specific data
	BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	APP_ADV_INTERVAL,       // Advertising interval in msec
	APP_ADV_TIMEOUT_IN_SECONDS,	// Advertising timeout in sec
	0,                          // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	0,
	0,
	-1,    // Led port nuber
	-1,     // Led pin number
	NULL						// RTOS Softdevice handler
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

I2C g_I2c;

static PTHSENSOR_CFG s_PthSensorCfg = {
	BME280_I2C_DEV_ADDR0,
	PTHSENSOR_OPMODE_SINGLE,
	0
};

// Environmental sensor
PthBme280 g_PthSensor;

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
    {
    	// Update environmental sensor data everytime advertisement timeout
    	// for re-advertisement
    	g_PthSensor.ReadPTH(g_PTHData);
    }
}


void HardwareInit()
{
	g_I2c.Init(s_I2cCfg);

    g_PthSensor.Init(s_PthSensorCfg, &g_I2c);

	g_PthSensor.ReadPTH(g_PTHData);
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

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    BleAppStart();

	return 0;
}
