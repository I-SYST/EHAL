/**-------------------------------------------------------------------------
@example	UartBleDemo.cpp


@brief	Uart BLE demo

This application demo shows UART Rx/Tx over BLE custom service using EHAL library.
For evaluating power consumption of the UART, the button 1 is used to enable/disable it.
This example also demonstrates passkey paring mode.

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017

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

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"
#include "blueio_board.h"
#include "uart.h"
#include "custom_board.h"
#include "iopincfg.h"
#include "iopinctrl.h"

#include "board.h"

#define DEVICE_NAME                     "UARTDemo"                          /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */

#ifdef NRF52
#define MODEL_NAME                      "IMM-NRF52x"                        /**< Model number. Will be passed to Device Information Service. */
#else
#define MODEL_NAME                      "IMM-NRF51x"                        /**< Model number. Will be passed to Device Information Service. */
#endif

#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                  /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                  /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)	/**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                 /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

static const ble_uuid_t  s_AdvUuids[] = {
	{BLUEIO_UUID_UART_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
};

static const char s_RxCharDescString[] = {
		"UART Rx characteristic",
};

static const char s_TxCharDescString[] = {
		"UART Tx characteristic",
};

uint8_t g_ManData[8];

/// Characteristic definitions
BLESRVC_CHAR g_UartChars[] = {
	{
		// Read characteristic
		BLUEIO_UUID_UART_RX_CHAR,
		20,
		BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
		s_RxCharDescString,         // char UTF-8 description string
		NULL,                       // Callback for write char, set to NULL for read char
		NULL,						// Callback on set notification
		NULL,						// Tx completed callback
		NULL,						// pointer to char default values
		0,							// Default value length in bytes
	},
	{
		// Write characteristic
		BLUEIO_UUID_UART_TX_CHAR,	// char UUID
		20,                         // char max data length
		BLESVC_CHAR_PROP_WRITEWORESP,	// char properties define by BLUEIOSVC_CHAR_PROP_...
		s_TxCharDescString,			// char UTF-8 description string
		UartTxSrvcCallback,         // Callback for write char, set to NULL for read char
		NULL,						// Callback on set notification
		NULL,						// Tx completed callback
		NULL,						// pointer to char default values
		0							// Default value length in bytes
	},
};

uint8_t g_LWrBuffer[512];

/// Service definition
const BLESRVC_CFG s_UartSrvcCfg = {
	BLESRVC_SECTYPE_NONE,	// Secure or Open service/char
	BLUEIO_UUID_BASE,           // Base UUID
	BLUEIO_UUID_UART_SERVICE,   // Service UUID
	2,                          // Total number of characteristics for the service
	g_UartChars,                // Pointer a an array of characteristic
	g_LWrBuffer,                // pointer to user long write buffer
	sizeof(g_LWrBuffer)         // long write buffer size
};

BLESRVC g_UartBleSrvc;

const BLEAPP_DEVDESC s_UartBleDevDesc {
	MODEL_NAME,           // Model name
	MANUFACTURER_NAME,          // Manufacturer name
	"",                     // Serial number string
	"0.0",                  // Firmware version string
	"0.0",                  // Hardware version string
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
	BLEAPP_MODE_APPSCHED,   // Use scheduler
	DEVICE_NAME,                 // Device name
	ISYST_BLUETOOTH_ID,     // PnP Bluetooth/USB vendor id
	1,                      // PnP Product ID
	0,						// Pnp prod version
	true,					// Enable device information service (DIS)
	&s_UartBleDevDesc,
	g_ManData,              // Manufacture specific data to advertise
	sizeof(g_ManData),      // Length of manufacture specific data
	BLEAPP_SECTYPE_STATICKEY_MITM,//BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	APP_ADV_INTERVAL,       // Advertising interval in msec
	APP_ADV_TIMEOUT_IN_SECONDS,	// Advertising timeout in sec
	0,                          // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	MIN_CONN_INTERVAL,
	MAX_CONN_INTERVAL,
	BLUEIO_CONNECT_LED_PORT,    // Led port nuber
	BLUEIO_CONNECT_LED_PIN,     // Led pin number
	0,							// Tx power
	NULL						// RTOS Softdevice handler
};

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

/// UART pins definitions
static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

/// UART configuration
const UARTCFG g_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPINCFG),
	1000000,
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
	UART_FLWCTRL_HW,
	true,
	APP_IRQ_PRIORITY_LOW,
	nRFUartEvthandler,
	false,
};

/// UART object instance
UART g_Uart;

static const IOPINCFG s_LedPins[] = {
	{BLUEIO_LED_BLUE_PORT, BLUEIO_LED_BLUE_PIN, BLUEIO_LED_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// LED1 (Blue)
	{BLUEIO_LED_GREEN_PORT, BLUEIO_LED_GREEN_PIN, BLUEIO_LED_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// LED2 (Green)
	{BLUEIO_LED_RED_PORT, BLUEIO_LED_RED_PIN, BLUEIO_LED_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// LED3 (Red)
};

static int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPINCFG);

static const IOPINCFG s_ButPins[] = {
	{BUTTON1_PORT, BUTTON1_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},// Button 1
	{BUTTON2_PORT, BUTTON2_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},// Button 2
};

static int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPINCFG);

int g_DelayCnt = 0;
volatile bool g_bUartState = false;

void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	g_Uart.Tx(pData, Len);
}

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
    BleSrvcEvtHandler(&g_UartBleSrvc, p_ble_evt);
}

void BleAppInitUserServices()
{
    uint32_t       err_code;

    err_code = BleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    APP_ERROR_CHECK(err_code);
}

void ButEvent(int IntNo)
{
	if (IntNo == 0)
	{
		if (g_bUartState == false)
		{
			g_Uart.Enable();
			g_bUartState = true;
		}
		else
		{
			g_Uart.Disable();
			g_bUartState = false;
		}
	}
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);

	IOPinCfg(s_ButPins, s_NbButPins);

	IOPinCfg(s_LedPins, s_NbLedPins);
	IOPinSet(BLUEIO_LED_BLUE_PORT, BLUEIO_LED_BLUE_PIN);
	IOPinSet(BLUEIO_LED_GREEN_PORT, BLUEIO_LED_GREEN_PIN);
	IOPinSet(BLUEIO_LED_RED_PORT, BLUEIO_LED_RED_PIN);

	IOPinCfg(s_ButPins, s_NbButPins);

	IOPinEnableInterrupt(0, APP_IRQ_PRIORITY_LOW, s_ButPins[0].PortNo, s_ButPins[0].PinNo, IOPINSENSE_LOW_TRANSITION, ButEvent);
}

void BleAppInitUserData()
{
	// Add passkey pairing
    ble_opt_t opt;
    opt.gap_opt.passkey.p_passkey = (uint8_t*)"123456";
	uint32_t err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &opt);
	APP_ERROR_CHECK(err_code);

}

void UartRxChedHandler(void * p_event_data, uint16_t event_size)
{
	uint8_t buff[128];

	int l = g_Uart.Rx(buff, 128);
	if (l > 0)
	{
		BleSrvcCharNotify(&g_UartBleSrvc, 0, buff, l);
	}
}

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			app_sched_event_put(NULL, 0, UartRxChedHandler);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
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

    //g_Uart.Disable();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    BleAppRun();

	return 0;
}
