/*--------------------------------------------------------------------------
File   : UartBleCentralDemo.cpp

Author : Hoang Nguyen Hoan          Feb. 4, 2017

Desc   : Uart BLE Central demo
		 This application demo shows UART Rx/Tx over BLE custom service
		 using EHAL library.

Copyright (c) 2016, I-SYST inc., all rights reserved

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
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "ble_gap.h"

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"
#include "blueio_board.h"
#include "uart.h"
#include "custom_board.h"
#include "iopincfg.h"
#include "stddev.h"
#include "board.h"

#define DEVICE_NAME                     "UARTDemo"                            /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME                      "IMM-NRF51x"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                               /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#define SCAN_INTERVAL           MSEC_TO_UNITS(1000, UNIT_0_625_MS)      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             MSEC_TO_UNITS(100, UNIT_0_625_MS)                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0                                 /**< Timout when scanning. 0x0000 disables timeout. */

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

BLESRVC_CHAR g_UartChars[] = {
	{
		// Read characteristic
		BLUEIO_UUID_UART_RX_CHAR,
		20,
		BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY,
		s_RxCharDescString,         // char UTF-8 description string
		NULL,                       // Callback for write char, set to NULL for read char
		true,                       // Notify flag for read characteristic
		NULL,						// Callback on set notification
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
		false,                      // Notify flag for read characteristic
		NULL,						// Callback on set notification
		NULL,						// pointer to char default values
		0							// Default value length in bytes
	},
};

uint8_t g_LWrBuffer[512];

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
		0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
#endif

	},
	1, 						// Number of central link
	0, 						// Number of peripheral link
	BLEAPP_MODE_APPSCHED,   // Use scheduler
	DEVICE_NAME,                 // Device name
	ISYST_BLUETOOTH_ID,     // PnP Bluetooth/USB vendor id
	1,                      // PnP Product ID
	0,						// Pnp prod version
	false,					// Enable device information service (DIS)
	NULL,//&s_UartBleDevDesc,
	NULL,//g_ManData,              // Manufacture specific data to advertise
	0,//sizeof(g_ManData),      // Length of manufacture specific data
	BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	0,       // Advertising interval in msec
	0,	// Advertising timeout in sec
	0,                          // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	MIN_CONN_INTERVAL,
	MAX_CONN_INTERVAL,
	BLUEIO_CONNECT_LED_PORT,    // Led port nuber
	BLUEIO_CONNECT_LED_PIN,     // Led pin number
	NULL						// RTOS Softdevice handler
};

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// UART configuration data

static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

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
	true,
};

// UART object instance
UART g_Uart;

int g_DelayCnt = 0;

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const g_ScanParams =
{
    1,		// Active scan
    #if (NRF_SD_BLE_API_VERSION <= 2)
        0,	// .selective
        NULL,	// .p_whitelist
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        0,				// Use whitelist
		0, 				// Report directed advertisement
    #endif
	    SCAN_INTERVAL,	// Scan interval
	    SCAN_WINDOW,	// Scan window
	    SCAN_TIMEOUT,	// Scan timeout
};

void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt)
{
    ret_code_t            err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    uint8_t addr[6] = { 0xda, 0x02, 0xe8, 0xfe, 0xac, 0xd1};

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
            if (memcmp(addr, p_adv_report->peer_addr.addr, 6) == 0)
            {
            	g_Uart.printf("Addr: %02x:%02x:%02x:%02x:%02x:%02x,  RSSI %d\r\n",
					p_adv_report->peer_addr.addr[0], p_adv_report->peer_addr.addr[1],
					p_adv_report->peer_addr.addr[2],
					p_adv_report->peer_addr.addr[3],
					p_adv_report->peer_addr.addr[4],
					p_adv_report->peer_addr.addr[5],
					p_adv_report->rssi
            	 );
            }
        }
        break; // BLE_GAP_EVT_ADV_REPORT
    }
}

void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	g_Uart.Tx(pData, Len);
}

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
//    BleSrvcEvtHandler(&g_UartBleSrvc, p_ble_evt);
}

void BleAppInitUserServices()
{
    //uint32_t       err_code;

    //err_code = BleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    //APP_ERROR_CHECK(err_code);
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	printf("UART BLE Central Demo\r\n");
}

void BleAppInitUserData()
{

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
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
    HardwareInit();

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    uint32_t ret = sd_ble_gap_scan_start(&g_ScanParams);
    APP_ERROR_CHECK(ret);

    BleAppRun();

	return 0;
}

