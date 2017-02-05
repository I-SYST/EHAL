/*--------------------------------------------------------------------------
File   : main.cpp

Author : Hoang Nguyen Hoan          Feb. 4, 2017

Desc   : Uart BLE demo
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

#include "istddef.h"
#include "ble_periph_app.h"
#include "blueio_blesrvc.h"
#include "blueio_board.h"
#include "uart.h"
#include "custom_board.h"
#include "iopincfg.h"
#include "app_util_platform.h"
#include "app_scheduler.h"

#define DEVICE_NAME                     "UART"                            /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "DISPLAY"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                               /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

void UartTxSrvcCallback(BLUEIOSRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

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

BLUEIOSRVC_CHAR g_UartChars[] = {
	{
		BLUEIO_UUID_UART_RX_CHAR,
		20,
		BLUEIOSVC_CHAR_PROP_READ | BLUEIOSVC_CHAR_PROP_NOTIFY,
		s_RxCharDescString,
		NULL,
		true,
	},
	{
		BLUEIO_UUID_UART_TX_CHAR,
		20,
		BLUEIOSVC_CHAR_PROP_WRITEWORESP,
		s_TxCharDescString,
		UartTxSrvcCallback,
		false,
	},
};

uint8_t g_LWrBuffer[512];

const BLUEIOSRVC_CFG s_UartSrvcCfg = {
	BLUEIOSRVC_SECTYPE_NONE,
	BLUEIO_UUID_BASE,
	BLUEIO_UUID_UART_SERVICE,
	2,
	g_UartChars,
	g_LWrBuffer,
	sizeof(g_LWrBuffer)
};

BLUEIOSRVC g_UartBleSrvc;

const BLEAPP_CFG s_BleAppCfg = {
	NRF_CLOCK_LFCLKSRC,
	BLEAPP_MODE_APPSCHED,
	"UART",
	"IBK-BLUEIO",
	"I-SYST inc.",
	"",
	"0.0",
	"0.0",
	ISYST_BLUETOOTH_ID,
	1,
	g_ManData,
	sizeof(g_ManData),
	BLEAPP_SECTYPE_NONE,
	BLEAPP_SECEXCHG_NONE,
	NULL,//s_AdvUuids,
	0,//sizeof(s_AdvUuids) / sizeof(ble_uuid_t),
	APP_ADV_INTERVAL,
	APP_ADV_TIMEOUT_IN_SECONDS,
	0,
	BLUEIO_CONNECT_LED_PORT,
	BLUEIO_CONNECT_LED_PIN,
	NULL
};

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// UART configuration data

static IOPINCFG s_UartPins[] = {
	{BLUEIO_UART_RX_PORT, BLUEIO_UART_RX_PIN, BLUEIO_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{BLUEIO_UART_TX_PORT, BLUEIO_UART_TX_PIN, BLUEIO_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{BLUEIO_UART_CTS_PORT, BLUEIO_UART_CTS_PIN, BLUEIO_UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{BLUEIO_UART_RTS_PORT, BLUEIO_UART_RTS_PIN, BLUEIO_UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

const UARTCFG g_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPINCFG),
	1000000,
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
	UART_FLWCTRL_NONE,
	true,
	APP_IRQ_PRIORITY_LOW,
	nRFUartEvthandler,
	false,
};

// UART object instance
UART g_Uart;

int g_DelayCnt = 0;

void UartTxSrvcCallback(BLUEIOSRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	g_Uart.Tx(pData, Len);
}

void BlePeriphAppSrvcEvtDispatch(ble_evt_t * p_ble_evt)
{
    BlueIOBleSvcEvtHandler(&g_UartBleSrvc, p_ble_evt);
}

void BlePeriphAppInitServices()
{
    uint32_t       err_code;

    err_code = BlueIOBleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    APP_ERROR_CHECK(err_code);
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);
}

void BlePeriphAppInitUserData()
{

}

void UartRxChedHandler(void * p_event_data, uint16_t event_size)
{
	uint8_t buff[128];

	int l = g_Uart.Rx(buff, 128);
	if (l > 0)
	{
		BlueIOBleSrvcCharSend(&g_UartBleSrvc, 0, buff, l);
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

    BlePeriphAppInit(&s_BleAppCfg, true);

    BlePeriphAppStart();

    while(1)
    {
    	BlePeriphAppProcessEvt();
    }
	return 0;
}
