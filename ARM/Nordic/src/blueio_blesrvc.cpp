/**-------------------------------------------------------------------------
@file	blueio_blesrvc.cpp

@brief	BLUEIO BLE custom service


@author	Hoang Nguyen Hoan
@date	Dec. 4, 2017

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
#include "ble.h"
#include "app_util_platform.h"

#include "ble_app.h"
#include "ble_service.h"
#include "ble_intrf.h"
#include "bluetooth/blueio_blesrvc.h"

void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

static const char s_NUSRxCharDescString[] = {
	"NUS Rx characteristic",
};

static const char s_NUSTxCharDescString[] = {
	"NUS Tx characteristic",
};

/// Characteristic definitions
BLESRVC_CHAR g_NUSChars[] = {
	{
		// Read characteristic
		.Uuid = BLE_UUID_NUS_RX_CHARACTERISTIC,
		.MaxDataLen = 20,
		.Property = BLE_UUID_NUS_RX_CHAR_PROP,
		.pDesc = s_NUSRxCharDescString,		// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0,						// Default value length in bytes
	},
	{
		// Write characteristic
		.Uuid = BLE_UUID_NUS_TX_CHARACTERISTIC,	// char UUID
		.MaxDataLen = 20,					// char max data length
		.Property = BLE_UUID_NUS_TX_CHAR_PROP,// char properties define by BLUEIOSVC_CHAR_PROP_...
		.pDesc = s_NUSTxCharDescString,		// char UTF-8 description string
		.WrCB = UartTxSrvcCallback,			// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0						// Default value length in bytes
	},
};

static const int s_NbNUSChar = sizeof(g_NUSChars) / sizeof(BLESRVC_CHAR);

//uint8_t g_LWrBuffer[512];

/// Service definition
const BLESRVC_CFG s_NUSSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.UuidBase = NUS_BASE_UUID,			// Base UUID
	.UuidSvc = BLE_UUID_NUS_SERVICE,		// Service UUID
	.NbChar = s_NbNUSChar,				// Total number of characteristics for the service
	.pCharArray = g_NUSChars,				// Pointer a an array of characteristic
	.pLongWrBuff = NULL,//g_LWrBuffer,				// pointer to user long write buffer
	.LongWrBuffSize = 0,//sizeof(g_LWrBuffer),	// long write buffer size
};

BLESRVC g_NUSBleSrvc;

static const char s_RxCharDescString[] = {
	"UART Rx characteristic",
};

static const char s_TxCharDescString[] = {
	"UART Tx characteristic",
};

/// Characteristic definitions
BLESRVC_CHAR g_UartChars[] = {
	{
		// Read characteristic
		.Uuid = BLUEIO_UUID_UART_RX_CHAR,
		.MaxDataLen = 20,
		.Property = BLUEIO_UUID_UART_RX_CHAR_PROP,
		.pDesc = s_RxCharDescString,		// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0,						// Default value length in bytes
	},
	{
		// Write characteristic
		.Uuid = BLUEIO_UUID_UART_TX_CHAR,	// char UUID
		.MaxDataLen = 20,					// char max data length
		.Property = BLUEIO_UUID_UART_TX_CHAR_PROP,// char properties define by BLUEIOSVC_CHAR_PROP_...
		.pDesc = s_TxCharDescString,		// char UTF-8 description string
		.WrCB = UartTxSrvcCallback,			// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0						// Default value length in bytes
	},
};

static const int s_NbUartChar = sizeof(g_UartChars) / sizeof(BLESRVC_CHAR);

//uint8_t g_LWrBuffer[512];

/// Service definition
const BLESRVC_CFG s_UartSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.UuidBase = BLUEIO_UUID_BASE,			// Base UUID
	.UuidSvc = BLUEIO_UUID_UART_SERVICE,		// Service UUID
	.NbChar = s_NbUartChar,				// Total number of characteristics for the service
	.pCharArray = g_UartChars,				// Pointer a an array of characteristic
	.pLongWrBuff = NULL,//g_LWrBuffer,				// pointer to user long write buffer
	.LongWrBuffSize = 0,//sizeof(g_LWrBuffer),	// long write buffer size
};

BLESRVC g_UartBleSrvc;

static const char s_BlueIOCmdCharDescString[] = {
	"Command characteristic",
};

static const char s_BlueIODataCharDescString[] = {
	"Data characteristic",
};

/// Characteristic definitions
BLESRVC_CHAR g_BlueIOChars[] = {
	{
		// Read characteristic
		.Uuid = BLUEIO_UUID_DATACHAR,
		.MaxDataLen = 20,
		.Property = BLUEIO_UUID_DATACHAR_PROP,
		.pDesc = s_BlueIODataCharDescString,		// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0,						// Default value length in bytes
	},
	{
		// Write characteristic
		.Uuid = BLUEIO_UUID_CMDCHAR,		// char UUID
		.MaxDataLen = 20,					// char max data length
		.Property = BLUEIO_UUID_CMDCHAR_PROP,// char properties define by BLUEIOSVC_CHAR_PROP_...
		.pDesc = s_BlueIOCmdCharDescString,		// char UTF-8 description string
		.WrCB = UartTxSrvcCallback,			// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0						// Default value length in bytes
	},
};

static const int s_NbBlueIOChar = sizeof(g_BlueIOChars) / sizeof(BLESRVC_CHAR);

uint8_t g_LWrBuffer[512];

/// Service definition
const BLESRVC_CFG s_BlueIOSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.UuidBase = BLUEIO_UUID_BASE,			// Base UUID
	.UuidSvc = BLUEIO_UUID_SERVICE,		// Service UUID
	.NbChar = s_NbBlueIOChar,				// Total number of characteristics for the service
	.pCharArray = g_BlueIOChars,				// Pointer a an array of characteristic
	.pLongWrBuff = g_LWrBuffer,				// pointer to user long write buffer
	.LongWrBuffSize = sizeof(g_LWrBuffer),	// long write buffer size
};

BLESRVC g_BlueIOBleSrvc;

#define BLUEIO_INTRF_CFIFO_MEMSIZE			CFIFO_TOTAL_MEMSIZE(6, sizeof(BLUEIO_PACKET))

static uint8_t g_BlueIOIntrfRxBuff[BLUEIO_INTRF_CFIFO_MEMSIZE];
static uint8_t g_BlueIOIntrfTxBuff[BLUEIO_INTRF_CFIFO_MEMSIZE];

static const BLEINTRF_CFG s_BlueIOBleIntrfCfg = {
	.pBleSrv = &g_BlueIOBleSrvc,
	.RxCharIdx = 1,
	.TxCharIdx = 0,
	.PacketSize = sizeof(BLUEIO_PACKET),
	.RxFifoMemSize = BLUEIO_INTRF_CFIFO_MEMSIZE,
	.pRxFifoMem = g_BlueIOIntrfRxBuff,
	.TxFifoMemSize = BLUEIO_INTRF_CFIFO_MEMSIZE,
	.pTxFifoMem = g_BlueIOIntrfTxBuff,
};

BLEINTRF g_BlueIOBleIntrf;

static bool g_BlueIOSrvcInitialized = false;

BLESRVC *GetUartSrvcInstance()
{
	return &g_UartBleSrvc;
}

BLESRVC *GetBlueIOSrvcInstance()
{
	return &g_BlueIOBleSrvc;
}

BLESRVC *GetNUSSrvcInstance()
{
	return &g_NUSBleSrvc;
}

bool BlueIOSrvcInit(BLUEIOSRVC_CFG * const pCfg)
{
    uint32_t err_code;

    err_code = BleSrvcInit(&g_NUSBleSrvc, &s_NUSSrvcCfg);
    APP_ERROR_CHECK(err_code);

    err_code = BleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    APP_ERROR_CHECK(err_code);

    err_code = BleSrvcInit(&g_BlueIOBleSrvc, &s_BlueIOSrvcCfg);
    APP_ERROR_CHECK(err_code);

    g_BlueIOSrvcInitialized = true;

    return err_code == NRF_SUCCESS;
}

void BlueIOSrvcEvtHandler(ble_evt_t * p_ble_evt)
{
	if (g_BlueIOSrvcInitialized == true)
	{
		BleSrvcEvtHandler(&g_NUSBleSrvc, p_ble_evt);
		BleSrvcEvtHandler(&g_UartBleSrvc, p_ble_evt);
		BleSrvcEvtHandler(&g_BlueIOBleSrvc, p_ble_evt);
	}
}
