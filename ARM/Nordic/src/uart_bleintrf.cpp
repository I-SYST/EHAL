/**-------------------------------------------------------------------------
@file	uart_bleintrf.cpp

@brief	Implementation of UART BLE Interface object

The interface can be either Nordic NUS or BlueIO UART based

@author	Hoang Nguyen Hoan
@date	Feb. 18, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#include "coredev/uart.h"
#include "bluetooth/blueio_blesrvc.h"
#include "ble_intrf.h"
#include "blueio_board.h"

int BleIntrfEvtCallback(DEVINTRF *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);
int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// NUS BLE

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
		.MaxDataLen = BLE_NUSUART_MAX_DATA_LEN,
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
		.MaxDataLen = BLE_NUSUART_MAX_DATA_LEN,					// char max data length
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
const BLESRVC_CFG s_NUSBleSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	.UuidBase = NUS_BASE_UUID,				// Base UUID
	.UuidSvc = BLE_UUID_NUS_SERVICE,		// Service UUID
	.NbChar = s_NbNUSChar,					// Total number of characteristics for the service
	.pCharArray = g_NUSChars,				// Pointer a an array of characteristic
	.pLongWrBuff = NULL,//g_LWrBuffer,		// pointer to user long write buffer
	.LongWrBuffSize = 0,//sizeof(g_LWrBuffer),	// long write buffer size
};

BLESRVC g_NUSBleSrvc;

#define NUS_INTRF_CFIFO_MEMSIZE			CFIFO_MEMSIZE(6 * BLE_NUSUART_MAX_DATA_LEN)

static uint8_t s_NUSIntrfRxBuff[NUS_INTRF_CFIFO_MEMSIZE];
static uint8_t s_NUSIntrfTxBuff[NUS_INTRF_CFIFO_MEMSIZE];

static BLEINTRF_CFG s_NUSBleIntrfCfg = {
	.pBleSrv = &g_NUSBleSrvc,
	.RxCharIdx = 1,
	.TxCharIdx = 0,
	.PacketSize = sizeof(BLUEIO_PACKET),
	.RxFifoMemSize = NUS_INTRF_CFIFO_MEMSIZE,
	.pRxFifoMem = s_NUSIntrfRxBuff,
	.TxFifoMemSize = NUS_INTRF_CFIFO_MEMSIZE,
	.pTxFifoMem = s_NUSIntrfTxBuff,
	.EvtCB = BleIntrfEvtCallback
};

static BleIntrf s_NUSBleIntrf;

// -----------------------
// UART BLE

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
		.MaxDataLen = BLE_NUSUART_MAX_DATA_LEN,
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
		.MaxDataLen = BLE_NUSUART_MAX_DATA_LEN,					// char max data length
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
	.UuidSvc = BLUEIO_UUID_UART_SERVICE,	// Service UUID
	.NbChar = s_NbUartChar,					// Total number of characteristics for the service
	.pCharArray = g_UartChars,				// Pointer a an array of characteristic
	.pLongWrBuff = NULL,//g_LWrBuffer,		// pointer to user long write buffer
	.LongWrBuffSize = 0,//sizeof(g_LWrBuffer),	// long write buffer size
};

BLESRVC g_UartBleSrvc;


#define BLUEIO_INTRF_CFIFO_MEMSIZE			CFIFO_MEMSIZE(6 * BLE_NUSUART_MAX_DATA_LEN)

static uint8_t s_BlueIOIntrfRxBuff[BLUEIO_INTRF_CFIFO_MEMSIZE];
static uint8_t s_BlueIOIntrfTxBuff[BLUEIO_INTRF_CFIFO_MEMSIZE];

static BLEINTRF_CFG s_BlueIOBleIntrfCfg = {
	.pBleSrv = &g_UartBleSrvc,
	.RxCharIdx = 1,
	.TxCharIdx = 0,
	.PacketSize = sizeof(BLUEIO_PACKET),
	.RxFifoMemSize = BLUEIO_INTRF_CFIFO_MEMSIZE,
	.pRxFifoMem = s_BlueIOIntrfRxBuff,
	.TxFifoMemSize = BLUEIO_INTRF_CFIFO_MEMSIZE,
	.pTxFifoMem = s_BlueIOIntrfTxBuff,
	.EvtCB = BleIntrfEvtCallback
};

static BleIntrf s_BlueIOBleIntrf;
/*
static const IOPINCFG s_UartPins[] = {
	{BLUEIO_UART_RX_PORT, BLUEIO_UART_RX_PIN, BLUEIO_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// RX
	{BLUEIO_UART_TX_PORT, BLUEIO_UART_TX_PIN, BLUEIO_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// TX
	{-1, -1, -1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{-1, -1, -1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
};

static UARTCFG s_UartCfg = {
	.DevNo = 0,							// Device number zero based
	.pIoMap = s_UartPins,				// UART assigned pins
	.IoMapLen = sizeof(s_UartPins) / sizeof(IOPINCFG),	// Total number of UART pins used
	.Rate = 1000000,						// Baudrate
	.DataBits = 8,						// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,						// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,					// Interrupt mode
	.IntPrio = APP_IRQ_PRIORITY_LOW,	// Interrupt priority
	.EvtCallback = nRFUartEvthandler,	// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
};

static UART s_Uart;
*/
int BleIntrfEvtCallback(DEVINTRF *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		uint8_t buff[128];

		int l = s_BlueIOBleIntrf.Rx(0, buff, 128);
/*		if (l > 0)
		{
			s_Uart.Tx(buff, l);
		}*/
		cnt += l;
	}

	return cnt;
}
/*
void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	s_Uart.Tx(pData, Len);
}

void UartRxChedHandler(void * p_event_data, uint16_t event_size)
{
	uint8_t buff[128];

	int l = s_Uart.Rx(buff, 128);
	if (l > 0)
	{
		BleSrvcCharNotify(&g_UartBleSrvc, 0, buff, l);
	}
}

bool BlueIOUartSrvcInit(IOPINCFG *pPins, int NbPins, uint32_t Rate)
{
    uint32_t err_code;

    if (pPins == NULL)
    	return false;

    s_UartCfg.pIoMap = pPins;
    s_UartCfg.IoMapLen = NbPins;
    s_UartCfg.Rate = Rate;

    s_Uart.Init(s_UartCfg);

    err_code = BleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    APP_ERROR_CHECK(err_code);

    s_BlueIOBleIntrf.Init(s_BlueIOBleIntrfCfg);

    return err_code;
}

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			//app_sched_event_put(NULL, 0, UartRxChedHandler);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}*/

BleIntrf * const NusBleIntrfInit(DEVINTRF_EVTCB EvtCB)
{
    uint32_t err_code = BleSrvcInit(&g_NUSBleSrvc, &s_NUSBleSrvcCfg);
    APP_ERROR_CHECK(err_code);

    s_NUSBleIntrfCfg.EvtCB = EvtCB;

    if (s_NUSBleIntrf.Init(s_NUSBleIntrfCfg))
    {
    	return &s_NUSBleIntrf;
    }

    return NULL;
}

BleIntrf * const UartBleIntrfInit(DEVINTRF_EVTCB EvtCB)
{
    uint32_t err_code = BleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    APP_ERROR_CHECK(err_code);

    s_BlueIOBleIntrfCfg.EvtCB = EvtCB;

    if (s_BlueIOBleIntrf.Init(s_BlueIOBleIntrfCfg))
    {
    	return &s_BlueIOBleIntrf;
    }

    return NULL;
}
