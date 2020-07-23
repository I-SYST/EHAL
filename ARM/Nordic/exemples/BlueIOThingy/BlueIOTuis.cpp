/**-------------------------------------------------------------------------
@file	BlueIOTcf.cpp

@brief	Thingy Configuration Service implementation

@author Hoang Nguyen Hoan
@date	Jul 14, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#include <inttypes.h>

#include "ble_service.h"
#include "iopinctrl.h"
#include "board.h"
#include "BlueIOThingy.h"

void LedCharWrhandler(BLESRVC *pBleSvc, uint8_t *pData, int Offset, int Len);
void ButtonCharSetNotify(BLESRVC *pBleSvc, bool bEnable);

// TUIS (Thingy User Interface Service characteristics)
#define BLE_UUID_TUIS_SERVICE          		0x0300

#define BLE_UUID_TUIS_LED_CHAR				0x0301                      /**< The UUID of the led Characteristic. */
#define BLE_UUID_TUIS_BUTTON_CHAR			0x0302                      /**< The UUID of the button Characteristic. */
#define BLE_UUID_TUIS_PIN_CHAR				0x0303                      /**< The UUID of the pin Characteristic. */


#define BLE_TUIS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Configuration service module. */

typedef enum {
	BLE_TUIS_LED_MODE_OFF,
	BLE_TUIS_LED_MODE_CONST,
	BLE_TUIS_LED_MODE_BREATHE,
	BLE_TUIS_LED_MODE_E_ONE_SHOT
} BLE_TUIS_LED_MODE;

#pragma pack(push, 1)
typedef union {
	struct {
        uint8_t r;          /**< Red intensity.   */
        uint8_t g;          /**< Green intensity. */
        uint8_t b;          /**< Blue intensity.  */
	};
	struct {
        uint8_t  color_mix; /**< Color mix. (on/off for each of the primary colors, R, G and B). */
        uint8_t  intensity; /**< LED intensity. */
        uint16_t delay;     /**< Delay between breathe sequences. */
	};
} BLE_TUIS_LED_DATA;
#pragma pack(pop)

typedef struct {
	BLE_TUIS_LED_MODE Mode;
	BLE_TUIS_LED_DATA Data;
} BLE_TUIS_LED;

//BLE_TUIS_LED s_ThingyVersion {
//	 2, 1, 0
//};

#define UICHAR_IDX_LED			0
#define UICHAR_IDX_BUTTON		1
#define UICHAR_IDX_PIN			2


static BLESRVC_CHAR s_UIChars[] = {
    {
        // Version characteristic.  This is the minimum required for Thingy App to work
    	BLE_UUID_TUIS_LED_CHAR,
        sizeof(BLE_TUIS_LED),
		BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_READ,
        NULL,    					// char UTF-8 description string
		LedCharWrhandler,           // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
		NULL,//(uint8_t*)&s_ThingyVersion,                       // pointer to char default values
        0,                          // Default value length in bytes
    },
    {
        // Version characteristic.  This is the minimum required for Thingy App to work
    	BLE_UUID_TUIS_BUTTON_CHAR,
        1,							// Max data len
		BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_READ,
        NULL,    					// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
		ButtonCharSetNotify,                       // Callback on set notification
        NULL,                       // Tx completed callback
		NULL,//(uint8_t*)&s_ThingyVersion,                       // pointer to char default values
        0,                          // Default value length in bytes
    },
    {
        // Version characteristic.  This is the minimum required for Thingy App to work
    	BLE_UUID_TUIS_PIN_CHAR,
        BLE_TUIS_MAX_DATA_LEN,
		BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY,
        NULL,    					// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
		NULL,//(uint8_t*)&s_ThingyVersion,                       // pointer to char default values
        0,                          // Default value length in bytes
    },
};

/// Service definition
const BLESRVC_CFG s_UISrvcCfg = {
    BLESRVC_SECTYPE_NONE,       	// Secure or Open service/char
    {THINGY_BASE_UUID,},           	// Base UUID
	1,
    BLE_UUID_TUIS_SERVICE,       	// Service UUID
    sizeof(s_UIChars) / sizeof(BLESRVC_CHAR),  // Total number of characteristics for the service
    s_UIChars,                 		// Pointer a an array of characteristic
    NULL,                       	// pointer to user long write buffer
    0,                           	// long write buffer size
	NULL,							// Authentication event callback
};

/// TUIS instance
BLESRVC g_UISrvc;

BLESRVC *GetUISrvcInstance()
{
	return &g_UISrvc;
}

uint32_t UISrvcInit()
{
	return BleSrvcInit(&g_UISrvc, &s_UISrvcCfg);
}

void LedCharWrhandler(BLESRVC *pBleSvc, uint8_t *pData, int Offset, int Len)
{
	BLE_TUIS_LED *leddata = (BLE_TUIS_LED*)pData;

	switch (leddata->Mode)
	{
		case BLE_TUIS_LED_MODE_OFF:
			IOPinSet(BLUEIO_TAG_EVIM_LED2_RED_PORT, BLUEIO_TAG_EVIM_LED2_RED_PIN);
			IOPinSet(BLUEIO_TAG_EVIM_LED2_GREEN_PORT, BLUEIO_TAG_EVIM_LED2_GREEN_PIN);
			IOPinSet(BLUEIO_TAG_EVIM_LED2_BLUE_PORT, BLUEIO_TAG_EVIM_LED2_BLUE_PIN);
			break;
		case BLE_TUIS_LED_MODE_CONST:
			IOPinClear(BLUEIO_TAG_EVIM_LED2_RED_PORT, BLUEIO_TAG_EVIM_LED2_RED_PIN);
			IOPinClear(BLUEIO_TAG_EVIM_LED2_GREEN_PORT, BLUEIO_TAG_EVIM_LED2_GREEN_PIN);
			IOPinClear(BLUEIO_TAG_EVIM_LED2_BLUE_PORT, BLUEIO_TAG_EVIM_LED2_BLUE_PIN);
			break;
		case BLE_TUIS_LED_MODE_BREATHE:
			break;
		case BLE_TUIS_LED_MODE_E_ONE_SHOT:
			break;
	}
}

void ButtonCharSetNotify(BLESRVC *pBleSvc, bool bEnable)
{
	s_UIChars[UICHAR_IDX_BUTTON].bNotify = true;
}

