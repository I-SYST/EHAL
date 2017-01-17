/*--------------------------------------------------------------------------
File   : blueio_blesrvc.h

Author : Hoang Nguyen Hoan          Mar. 25, 2014

Desc   : Implementation allow the creation of generic custom Bluetooth Smart
		 service with multiple user defined characteristics.

Copyright (c) 2014, I-SYST inc., all rights reserved

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

#ifndef __BLUEIO_SVC_H__
#define __BLUEIO_SVC_H__

#include "ble_srv_common.h"

// Default BlueIO UUID.  User should use privately generated UUID
// UUID : 00000000-287c-11e4-ab74-0002a5d5c51b
#define BLUEIO_UUID_BASE { 	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x74, 0xab, \
							0xe4, 0x11, 0x7c, 0x28, 0x00, 0x00, 0x00, 0x00 }
#define BLUEIO_UUID_SERVICE 		0x1
#define BLUEIO_UUID_RDCHAR 			0x2
#define BLUEIO_UUID_WRCHAR 			0x3

#define BLUEIOSVC_CHAR_PROP_READ			(1<<0)
#define BLUEIOSVC_CHAR_PROP_NOTIFY			(1<<1)
#define BLUEIOSVC_CHAR_PROP_WRITEWORESP		(1<<2)
#define BLUEIOSVC_CHAR_PROP_WRITE			(1<<3)

typedef struct __BlueIOBLEService BLUEIOSRVC;

typedef void (*BLUEIOSRVC_WRCB) (BLUEIOSRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

// Service connection security types
typedef enum {
	BLUEIOSRVC_SECTYPE_NONE,				// open, no security
	BLUEIOSRVC_SECTYPE_STATICKEY_NO_MITM,	// Bonding static pass key without Man In The Middle
	BLUEIOSRVC_SECTYPE_STATICKEY_MITM,		// Bonding static pass key with MITM
	BLUEIOSRVC_SECTYPE_LESC_MITM,			// LE secure encryption
	BLUEIOSRVC_SECTYPE_SIGNED_NO_MITM,		// AES signed encryption without MITM
	BLUEIOSRVC_SECTYPE_SIGNED_MITM,			// AES signed encryption with MITM
} BLUEIOSRVC_SECTYPE;

#pragma pack(push,4)

typedef struct {
    uint16_t Uuid;                          // char UUID
    int MaxDataLen;                         // char max data length
    uint32_t Property;                      // char properties define by BLUEIOSVC_CHAR_PROP_...
    const char *pDesc;                      // char UTF-8 description string
    BLUEIOSRVC_WRCB WrCB;                   // Callback for write char, set to NULL for read char
    bool bNotify;                           // Notify flag for read characteristic
    ble_gatts_char_handles_t Hdl;           // char handle
} BLUEIOSRVC_CHAR;

/*
 * User configuration for the service to be created
 */
typedef struct {
	BLUEIOSRVC_SECTYPE SecType;				// Secure or Open service/char
	ble_uuid128_t	UuidBase;				// Base UUID
	uint16_t		UuidSvc;				// Service UUID
	int             NbChar;                 // Total number of characteristics for the service
	BLUEIOSRVC_CHAR *pCharArray;            // Pointer a an array of characteristic
} BLUEIOSRVC_CFG;

/*
 * Blue IO Service private data to be passed when calling service related functions.
 * The data is filled by BlueIOBleSrvcInit function.
 * Pointer to this structure is often referred as Service Handle
 *
 */
struct __BlueIOBLEService {
    int             NbChar;                 // Number of characteristic defined for this service
    BLUEIOSRVC_CHAR *pCharArray;            // Pointer to array of characteristics
    uint16_t        SrvcHdl;                // Service handle
    uint16_t        ConnHdl;				// Connection handle
    uint16_t        UuidSvc;                // Service UUID
    uint8_t         UuidType;
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create BLE custom service
 *
 * @param	pSrvc	: Pointer to Blue IO service data to be filled when service
 * 					  is created
 * 			pCfg	: Pointer to configuration data for the service creation
 *
 * @return	0 - Success
 */
uint32_t BlueIOBleSrvcInit(BLUEIOSRVC *pSrvc, const BLUEIOSRVC_CFG *pCfg);

/**
 * Send characteristic data
 *
 * @param	pSrvc : Pointer to Blue IO service data (Service Handle)
 *			pData : Pointer to data to be sent
 *			DataLen : Length of data to send in bytes
 *
 * @return	Number of bytes sent
 */
uint16_t BlueIOBleSrvcCharSend(BLUEIOSRVC *pSrvc, int CharIdx, uint8_t *pData, uint16_t DataLen);

/**
 * BlueIO service event handler.  Call this within BLE dispatch event callback
 */
void BlueIOBleSvcEvtHandler(BLUEIOSRVC *pSrvc, ble_evt_t *pBleEvt);

#ifdef __cplusplus
}
#endif

#endif // __BLUEIO_SVC_H__

