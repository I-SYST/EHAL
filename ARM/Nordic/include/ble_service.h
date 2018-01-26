/**-------------------------------------------------------------------------
@file	ble_service.h

@brief	Implementation allow the creation of generic custom Bluetooth LE
service with multiple user defined characteristics. This implementation is to be used with Nordic SDK


@author	Hoang Nguyen Hoan
@date	Mar. 25, 2014

@license

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

----------------------------------------------------------------------------*/

#ifndef __BLE_SERVICE_H__
#define __BLE_SERVICE_H__

#include "ble_srv_common.h"

/// Default BlueIO UUID.  User should use privately generated UUID
/// UUID : 00000000-287c-11e4-ab74-0002a5d5c51b
#define BLUEIO_UUID_BASE { 	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x74, 0xab, \
							0xe4, 0x11, 0x7c, 0x28, 0x00, 0x00, 0x00, 0x00 }
#define BLUEIO_UUID_SERVICE 		0x1
#define BLUEIO_UUID_RDCHAR 			0x2
#define BLUEIO_UUID_WRCHAR 			0x3

#define BLUEIO_UUID_UART_SERVICE 	0x101
#define BLUEIO_UUID_UART_RX_CHAR	0x102
#define BLUEIO_UUID_UART_TX_CHAR	0x103

#define BLESVC_CHAR_PROP_READ			(1<<0)
#define BLESVC_CHAR_PROP_NOTIFY			(1<<1)
#define BLESVC_CHAR_PROP_WRITEWORESP	(1<<2)
#define BLESVC_CHAR_PROP_WRITE			(1<<3)
#define BLESVC_CHAR_PROP_VARLEN			(1<<4)
#define BLESVC_CHAR_PROP_RDAUTH			(1<<5)
#define BLESVC_CHAR_PROP_WRAUTH			(1<<6)


typedef struct __BLE_Service_Data BLESRVC;

/**
 * @brief	Callback on write
 */
typedef void (*BLESRVC_WRCB) (BLESRVC *pBleSvc, uint8_t *pData, int Offset, int Len);

/**
 * @brief	Callback on set notification
 */
typedef void (*BLESRVC_SETNOTCB) (BLESRVC *pBleSvc, bool bEnable);

/**
 * @brief	Callback when transmission is completed
 *
 * @param	pBlueIOSvc
 * @param	CharIdx
 */
typedef void (*BLESRVC_TXCOMPLETE) (BLESRVC *pBleSvc, int CharIdx);

/**
 * @brief	Callback on authorization request
 *
 * @param	pBlueIOSvc
 * @param	p_ble_evt
 */
typedef void (*BLESRVC_AUTHREQ)(BLESRVC *pBleSvc, ble_evt_t * p_ble_evt);

// Service connection security types
typedef enum {
	BLESRVC_SECTYPE_NONE,				//!< open, no security
	BLESRVC_SECTYPE_STATICKEY_NO_MITM,	//!< Bonding static pass key without Man In The Middle
	BLESRVC_SECTYPE_STATICKEY_MITM,		//!< Bonding static pass key with MITM
	BLESRVC_SECTYPE_LESC_MITM,			//!< LE secure encryption
	BLESRVC_SECTYPE_SIGNED_NO_MITM,		//!< AES signed encryption without MITM
	BLESRVC_SECTYPE_SIGNED_MITM,		//!< AES signed encryption with MITM
} BLESRVC_SECTYPE;

#pragma pack(push,4)

typedef struct __BLE_Service_Char_Data {
    uint16_t Uuid;                      //!< char UUID
    int MaxDataLen;                     //!< char max data length
    uint32_t Property;                  //!< char properties define by BLUEIOSVC_CHAR_PROP_...
    const char *pDesc;                  //!< char UTF-8 description string
    BLESRVC_WRCB WrCB;                  //!< Callback for write char, set to NULL for read char
    BLESRVC_SETNOTCB SetNotifCB;		//!< Callback on set notification
    BLESRVC_TXCOMPLETE TxCompleteCB;	//!< Callback when TX is completed
    uint8_t *pDefValue;					//!< pointer to char default values
    uint16_t ValueLen;					//!< Default value length in bytes
    ble_gatts_char_handles_t Hdl;       //!< char handle
    bool bNotify;                       //!< Notify flag for read characteristic
} BLESRVC_CHAR;

/*
 * User configuration for the service to be created
 */
typedef struct __BLE_Service_Config {
	BLESRVC_SECTYPE SecType;			//!< Secure or Open service/char
	ble_uuid128_t	UuidBase;			//!< Base UUID
	uint16_t		UuidSvc;			//!< Service UUID
	int             NbChar;				//!< Total number of characteristics for the service
	BLESRVC_CHAR *pCharArray;           //!< Pointer a an array of characteristic
    uint8_t			*pLongWrBuff;		//!< pointer to user long write buffer
    int				LongWrBuffSize;		//!< long write buffer size
    BLESRVC_AUTHREQ	AuthReqCB;			//!< Authorization request callback
} BLESRVC_CFG;

/*
 * Blue IO Service private data to be passed when calling service related functions.
 * The data is filled by BlueIOBleSrvcInit function.
 * Pointer to this structure is often referred as Service Handle
 *
 */
struct __BLE_Service_Data {
    int             NbChar;				//!< Number of characteristic defined for this service
    BLESRVC_CHAR 	*pCharArray;        //!< Pointer to array of characteristics
    uint16_t        SrvcHdl;            //!< Service handle
    uint16_t        ConnHdl;			//!< Connection handle
    uint16_t        UuidSvc;            //!< Service UUID
    uint8_t         UuidType;
    uint8_t			*pLongWrBuff;		//!< pointer to user long write buffer
    int				LongWrBuffSize;		//!< long write buffer size
    void			*pContext;
    BLESRVC_AUTHREQ	AuthReqCB;			//!< Authorization request callback
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Create BLE custom service
 *
 * @param	pSrvc	: Pointer to Blue IO service data to be filled when service
 * 					  is created
 * @param	pCfg	: Pointer to configuration data for the service creation
 *
 * @return	0 - Success
 */
uint32_t BleSrvcInit(BLESRVC *pSrvc, const BLESRVC_CFG *pCfg);

/**
 * @brief	Notify characteristic data
 *
 * @param	pSrvc : Pointer to Blue IO service data (Service Handle)
 * @param	Idx   : Characteristic index to notify
 * @param	pData : Pointer to data to be sent
 * @param	DataLen : Length of data to send in bytes
 *
 * @return	0 - Success
 */
uint32_t BleSrvcCharNotify(BLESRVC *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen);

/**
 * @brief	Update characteristic data
 *
 * @param	pSrvc : Pointer to Blue IO service data (Service Handle)
 * @param	Idx   : Characteristic index to update
 * @param	pData : Pointer to data to be sent
 * @param	DataLen : Length of data to send in bytes
 *
 * @return	0 - Success
 */
uint32_t BleSrvcCharSetValue(BLESRVC *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen);

/**
 * #brief	BlueIO service event handler.  Call this within BLE dispatch event callback
 */
void BleSrvcEvtHandler(BLESRVC *pSrvc, ble_evt_t *pBleEvt);

#ifdef __cplusplus
}

class BleService {

	BleService() {}
	virtual ~BleService() {}

	virtual uint32_t Init(BLESRVC_CFG &Cfg) {
	    vSrvc.pContext = (void*)this;

		return BleSrvcInit(&vSrvc, &Cfg);
	}
	virtual uint32_t CharNotify(int Idx, uint8_t *pData, int DataLen) {
		return BleSrvcCharNotify(&vSrvc, Idx, pData, DataLen);
	}
	virtual uint32_t CharSetValue(int Idx, uint8_t *pData, int DataLen) {
		return BleSrvcCharSetValue(&vSrvc, Idx, pData, DataLen);
	}
	virtual operator BLESRVC* () { return &vSrvc; }

public:
	BLESRVC vSrvc;
};

#endif

#endif // __BLE_SERVICE_H__

