/*--------------------------------------------------------------------------
File   : blueio_svc.h

Author : Hoang Nguyen Hoan          Mar. 25, 2014

Desc   : Implementation of a generic custom Bleutooth Smart service
		 with 2 characteristics.
		 	 control - where control packet is sent
		 	 data - where data is sent, streaming data

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

//#include "ble.h"
#include "ble_srv_common.h"
// UUID : c267dd40-287c-11e4-ab74-0002a5d5c51b
#define BLUEIO_UUID_BASE { 	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x74, 0xab, \
							0xe4, 0x11, 0x7c, 0x28, 0x40, 0xdd, 0x67, 0xc2 }
#define BLUEIO_UUID_SERVICE 		0xdd41
#define BLUEIO_UUID_CTRL 			0xdd42
#define BLUEIO_UUID_RXDATA 			0xdd43
#define BLUEIO_UUID_TXDATA 			0xdd44

#define BLUEIOSVC_CHAR_PROP_READ			(1<<0)
#define BLUEIOSVC_CHAR_PROP_NOTIFY			(1<<1)
#define BLUEIOSVC_CHAR_PROP_WRITEWORESP		(1<<2)
#define BLUEIOSVC_CHAR_PROP_WRITE			(1<<3)


typedef struct __BlueIOService BLUEIOSVC;

typedef void (*BLUEIOSVC_WRCB) (BLUEIOSVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len);

/*
 * Blue IO Service data structure
 */
struct __BlueIOService {
    uint8_t                     UuidType;
    uint16_t                    SvcHdl;					// Service handle
    ble_gatts_char_handles_t    CtrlCharHdl;			// Control char handle
    ble_gatts_char_handles_t    RxDataCharHdl;    		// Data char handle
    ble_gatts_char_handles_t    TxDataCharHdl;    		// Data char handle
    uint16_t                    ConnHdl;				// Connection handle
    bool                        bNotify;				// Notify flag
    BLUEIOSVC_WRCB				CtrlWrCB;				// Control char write callback
    BLUEIOSVC_WRCB 				TxDataWrCB;				// Data char write callback
};

typedef struct {
	bool			Secur;					// Secure or Open service/char
	ble_uuid128_t	UuidBase;				// Base UUID
	uint16_t		UuidSvc;				// Service UUID
	uint16_t		UuidCtrlChar;			// Control char UUID
	int				CtrlCharMaxLen;			// Control char max data length
	ble_gatts_char_md_t CtrlChar;			// Control char meta data
	uint16_t		UuidRxDataChar;			// Data char UUID
	int				RxDataCharMaxLen;		// Data char max data length
	ble_gatts_char_md_t RxDataChar;			// Data char meta data
	uint16_t		UuidTxDataChar;			// Data char UUID
	int				TxDataCharMaxLen;		// Data char max data length
	ble_gatts_char_md_t TxDataChar;			// Data char meta data
	BLUEIOSVC_WRCB	CtrlWrCB;				// Control char write callback
	BLUEIOSVC_WRCB	TxDataWrCB;				// Data char write callback
} BLUEIOSVC_CFG;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t BlueIOSvcInit(BLUEIOSVC *pSvc, const BLUEIOSVC_CFG *pCfg);

#ifdef __cplusplus
}
#endif

#endif // __BLUEIO_SVC_H__

