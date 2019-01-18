/**-------------------------------------------------------------------------
@file	ble_dev.h

@brief	Implementation allow the creation of generic BLE peripheral device class.

This BLE device class is used by BLE Central application to connect and communicate
with it.

@author	Hoang Nguyen Hoan
@date	Jan. 17, 2019

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

#ifndef __BLE_DEV_H__
#define __BLE_DEV_H__

#include "ble_gatt_db.h"

#include "device.h"

#define BLEDEV_NAME_MAXLEN			20

typedef struct __Ble_Dev_Cfg {
	char Name[BLEDEV_NAME_MAXLEN];			//!< Device name
	uint8_t Addr[6];						//!< Device MAC address
} BLEDEV_CFG;

#define BLEPERIPH_DEV_SERVICE_MAXCNT	10
#define BLEPERIPH_DEV_NAME_MAXLEN		20

typedef struct __BlePeriphDevData {
	char Name[BLEPERIPH_DEV_NAME_MAXLEN];
	ble_gap_addr_t Addr;
	uint16_t ConnHdl;
	int NbSrvc;
	ble_gatt_db_srv_t Services[BLEPERIPH_DEV_SERVICE_MAXCNT];
} BLEPERIPH_DEV;

#ifdef __cplusplus

class BleDev : public Device {
public:
	virtual bool Init(BLEDEV_CFG &Cfg, DeviceIntrf * const pIntrf);
	virtual bool Connect();
	virtual void Disconnect();
	virtual int Send(uint8_t * const pData, int DataLen);
	virtual int Receive(uint8_t * const pBuff, int BuffLen);

protected:
private:
	BLEPERIPH_DEV Dev;
};

extern "C" {
#endif

bool BleAppDiscoverDevice(BLEPERIPH_DEV * const pDev);
/**
 * @brief	Peripheral discovered callback.
 *
 * This function is called in central mode when all services of the
 * device is fully discovered for the connected peripheral device.
 *
 * Application firmware should keep a copy of the peripheral data passed
 * for communication with the device.  The pointer is temporary and will be
 * destroyed upon return
 *
 * @param	pDev :	Pointer to peripheral device data
 */
void BleDevDiscovered(BLEPERIPH_DEV *pDev);
int FindService(BLEPERIPH_DEV * const pDev, uint16_t Uuid);

#ifdef __cplusplus
}
#endif


#endif // __BLE_DEV_H__

