/**-------------------------------------------------------------------------
@file	ble_app.h

@brief	Nordic SDK based BLE peripheral application creation helper


@author	Hoang Nguyen Hoan
@date	Dec 26, 2016

@license

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

----------------------------------------------------------------------------*/
#ifndef __BLE_APP_H__
#define __BLE_APP_H__

#include <stdint.h>

#include "ble.h"
#include "nrf_sdm.h"

#include "bluetooth/bleadv_mandata.h"

/** @addtogroup Bluetooth
  * @{
  */

/**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#if (NRF_SD_BLE_API_VERSION <= 3)
   #define NRF_BLE_MAX_MTU_SIZE        GATT_MTU_SIZE_DEFAULT
#else

#if  defined(BLE_GATT_MTU_SIZE_DEFAULT) && !defined(GATT_MTU_SIZE_DEFAULT)
#define GATT_MTU_SIZE_DEFAULT BLE_GATT_MTU_SIZE_DEFAULT
#endif

#if  defined(BLE_GATT_ATT_MTU_DEFAULT) && !defined(GATT_MTU_SIZE_DEFAULT)
#define GATT_MTU_SIZE_DEFAULT  			BLE_GATT_ATT_MTU_DEFAULT
#endif

#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT

#endif

#define BLE_MAX_DATA_LEN				251

typedef enum __BleApp_AdvMode {
	BLEAPP_ADVMODE_IDLE,				//!< no connectable advertising is ongoing.
	BLEAPP_ADVMODE_DIRECTED,			//!< Directed advertising attempts to connect to the most recently disconnected peer.
	BLEAPP_ADVMODE_DIRECTED_SLOW,	//!< Directed advertising (low duty cycle) attempts to connect to the most recently disconnected peer.
	BLEAPP_ADVMODE_FAST,				//!< Fast advertising will connect to any peer device, or filter with a whitelist if one exists.
	BLEAPP_ADVMODE_SLOW				//!< Slow advertising is similar to fast advertising. By default, it uses a longer
									//!< advertising interval and time-out than fast advertising. However, these options are defined by the user.
} BLEAPP_ADVMODE;

typedef enum __BleApp_Mode {
	BLEAPP_MODE_LOOP,				//!< just main loop (event mode), No scheduler, no RTOS
	BLEAPP_MODE_APPSCHED,			//!< use app_cheduler
	BLEAPP_MODE_RTOS,				//!< use RTOS
	BLEAPP_MODE_NOCONNECT,			//!< Connectionless beacon type of app.
	BLEAPP_MODE_IBEACON				//!< Apple iBeacon
} BLEAPP_MODE;

// Service connection security types
typedef enum __BleApp_SecurityType {
	BLEAPP_SECTYPE_NONE,					//!< open, no security
	BLEAPP_SECTYPE_STATICKEY_NO_MITM,	//!< Bonding static pass key without Man In The Middle
	BLEAPP_SECTYPE_STATICKEY_MITM,		//!< Bonding static pass key with MITM
	BLEAPP_SECTYPE_LESC_MITM,			//!< LE secure encryption
	BLEAPP_SECTYPE_SIGNED_NO_MITM,		//!< AES signed encryption without MITM
	BLEAPP_SECTYPE_SIGNED_MITM,			//!< AES signed encryption with MITM
} BLEAPP_SECTYPE;

#define BLEAPP_SECEXCHG_NONE			0
#define BLEAPP_SECEXCHG_KEYBOARD		(1<<0)
#define BLEAPP_SECEXCHG_DISPLAY			(1<<1)
#define BLEAPP_SECEXCHG_OOB				(1<<2)

#define BLEAPP_DEVNAME_MAX_SIZE			BLE_GAP_DEVNAME_DEFAULT_LEN
#define BLEAPP_INFOSTR_MAX_SIZE			20

#define BLEAPP_CONN_CFG_TAG            1     /**< A tag identifying the SoftDevice BLE configuration. */

#pragma pack(push, 4)

/// BLE App Device Info
typedef struct __BleApp_DevInfo {
	const char ModelName[BLEAPP_INFOSTR_MAX_SIZE];	//!< Model name
	const char ManufName[BLEAPP_INFOSTR_MAX_SIZE];	//!< Manufacturer name
	const char *pSerialNoStr;	//!< Serial number string
	const char *pFwVerStr;		//!< Firmware version string
	const char *pHwVerStr;		//!< Hardware version string
} BLEAPP_DEVDESC;

/// BLE App configuration
typedef struct __BleApp_Config {
	nrf_clock_lf_cfg_t ClkCfg;		//!< Clock config
	int CentLinkCount;				//!< Number of central link
	int	PeriLinkCount;				//!< Number of peripheral link
	BLEAPP_MODE AppMode;			//!< App use scheduler, rtos
	const char *pDevName;			//!< Device name
	uint16_t VendorID;				//!< PnP Bluetooth/USB vendor id. iBeacon mode, this is Major value
	uint16_t ProductId;				//!< PnP product ID. iBeacon mode, this is Minor value
	uint16_t ProductVer;			//!< PnP product version
	bool bEnDevInfoService;			//!< Enable device information service (DIS)
	const BLEAPP_DEVDESC *pDevDesc;	//!< Pointer device info descriptor
	const uint8_t *pAdvManData;		//!< Manufacture specific data to advertise
	int AdvManDataLen;				//!< Length of manufacture specific data
	const uint8_t *pSrManData;		//!< Addition Manufacture specific data to advertise in scan response
	int SrManDataLen;				//!< Length of manufacture specific data in scan response
	BLEAPP_SECTYPE SecType;			//!< Secure connection type
	uint8_t SecExchg;				//!< Sec key exchange
	const ble_uuid_t *pAdvUuids;	//!< Service uuids to advertise
	int NbAdvUuid;					//!< Total number of uuids
	uint32_t AdvInterval;			//!< In msec
	uint32_t AdvTimeout;			//!< In sec
	uint32_t AdvSlowInterval;		//!< Slow advertising interval, if > 0, fallback to
									//!< slow interval on adv timeout and advertise until connected
	uint32_t ConnIntervalMin;   	//!< Min. connection interval
	uint32_t ConnIntervalMax;   	//!< Max connection interval
	int8_t ConnLedPort;			//!< Connection LED port number
	int8_t ConnLedPin;				//!< Connection LED pin number
	uint8_t ConnLedActLevel;        //!< Connection LED ON logic level (0: Logic low, 1: Logic high)
	int TxPower;					//!< Tx power in dBm
	uint32_t (*SDEvtHandler)(void); //!< Require for BLEAPP_MODE_RTOS
	int	MaxMtu;						//!< Max MTU size or 0 for default
	int PeriphDevCnt;				//!< Max number of peripheral connection
//	BLEPERIPH_DEV *pPeriphDev;		//!< Connected peripheral data table
} BLEAPP_CFG;

typedef struct __BleApp_Scan_Cfg {
	uint32_t Interval;			//!< Scan interval in msec
	uint32_t Duration;			//!< Scan window in msec
	uint32_t Timeout;			//!< Scan timeout in sec
	ble_uuid128_t BaseUid;		//!< Base UUID to look for
	ble_uuid_t ServUid;			//!< Service Uid to look for
} BLEAPP_SCAN_CFG;

#pragma pack(pop)

#ifdef __cplusplus

class BleApp {
public:
	virtual bool Init(BLEAPP_CFG &CfgData);

	virtual void InitCustomData() = 0;
	virtual void InitServices() = 0;
	virtual void SrvcEvtDispatch(ble_evt_t * p_ble_evt) = 0;

	virtual void ProcessEvt();
	virtual void EnterDfu();
	virtual void Start();

private:
};

extern "C" {
#endif


// ***
// Require implementations per app
//
/**
 * @Brief	User function to initialize any app specific data
 * 	This function is called prio to initializing services
 *
 */
void BleAppInitUserData();

/**
 * @brief	User function to initialize all app services
 * 	This is called before initializing advertisement
 */
void BleAppInitUserServices();

/**
 * @Brief	User peripheral app event handler
 */
void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt);

/**
 * @Brief	User central app event handler
 *
 */
void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt);

//void BleDevServiceDiscovered(uint16_t ConnHdl, uint16_t Count, ble_gattc_service_t * const pServices);

//*** Require implementation if app operating mode is BLEAPP_MODE_RTOS
// This function should normal wait for RTOS to signal an event on sent by
// Softdevice
void BleAppRtosWaitEvt(void);

/**
 * @brief	BLE main App initialization
 *
 * @param	pBleAppCfg : Pointer to app configuration data
 * @param	bEraseBond : true to force erase all bonding info
 *
 * @return	true - success
 */
bool BleAppInit(const BLEAPP_CFG *pBleAppCfg, bool bEraseBond);
void BleAppEnterDfu();
void BleAppRun();
uint16_t BleAppGetConnHandle();
void BleAppGapDeviceNameSet(const char* ppDeviceName);

/**
 *
 * @return	true - advertising
 * 			false - not advertising
 */
bool BleAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen);
void BleAppAdvTimeoutHandler();
void BleAppAdvStart(BLEAPP_ADVMODE AdvMode);
void BleAppAdvStop();
void BleAppDisconnect();

bool BleAppScanInit(BLEAPP_SCAN_CFG *pCfg);
void BleAppScan();
void BleAppScanStop();
bool BleAppConnect(ble_gap_addr_t * const pDevAddr, ble_gap_conn_params_t * const pConnParam);
bool BleAppEnableNotify(uint16_t ConnHandle, uint16_t CharHandle);
bool BleAppWrite(uint16_t ConnHandle, uint16_t CharHandle, uint8_t *pData, uint16_t DatLen);
int8_t GetValidTxPower(int TxPwr);
bool isConnected();

#ifdef __cplusplus
}
#endif

/** @} end group Bluetooth */

#endif // __BLE_APP_H__

