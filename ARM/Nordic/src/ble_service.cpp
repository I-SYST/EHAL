/**-------------------------------------------------------------------------
@file	ble_service.c

@brief	Implement Bluetooth LE service and characteristic

Implementation allow the creation of generic custom Bluetooth Smart service
with multiple user defined characteristics.

This implementation is to be used with Nordic SDK

@author	Hoang Nguyen Hoan
@date	Mar. 25, 2014

@license

MIT License

Copyright (c) 2014-2020 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"
#include "ble_service.h"

#pragma pack(push, 1)
typedef struct {
	uint16_t Handle;
	uint16_t Offset;
	uint16_t Len;
} GATLWRHDR;

typedef struct {
	GATLWRHDR Hdr;
	uint8_t Data[1];
} GETTLWRMEM;

#pragma pack(pop)

uint32_t BleSrvcCharNotify(BLESRVC *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
{
	if (pSrvc->ConnHdl == BLE_CONN_HANDLE_INVALID)
		return NRF_ERROR_INVALID_STATE;

	if (pSrvc->pCharArray[Idx].bNotify == false)
		return NRF_ERROR_INVALID_STATE;

    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = pSrvc->pCharArray[Idx].Hdl.value_handle;
    params.p_data = pData;
    params.p_len = &DataLen;

    uint32_t err_code = sd_ble_gatts_hvx(pSrvc->ConnHdl, &params);

    return err_code;
}

uint32_t BleSrvcCharSetValue(BLESRVC *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
{
	ble_gatts_value_t value;

    memset(&value, 0, sizeof(ble_gatts_value_t));

    value.offset = 0;
    value.len = DataLen;
    value.p_value = pData;

    uint32_t err_code = sd_ble_gatts_value_set(pSrvc->ConnHdl,
    										   pSrvc->pCharArray[Idx].Hdl.value_handle,
											   &value);
    return err_code;
}

void GatherLongWrBuff(GATLWRHDR *pHdr)
{
	uint8_t *p = (uint8_t*)pHdr + pHdr->Len + sizeof(GATLWRHDR);
	GATLWRHDR *hdr = (GATLWRHDR*)p;
	if (hdr->Handle == pHdr->Handle)
	{
		GatherLongWrBuff(hdr);
		pHdr->Len += hdr->Len;
		memcpy(p, p + sizeof(GATLWRHDR), hdr->Len);
	}
}

void BleSrvcEvtHandler(BLESRVC *pSrvc, ble_evt_t *pBleEvt)
{
    switch (pBleEvt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	pSrvc->ConnHdl = pBleEvt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	pSrvc->ConnHdl = BLE_CONN_HANDLE_INVALID;
        	for (int i = 0; i < pSrvc->NbChar; i++)
        	{
        		pSrvc->pCharArray[i].bNotify = false;
        	}
            break;

        case BLE_GATTS_EVT_WRITE:
			{
				ble_gatts_evt_write_t * p_evt_write = &pBleEvt->evt.gatts_evt.params.write;

				for (int i = 0; i < pSrvc->NbChar; i++)
				{
				    if (p_evt_write->op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
					//if (p_evt_write->handle == 0)
					{
						//printf("Long Write\r\n");
						GATLWRHDR *hdr = (GATLWRHDR *)pSrvc->pLongWrBuff;
					    uint8_t *p = (uint8_t*)pSrvc->pLongWrBuff + sizeof(GATLWRHDR);
						if (hdr->Handle == pSrvc->pCharArray[i].Hdl.value_handle)
					    {
#if 1
							GatherLongWrBuff(hdr);
							pSrvc->pCharArray[i].WrCB(pSrvc, p, hdr->Offset, hdr->Len);
#else
							bool done = false;
							GATLWRHDR hdr1;
							uint8_t *p1 = p + hdr->Len;
							memcpy(&hdr1, p1, sizeof(GATLWRHDR));

							while (hdr1.Handle == hdr->Handle)
							{
								p1 += sizeof(GATLWRHDR);
								memcpy(&p[hdr->Len], p1, pSrvc->LongWrBuffSize - hdr->Len - sizeof(GATLWRHDR));
								hdr->Len += hdr1.Len;
								p1 = p + hdr->Len;
								memcpy(&hdr1, p1, sizeof(GATLWRHDR));
							}
							pSrvc->pCharArray[i].WrCB(pSrvc, p, hdr->Offset, hdr->Len);
#endif
					    }
					}
					else
					{
						if ((p_evt_write->handle == pSrvc->pCharArray[i].Hdl.cccd_handle) &&
							(p_evt_write->len == 2))
						{
							if (ble_srv_is_notification_enabled(p_evt_write->data))
							{
								pSrvc->pCharArray[i].bNotify = true;
							}
							else
							{
								pSrvc->pCharArray[i].bNotify = false;
							}
							// Set notify callback
							if (pSrvc->pCharArray[i].SetNotifCB)
							{
								pSrvc->pCharArray[i].SetNotifCB(pSrvc, pSrvc->pCharArray[i].bNotify);
							}
						}
						else if ((p_evt_write->handle == pSrvc->pCharArray[i].Hdl.value_handle) &&
								 (pSrvc->pCharArray[i].WrCB != NULL))
						{
							//printf("Write value handle\r\n");
							pSrvc->pCharArray[i].WrCB(pSrvc, p_evt_write->data, 0, p_evt_write->len);
						}
						else
						{
							// Do Nothing. This event is not relevant for this service.
						}
					}
				}
			}
            break;

        case BLE_EVT_USER_MEM_REQUEST:
        	{
        		if (pSrvc->pLongWrBuff != NULL && pSrvc->ConnHdl == pBleEvt->evt.gatts_evt.conn_handle)
        		{
					ble_user_mem_block_t mblk;
					memset(&mblk, 0, sizeof(ble_user_mem_block_t));
					mblk.p_mem = pSrvc->pLongWrBuff;
					mblk.len = pSrvc->LongWrBuffSize;
					memset(pSrvc->pLongWrBuff, 0, pSrvc->LongWrBuffSize);
					uint32_t err_code = sd_ble_user_mem_reply(pSrvc->ConnHdl, &mblk);
					APP_ERROR_CHECK(err_code);
        		}
        	}
        	break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	if (pSrvc->AuthReqCB)
        	{
        		pSrvc->AuthReqCB(pSrvc, pBleEvt);
        	}
        	break;

#if (NRF_SD_BLE_API_VERSION > 3)
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
#else
        case BLE_EVT_TX_COMPLETE:

#endif
            if (pSrvc->ConnHdl == pBleEvt->evt.gatts_evt.conn_handle)
            {
                for (int i = 0; i < pSrvc->NbChar; i++)
                {
                    //if (pBleEvt->evt.gatts_evt.params.hvc.handle == pSrvc->pCharArray[i].Hdl.value_handle &&
                    if (pSrvc->pCharArray[i].TxCompleteCB != NULL)
                    {
                        pSrvc->pCharArray[i].TxCompleteCB(pSrvc, i);
                    }
                }
            }
            break;

        default:
            break;
    }
}

static void BleSrvcEncSec(ble_gap_conn_sec_mode_t *pSecMode, BLESRVC_SECTYPE SecType)
{
	switch (SecType)
    {
		case BLESRVC_SECTYPE_STATICKEY_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(pSecMode);
			break;
		case BLESRVC_SECTYPE_STATICKEY_MITM:
	    	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(pSecMode);
	    	break;
		case BLESRVC_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(pSecMode);
			break;
		case BLESRVC_SECTYPE_SIGNED_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(pSecMode);
			break;
		case BLESRVC_SECTYPE_SIGNED_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(pSecMode);
			break;
    	case BLESRVC_SECTYPE_NONE:
    	default:
    		BLE_GAP_CONN_SEC_MODE_SET_OPEN(pSecMode);
    		break;
    }
}

/**
 * @brief Add control characteristic.
 *
 * @param[in]   	pSrvc   : Service data.
 * @param[in/out]   pChar   : characteristic to initialize.
 * @param[in]		SecType : Security type
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t BlueIOBleSrvcCharAdd(BLESRVC *pSrvc, BLESRVC_CHAR *pChar,
									 BLESRVC_SECTYPE SecType)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&char_md, 0, sizeof(char_md));

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    char_md.p_char_user_desc  = (uint8_t*)pChar->pDesc;
    if (pChar->pDesc != NULL)
    {
    	char_md.char_user_desc_max_size = strlen(pChar->pDesc) + 1;
    	char_md.char_user_desc_size = strlen(pChar->pDesc) + 1;
    }
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    if (pChar->Property & BLESVC_CHAR_PROP_NOTIFY)
    {
    	char_md.char_props.notify = 1;
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    	BleSrvcEncSec(&cccd_md.write_perm, SecType);
    	BleSrvcEncSec(&attr_md.read_perm, SecType);
        char_md.p_cccd_md         = &cccd_md;
    }

    if (pChar->Property & BLESVC_CHAR_PROP_READ)
    {
    	char_md.char_props.read   = 1;
    	BleSrvcEncSec(&attr_md.read_perm, SecType);
       // BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    }

    if (pChar->Property & (BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_WRITEWORESP))
    {
        if (pChar->Property & BLESVC_CHAR_PROP_WRITE)
            char_md.char_props.write  = 1;
    	if (pChar->Property & BLESVC_CHAR_PROP_WRITEWORESP)
            char_md.char_props.write_wo_resp = 1;

    	BleSrvcEncSec(&attr_md.write_perm, SecType);
        //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    }

    ble_uuid.type = pSrvc->UuidType[pChar->BaseUuidIdx];
    ble_uuid.uuid = pChar->Uuid;

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

    if (pChar->Property & BLESVC_CHAR_PROP_RDAUTH)
    {
    	attr_md.rd_auth    = 1;
    }
    else
    {
    	attr_md.rd_auth    = 0;
    }

    if (pChar->Property & BLESVC_CHAR_PROP_WRAUTH)
    {
    	attr_md.wr_auth    = 1;
    }
    else
    {
    	attr_md.wr_auth    = 0;
    }

    if (pChar->Property & BLESVC_CHAR_PROP_VARLEN)
    {
    	attr_md.vlen       = 1;	// Variable length
    }
    else
    {
    	attr_md.vlen       = 0;	// Fixed length
    }

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = pChar->MaxDataLen;
    attr_char_value.init_len     = pChar->ValueLen;
    attr_char_value.p_value      = pChar->pDefValue;

    return sd_ble_gatts_characteristic_add(pSrvc->SrvcHdl, &char_md, &attr_char_value, &pChar->Hdl);
}

/**
 * @brief Create BLE service
 *
 * @param [in/out]	pSrvc : Service handle
 * @param [in]		pCfg  : Service configuration data
 */
uint32_t BleSrvcInit(BLESRVC *pSrvc, const BLESRVC_CFG *pCfg)
{
    uint32_t   err;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    pSrvc->ConnHdl  = BLE_CONN_HANDLE_INVALID;

    // Add base UUID to softdevice's internal list.
    for (int i = 0; i < pCfg->NbUuidBase; i++)
    {
		err = sd_ble_uuid_vs_add(&pCfg->UuidBase[i], &pSrvc->UuidType[i]);
		if (err != NRF_SUCCESS)
		{
			return err;
		}
    }

    ble_uuid.type = pSrvc->UuidType[0];
    ble_uuid.uuid = pCfg->UuidSvc;

    err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &pSrvc->SrvcHdl);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    pSrvc->NbChar = pCfg->NbChar;

    pSrvc->pCharArray = pCfg->pCharArray;

    for (int i = 0; i < pCfg->NbChar; i++)
    {
    	err = BlueIOBleSrvcCharAdd(pSrvc, &pSrvc->pCharArray[i],
								   pCfg->SecType);
        if (err != NRF_SUCCESS)
        {
            return err;
        }
        pSrvc->pCharArray[i].bNotify = false;
    }

    pSrvc->pLongWrBuff = pCfg->pLongWrBuff;
    pSrvc->LongWrBuffSize = pCfg->LongWrBuffSize;
    pSrvc->AuthReqCB = pCfg->AuthReqCB;

    return NRF_SUCCESS;
}


