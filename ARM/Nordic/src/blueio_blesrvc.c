/*--------------------------------------------------------------------------
File   : blueio_blesrvc.c

Author : Hoang Nguyen Hoan          Mar. 25, 2014

Desc   : Implementation allow the creation of generic custom Bluetooth Smart
		 service with 2 characteristics.
		 	 Read characteristics
		 	 Write characteristics

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
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"

#include "blueio_blesrvc.h"

uint16_t BlueIOBleSrvcCharSend(BLUEIOSRVC *pSrvc, uint8_t *pData, uint16_t DataLen)
{
    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = pSrvc->RdCharHdl.value_handle;
    params.p_data = pData;
    params.p_len = &DataLen;

    if (sd_ble_gatts_hvx(pSrvc->ConnHdl, &params) == NRF_SUCCESS)
    	return DataLen;

    return 0;
}


void BlueIOBleSvcEvtHandler(BLUEIOSRVC *pSrvc, ble_evt_t *pBleEvt)
{
    switch (pBleEvt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	pSrvc->ConnHdl = pBleEvt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	pSrvc->ConnHdl = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
			{
				ble_gatts_evt_write_t * p_evt_write = &pBleEvt->evt.gatts_evt.params.write;

				if ((p_evt_write->handle == pSrvc->RdCharHdl.cccd_handle) &&
					(p_evt_write->len == 2))
				{
					if (ble_srv_is_notification_enabled(p_evt_write->data))
					{
						pSrvc->bNotify = true;
					}
					else
					{
						pSrvc->bNotify = false;
					}
				}
				else if ((p_evt_write->handle == pSrvc->WrCharHdl.value_handle) &&
						 (pSrvc->WrCB != NULL))
				{
					pSrvc->WrCB(pSrvc, p_evt_write->data, 0, p_evt_write->len);
				}
				else
				{
					// Do Nothing. This event is not relevant for this service.
				}
			}
            break;

        default:
            break;
    }
}

static void BlueIOBleSrvcEncSec(ble_gap_conn_sec_mode_t *pSecMode, BLUEIOSRVC_SECTYPE SecType)
{
	switch (SecType)
    {
		case BLUEIOSRVC_SECTYPE_STATICKEY_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(pSecMode);
			break;
		case BLUEIOSRVC_SECTYPE_STATICKEY_MITM:
	    	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(pSecMode);
	    	break;
		case BLUEIOSRVC_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(pSecMode);
			break;
		case BLUEIOSRVC_SECTYPE_SIGNED_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(pSecMode);
			break;
		case BLUEIOSRVC_SECTYPE_SIGNED_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(pSecMode);
			break;
    	case BLUEIOSRVC_SECTYPE_NONE:
    	default:
    		BLE_GAP_CONN_SEC_MODE_SET_OPEN(pSecMode);
    		break;
    }
}

/**@brief Add control characteristic.
 *
 * @param[in]   p_blueios        LEDButton Service structure.
 * @param[in]   p_iodata_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t BlueIOBleSrvcCharAdd(BLUEIOSRVC *pSvc, uint16_t CharUuid,
								 	int MaxDataLen, uint32_t CharProp, const char *pDesc,
									ble_gatts_char_handles_t *pCharHdl, BLUEIOSRVC_SECTYPE SecType)
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

    char_md.p_char_user_desc  = (uint8_t*)pDesc;
    if (pDesc != NULL)
    	char_md.char_user_desc_max_size = strlen(pDesc) + 1;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    if (CharProp & BLUEIOSVC_CHAR_PROP_NOTIFY)
    {
    	char_md.char_props.notify = 1;
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    	BlueIOBleSrvcEncSec(&cccd_md.write_perm, SecType);
    	BlueIOBleSrvcEncSec(&attr_md.read_perm, SecType);
        char_md.p_cccd_md         = &cccd_md;
    }

    if (CharProp & BLUEIOSVC_CHAR_PROP_READ)
    {
    	char_md.char_props.read   = 1;
    	BlueIOBleSrvcEncSec(&attr_md.read_perm, SecType);
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    }

    if (CharProp & BLUEIOSVC_CHAR_PROP_WRITE)
    {
    	char_md.char_props.write  = 1;
    	BlueIOBleSrvcEncSec(&attr_md.write_perm, SecType);
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    }

    if (CharProp & BLUEIOSVC_CHAR_PROP_WRITEWORESP)
	{
		char_md.char_props.write_wo_resp = 1;
    	BlueIOBleSrvcEncSec(&attr_md.write_perm, SecType);
		BlueIOBleSrvcEncSec(&attr_md.read_perm, SecType);
	}

    ble_uuid.type = pSvc->UuidType;
    ble_uuid.uuid = CharUuid;



    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;	// Variable length

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 0;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MaxDataLen;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(pSvc->SvcHdl, &char_md, &attr_char_value, pCharHdl);
}

uint32_t BlueIOBleSrvcInit(BLUEIOSRVC *pSvc, const BLUEIOSRVC_CFG *pCfg)
{
    uint32_t   err;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    pSvc->ConnHdl  = BLE_CONN_HANDLE_INVALID;
    pSvc->WrCB = pCfg->WrCB;

    // Add base UUID to softdevice's internal list.
    err = sd_ble_uuid_vs_add(&pCfg->UuidBase, &pSvc->UuidType);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    ble_uuid.type = pSvc->UuidType;
    ble_uuid.uuid = pCfg->UuidSvc;

    err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &pSvc->SvcHdl);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    err = BlueIOBleSrvcCharAdd(pSvc, pCfg->UuidWrChar, pCfg->WrCharMaxLen,
    					   	   pCfg->WrCharProp, pCfg->pWrCharDesc,
							   &pSvc->WrCharHdl, pCfg->SecType);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    err = BlueIOBleSrvcCharAdd(pSvc, pCfg->UuidRdChar, pCfg->RdCharMaxLen,
    			 	 	   	   pCfg->RdCharProp, pCfg->pRdCharDesc,
							   &pSvc->RdCharHdl, pCfg->SecType);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    return NRF_SUCCESS;
}


