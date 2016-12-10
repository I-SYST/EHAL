/*--------------------------------------------------------------------------
File   : BlueIOBLEService.c

Author : Hoang Nguyen Hoan          Mar. 25, 2014

Desc   : Bluetooth Low Energy BlueIO Services implementation

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


//uint8_t g_GatWriteBuff[512];

/**@brief Connect event handler.
 *
 * @param[in]   p_blueios    EkoCC Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
/*static void on_connect(ble_blueios_t * p_blueios, ble_evt_t * p_ble_evt)
{
	p_blueios->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}*/


/**@brief Disconnect event handler.
 *
 * @param[in]   p_blueios       LEDButton Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
/*static void on_disconnect(ble_blueios_t * p_blueios, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_blueios->conn_handle = BLE_CONN_HANDLE_INVALID;
}
*/


/*
void ble_blueios_on_ble_evt(ble_blueios_t * p_blueios, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_blueios, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_blueios, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_blueios, p_ble_evt);
            break;

        default:
            break;
    }
}

*/


/**@brief Add control characteristic.
 *
 * @param[in]   p_blueios        LEDButton Service structure.
 * @param[in]   p_iodata_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t BlueIOSvcCharAdd(BLUEIOSVC *pSvc, uint16_t CharUuid,
								 int MaxDataLen, ble_gatts_char_md_t *pCharMd,
								 ble_gatts_char_handles_t *pCharHdl, bool bSecur)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&attr_md, 0, sizeof(attr_md));

    if (bSecur)
    {
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&cccd_md.write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&attr_md.write_perm);
    }
    else
    {
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    }
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memcpy(&char_md, pCharMd, sizeof(char_md));
/*
    if (CharProp & BLUEIOSVC_CHAR_PROP_READ)
    	char_md.char_props.read   = 1;
    if (CharProp & BLUEIOSVC_CHAR_PROP_WRITE)
    	char_md.char_props.write  = 1;
    if (CharProp & BLUEIOSVC_CHAR_PROP_NOTIFY)
    	char_md.char_props.notify = 1;
    if (CharProp & BLUEIOSVC_CHAR_PROP_WRITEWORESP)
    	char_md.char_props.write_wo_resp = 1;

    char_md.p_char_user_desc  = NULL;//"input report";
    char_md.p_char_pf         = NULL;//"xxxx";
    char_md.p_user_desc_md    = NULL;//&cccd_md;
*/
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = pSvc->UuidType;
    ble_uuid.uuid = CharUuid;



    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 0;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MaxDataLen;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(pSvc->SvcHdl, &char_md, &attr_char_value, pCharHdl);
}
/*
static uint32_t DataCharAdd(BLUEIOSVC *pSvc, const BLUEIOSVC_CFG *pCfg)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc  = NULL;//"input report";
    char_md.p_char_pf         = NULL;//"xxxx";
    char_md.p_user_desc_md    = NULL;//&cccd_md;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = pSvc->UuidType;
    ble_uuid.uuid = pCfg->UuidDataChar;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 0;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 512;//sizeof(BLUEIO_IOPIN) * BLUEIO_IOPIN_MAX;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(pSvc->SvcHdl, &char_md,
                                           &attr_char_value,
                                           &pSvc->DataCharHdl);
}
*/
uint32_t BlueIOSvcInit(BLUEIOSVC *pSvc, const BLUEIOSVC_CFG *pCfg)
{
    uint32_t   err;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    pSvc->ConnHdl  = BLE_CONN_HANDLE_INVALID;
    pSvc->CtrlWrCB = pCfg->CtrlWrCB;
    pSvc->TxDataWrCB = pCfg->TxDataWrCB;

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

    err = BlueIOSvcCharAdd(pSvc, pCfg->UuidCtrlChar, pCfg->CtrlCharMaxLen,
    					   (ble_gatts_char_md_t*)&pCfg->CtrlChar,
						   &pSvc->CtrlCharHdl, false);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    err = BlueIOSvcCharAdd(pSvc, pCfg->UuidRxDataChar, pCfg->RxDataCharMaxLen,
    			 	 	   (ble_gatts_char_md_t*)&pCfg->RxDataChar,
						   &pSvc->RxDataCharHdl, false);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    err = BlueIOSvcCharAdd(pSvc, pCfg->UuidTxDataChar, pCfg->TxDataCharMaxLen,
    			 	 	   (ble_gatts_char_md_t*)&pCfg->TxDataChar,
						   &pSvc->TxDataCharHdl, false);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    return NRF_SUCCESS;
}
/*
uint32_t ble_blueios_on_data_change(ble_blueios_t * p_blueios)
{
    ble_gatts_hvx_params_t params;
    uint8_t data;
    uint16_t len = sizeof(data);

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_blueios->data_char_handles.value_handle;
    params.p_data = &data;
    params.p_len = &len;

    return sd_ble_gatts_hvx(p_blueios->conn_handle, &params);
}
*/


/**@brief Function for handling a Bond Manager error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
//static void bond_manager_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}


/**@brief Function for handling the Bond Manager events.
 *
 * @param[in]   p_evt   Data associated to the bond manager event.
 */
/*static void bond_evt_handler(ble_bondmngr_evt_t * p_evt)
{
	g_LastConnectedCentral = p_evt->central_handle;
}
*/
/**@brief Function for the Bond Manager initialization.
 */
/*static void bond_manager_init(void)
{
    uint32_t            err_code;
    ble_bondmngr_init_t bondmngrcfg;
    //bool                bonds_delete;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed
    //err_code = app_button_is_pushed(BONDMNGR_DELETE_BUTTON_PIN_NO, &bonds_delete);
    //APP_ERROR_CHECK(err_code);

    // Initialize the Bond Manager
    bondmngrcfg.flash_page_num_bond     = FLASH_PAGE_BOND;
    bondmngrcfg.flash_page_num_sys_attr = FLASH_PAGE_SYS_ATTR;
    bondmngrcfg.evt_handler             = bond_evt_handler;
    bondmngrcfg.error_handler           = bond_manager_error_handler;
    bondmngrcfg.bonds_delete            = false;//bonds_delete;

    err_code = ble_bondmngr_init(&bondmngrcfg);
    APP_ERROR_CHECK(err_code);
}
*/

