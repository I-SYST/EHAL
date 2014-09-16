/*--------------------------------------------------------------------------
File   : UartBleService.c

Author : Hoang Nguyen Hoan          Sep. 13, 2014

Desc   : UART over custom BLE exemple.

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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "istddef.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "device_manager.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_error.h"
#include "app_timer.h"
#include "softdevice_handler.h"
#include "UartBleService.h"


/**@brief Connect event handler.
 *
 * @param[in]   p_blueios    EkoCC Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_uarts_t * p_uarts, ble_evt_t * p_ble_evt)
{
	p_uarts->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


static void on_disconnect(ble_uarts_t * p_uarts, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_uarts->conn_handle = BLE_CONN_HANDLE_INVALID;
}


void ble_uarts_on_ble_evt(ble_uarts_t * p_uarts, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_uarts, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_uarts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
        	{
        		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        		if ((p_evt_write->handle == p_uarts->data_char_handles.value_handle) &&
        	       (p_evt_write->len > 0) &&
        	       (p_uarts->write_handler != NULL))
        		{
        			p_uarts->write_handler(p_uarts, p_evt_write->data, p_evt_write->len);
        		}
        		if ((p_evt_write->handle == p_uarts->ctrl_char_handles.value_handle) &&
         	       (p_evt_write->len > 0) &&
         	       (p_uarts->write_handler != NULL))
        		{
				}
        	}
            break;

        default:
            break;
    }
}

static uint32_t uart_ctrl_char_add(ble_uarts_t * p_uarts, const ble_uarts_init_t * p_uarts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&cccd_md, 0, sizeof(cccd_md));

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 0;
    char_md.char_props.notify = 0;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc  = NULL;//"input report";
    char_md.p_char_pf         = NULL;//"xxxx";
    char_md.p_user_desc_md    = NULL;//&cccd_md;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_uarts->uuid_type;
    ble_uuid.uuid = UART_UUID_CTRL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 0;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 20;//sizeof(BLUEIO_IOPIN) * BLUEIO_IOPIN_MAX;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_uarts->service_handle, &char_md,
                                           &attr_char_value,
                                           &p_uarts->ctrl_char_handles);
}

static uint32_t uart_data_char_add(ble_uarts_t * p_uarts, const ble_uarts_init_t * p_uarts_init)
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
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_uarts->uuid_type;
    ble_uuid.uuid = UART_UUID_DATA_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = 0;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 20;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_uarts->service_handle, &char_md,
                                           &attr_char_value,
                                           &p_uarts->data_char_handles);
}

uint32_t ble_uarts_init(ble_uarts_t * p_uarts, const ble_uarts_init_t * p_uarts_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_uarts->conn_handle	= BLE_CONN_HANDLE_INVALID;
    p_uarts->write_handler 	= p_uarts_init->write_handler;

    // Add base UUID to softdevice's internal list.
    ble_uuid128_t base_uuid = {UART_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_uarts->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_uarts->uuid_type;
    ble_uuid.uuid = UART_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_uarts->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = uart_data_char_add(p_uarts, p_uarts_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = uart_ctrl_char_add(p_uarts, p_uarts_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_uarts_on_data_change(ble_uarts_t * p_uarts)
{
    ble_gatts_hvx_params_t params;
    uint8_t data;
    uint16_t len = sizeof(data);

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_uarts->data_char_handles.value_handle;
    params.p_data = &data;
    params.p_len = &len;

    return sd_ble_gatts_hvx(p_uarts->conn_handle, &params);
}





