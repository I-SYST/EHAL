/*
 * TPHConfSrvc.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: hoan
 */
#include <string.h>

#include "app_util_platform.h"

#include "ble_service.h"
#include "TPHThingy.h"


#define BLE_UUID_TCS_DEVICE_NAME_CHAR   0x0101                      /**< The UUID of the device name Characteristic. */
#define BLE_UUID_TCS_ADV_PARAMS_CHAR    0x0102                      /**< The UUID of the advertising parameters Characteristic. */
#define BLE_UUID_TCS_APPEARANCE_CHAR    0x0103                      /**< The UUID of the appearance Characteristic. */
#define BLE_UUID_TCS_CONN_PARAM_CHAR    0x0104                      /**< The UUID of the connection parameters Characteristic. */
#define BLE_UUID_TCS_BEACON_PARAM_CHAR  0x0105                      /**< The UUID of the beacon Characteristic. */
#define BLE_UUID_TCS_CLOUD_PARAM_CHAR   0x0106                      /**< The UUID of the cloud token Characteristic. */
#define BLE_UUID_TCS_FW_VERSION_CHAR    0x0107                      /**< The UUID of the FW version Characteristic. */
#define BLE_UUID_TCS_MTU_CHAR           0x0108                      /**< The UUID of the MTU Characteristic. */

//#define THINGY_TCS_DEVICE_NAME_CHAR_IDX		0
//#define THINGY_TCS_ADV_PARAMS_CHAR_IDX     	1
//#define THINGY_TCS_APPEARANCE_CHAR_IDX     	2
//#define THINGY_TCS_CONN_PARAM_CHAR_IDX     	2
//#define THINGY_TCS_BEACON_PARAM_CHAR_IDX   	3
//#define THINGY_TCS_CLOUD_PARAM_CHAR_IDX   	4
#define THINGY_TCS_FW_VERSIO_CHAR_IDX   	0//5
//#define THINGY_TCS_MTU_CHAR_IDX   			4//6

#define BLE_TCS_DEVICE_NAME_LEN_MAX 10
#define BLE_TCS_BEACON_LEN_MAX      17
#define BLE_TCS_BEACON_LEN_MIN       3
#define BLE_TCS_CLOUD_LEN_MAX      250
#define BLE_TCS_CLOUD_LEN_MIN        0

#define BLE_TCS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Configuration service module. */

void ConfAuthReqHandler(BLESRVC *pBleSvc, ble_evt_t * p_ble_evt);
void ConfWrDevNameHandler(BLESRVC *pBleSvc, uint8_t *pData, int Offset, int Len);

#pragma pack(push, 1)
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} ble_tcs_fw_version_t;

typedef struct {
    uint8_t  req;
    uint16_t size;
} ble_tcs_mtu_t;
#pragma pack(pop)

const ble_tcs_fw_version_t s_ThingyVersion {
	 2, 1, 0
};

const ble_tcs_mtu_t s_ThingyMtu = {
	23, 153
};

/// Characteristic definitions
BLESRVC_CHAR g_ConfChars[] = {
/*    {
        // Device name characteristic
    	BLE_UUID_TCS_DEVICE_NAME_CHAR,
        BLE_TCS_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_VARLEN | BLESVC_CHAR_PROP_WRAUTH,
        NULL,    					// char UTF-8 description string
		ConfWrDevNameHandler,        // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
		(uint8_t*)DEVICE_NAME,                // pointer to char default values
        strlen(DEVICE_NAME),        // Default value length in bytes
    },
    {
        // Advertisement characteristic
		BLE_UUID_TCS_ADV_PARAMS_CHAR,
        BLE_TCS_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_VARLEN | BLESVC_CHAR_PROP_WRAUTH,
        NULL,    					// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0,                          // Default value length in bytes
    },
    {
        // Connection characteristic
		THINGY_TCS_CONN_PARAM_CHAR_IDX,
        BLE_TCS_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_VARLEN | BLESVC_CHAR_PROP_WRAUTH,
        NULL,    					// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0,                          // Default value length in bytes
    },*/
    {
        // Advertisement characteristic
		BLE_UUID_TCS_FW_VERSION_CHAR,
        BLE_TCS_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_VARLEN,
        NULL,    					// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
		(uint8_t*)&s_ThingyVersion,                       // pointer to char default values
        3,                          // Default value length in bytes
    },
/*    {
        // Advertisement characteristic
		BLE_UUID_TCS_MTU_CHAR,
        BLE_TCS_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_VARLEN | BLESVC_CHAR_PROP_WRAUTH,
        NULL,    					// char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
		(uint8_t*)&s_ThingyMtu,                       // pointer to char default values
        3,                          // Default value length in bytes
    },*/
};

/// Service definition
const BLESRVC_CFG s_ConfSrvcCfg = {
    BLESRVC_SECTYPE_NONE,       	// Secure or Open service/char
    THINGY_BASE_UUID,           	// Base UUID
    BLE_UUID_TCS_SERVICE,       	// Service UUID
    sizeof(g_ConfChars) / sizeof(BLESRVC_CHAR),  // Total number of characteristics for the service
    g_ConfChars,                 	// Pointer a an array of characteristic
    NULL,                       	// pointer to user long write buffer
    0,                           	// long write buffer size
	ConfAuthReqHandler
};

BLESRVC g_ConfSrvc;

BLESRVC *GetConfServiceInstance()
{
	return &g_ConfSrvc;
}

uint32_t InitConfService()
{
    return BleSrvcInit(&g_ConfSrvc, &s_ConfSrvcCfg);
}

void ConfWrDevNameHandler(BLESRVC *pBleSvc, uint8_t *pData, int Offset, int Len)
{

}

void ConfAuthReqHandler(BLESRVC *pBleSvc, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_rw_authorize_request_t * p_evt_rw_authorize_request = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    uint32_t err_code;

    if (p_evt_rw_authorize_request->type  == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
        bool                                  valid_data = true;
        bool                                  reply = true;
//        ble_tcs_evt_type_t                    evt_type = BLE_TCS_EVT_DEV_NAME;
/*
        if (p_evt_rw_authorize_request->request.write.handle == pBleSvc->pCharArray[THINGY_TCS_DEVICE_NAME_CHAR_IDX].Hdl.value_handle)
        {
            // Check for valid data
            if(p_evt_rw_authorize_request->request.write.len > BLE_TCS_DEVICE_NAME_LEN_MAX)
            {
                valid_data = false;
            }
        }
        else if (p_evt_rw_authorize_request->request.write.handle == pBleSvc->pCharArray[THINGY_TCS_ADV_PARAMS_CHAR_IDX].Hdl.value_handle)
        {
            // Check for valid data
            if(p_evt_rw_authorize_request->request.write.len != sizeof(ble_tcs_adv_params_t))
            {
                valid_data = false;
            }
            else
            {
                ble_tcs_adv_params_t * p_data = (ble_tcs_adv_params_t *)p_evt_rw_authorize_request->request.write.data;

                evt_type = BLE_TCS_EVT_ADV_PARAM;

                if ( (p_data->interval  < TCS_ADV_PARAMS_INTERVAL_MIN)    ||
                     (p_data->interval  > TCS_ADV_PARAMS_INTERVAL_MAX)    ||
                     (p_data->timeout  > TCS_ADV_PARAMS_TIMEOUT_MAX))
                {
                    valid_data = false;
                }
            }
        }
        else if (p_evt_rw_authorize_request->request.write.handle == p_tcs->conn_param_handles.value_handle)
        {
            // Check for valid data
            if(p_evt_rw_authorize_request->request.write.len != sizeof(ble_tcs_conn_params_t))
            {
                valid_data = false;
            }
            else
            {
                ble_tcs_conn_params_t * p_data = (ble_tcs_conn_params_t *)p_evt_rw_authorize_request->request.write.data;

                evt_type = BLE_TCS_EVT_CONN_PARAM;

                if ( (p_data->min_conn_int  < BLE_GAP_CP_MIN_CONN_INTVL_MIN)    ||
                     (p_data->min_conn_int  > BLE_GAP_CP_MIN_CONN_INTVL_MAX)    ||
                     (p_data->max_conn_int  < BLE_GAP_CP_MAX_CONN_INTVL_MIN)    ||
                     (p_data->max_conn_int  > BLE_GAP_CP_MAX_CONN_INTVL_MAX)    ||
                     (p_data->slave_latency > BLE_GAP_CP_SLAVE_LATENCY_MAX)     ||
                     (p_data->sup_timeout   < BLE_GAP_CP_CONN_SUP_TIMEOUT_MIN)  ||
                     (p_data->sup_timeout   > BLE_GAP_CP_CONN_SUP_TIMEOUT_MAX))
                {
                    valid_data = false;
                }

              // If both conn_sup_timeout and max_conn_interval are specified, then the following constraint applies:
              //   conn_sup_timeout * 4 > (1 + slave_latency) * max_conn_interval
              //   that corresponds to the following Bluetooth Spec requirement:
              //   The Supervision_Timeout in milliseconds shall be larger than
              //   (1 + Conn_Latency) * Conn_Interval_Max * 2, where Conn_Interval_Max is given in milliseconds.

                if ( (p_data->sup_timeout * 4) <= ((1 + p_data->slave_latency) * p_data->max_conn_int) )
                {
                    valid_data = false;
                }
            }
        }
        else if (p_evt_rw_authorize_request->request.write.handle == p_tcs->beacon_handles.value_handle)
        {
            evt_type = BLE_TCS_EVT_BEACON;

            if (p_evt_rw_authorize_request->request.write.len > 0)
            {
            // Check for valid data
                if( (p_evt_rw_authorize_request->request.write.len > BLE_TCS_BEACON_LEN_MAX) ||
                    (p_evt_rw_authorize_request->request.write.len < BLE_TCS_BEACON_LEN_MIN))
                {
                    valid_data = false;
                }
            }
        }
        else if (p_evt_rw_authorize_request->request.write.handle == p_tcs->cloud_handles.value_handle)
        {
            evt_type = BLE_TCS_EVT_CLOUD_TOKEN;

            // Check for valid data
            if(p_evt_rw_authorize_request->request.write.len > BLE_TCS_CLOUD_LEN_MAX)
            {
                valid_data = false;
            }
        }
        else if (p_evt_rw_authorize_request->request.write.handle == p_tcs->mtu_handles.value_handle)
        {
            // Check for valid data
            if(p_evt_rw_authorize_request->request.write.len != sizeof(ble_tcs_mtu_t))
            {
                valid_data = false;
            }
            else
            {
                ble_tcs_mtu_t * p_data = (ble_tcs_mtu_t *)p_evt_rw_authorize_request->request.write.data;

                evt_type = BLE_TCS_EVT_MTU;

                if ( //(p_data->req   < TCS_MTU_REQ_MIN)      ||
                     (p_data->req   > TCS_MTU_REQ_MAX)      ||
                     (p_data->size  < TCS_MTU_SIZE_MIN)     ||
                     (p_data->size  > TCS_MTU_SIZE_MAX))
                {
                    valid_data = false;
                }
            }
        }

        else
        {
            valid_data = false;
            reply = false;
        }
*/
        if (reply)
        {
            // Reply depending on valid data or not
            rw_authorize_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;

            if (valid_data)
            {
                rw_authorize_reply.params.write.update      = 1;
                rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                rw_authorize_reply.params.write.p_data      = p_evt_rw_authorize_request->request.write.data;
                rw_authorize_reply.params.write.len         = p_evt_rw_authorize_request->request.write.len;
                rw_authorize_reply.params.write.offset      = p_evt_rw_authorize_request->request.write.offset;
            }
            else
            {
                rw_authorize_reply.params.write.update      = 0;
                rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
            }

            err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       &rw_authorize_reply);
            APP_ERROR_CHECK(err_code);
        }

        // Call event handler
/*        if ( valid_data && (p_tcs->evt_handler != NULL))
        {
            p_tcs->evt_handler(p_tcs,
                               evt_type,
                               p_evt_rw_authorize_request->request.write.data,
                               p_evt_rw_authorize_request->request.write.len);
        }*/
    }
}



