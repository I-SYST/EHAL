/*
 * TPHEnvSrvc.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: hoan
 */
#include <string.h>

#include "app_util_platform.h"

#include "ble_service.h"
#include "TPHThingy.h"


#define BLE_UUID_TES_SERVICE          0x0200

#define BLE_UUID_TES_TEMPERATURE_CHAR 0x0201                      /**< The UUID of the temperature Characteristic. */
#define BLE_UUID_TES_PRESSURE_CHAR    0x0202                      /**< The UUID of the pressure Characteristic. */
#define BLE_UUID_TES_HUMIDITY_CHAR    0x0203                      /**< The UUID of the humidity Characteristic. */
#define BLE_UUID_TES_GAS_CHAR         0x0204                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_COLOR_CHAR       0x0205                      /**< The UUID of the gas Characteristic. */
#define BLE_UUID_TES_CONFIG_CHAR      0x0206                      /**< The UUID of the config Characteristic. */

#define THINGY_TES_CONFIGCHAR_IDX   0
#define THINGY_TES_TEMPCHAR_IDX     1
#define THINGY_TES_PRESCHAR_IDX     2
#define THINGY_TES_HUMICHAR_IDX     3

#define BLE_TES_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3)       /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Thingy Environment service module. */

#define BLE_TES_CONFIG_TEMPERATURE_INT_MIN      100
#define BLE_TES_CONFIG_TEMPERATURE_INT_MAX    60000
#define BLE_TES_CONFIG_PRESSURE_INT_MIN          50
#define BLE_TES_CONFIG_PRESSURE_INT_MAX       60000
#define BLE_TES_CONFIG_HUMIDITY_INT_MIN         100
#define BLE_TES_CONFIG_HUMIDITY_INT_MAX       60000
#define BLE_TES_CONFIG_COLOR_INT_MIN            200
#define BLE_TES_CONFIG_COLOR_INT_MAX          60000
#define BLE_TES_CONFIG_GAS_MODE_MIN               1
#define BLE_TES_CONFIG_GAS_MODE_MAX               3

#pragma pack(push, 1)
typedef struct {
    int8_t  integer;
    uint8_t decimal;
} ble_tes_temperature_t;

typedef struct {
    int32_t  integer;
    uint8_t  decimal;
} ble_tes_pressure_t;

typedef struct {
    uint8_t  led_red;
    uint8_t  led_green;
    uint8_t  led_blue;
} ble_tes_color_config_t;

typedef struct {
    uint16_t                temperature_interval_ms;
    uint16_t                pressure_interval_ms;
    uint16_t                humidity_interval_ms;
    uint16_t                color_interval_ms;
    uint8_t                 gas_interval_mode;
    ble_tes_color_config_t  color_config;
} ble_tes_config_t;

#pragma pack(pop)

void TesAuthReqHandler(BLESRVC *pSrvc, ble_evt_t * p_ble_evt);

static const char s_EnvTempCharDescString[] = {
        "Temperature characteristic",
};

static const char s_EnvPresCharDescString[] = {
        "Pressure characteristic",
};

static const char s_EnvHumCharDescString[] = {
        "Humidity characteristic",
};

/// Characteristic definitions
BLESRVC_CHAR g_EnvChars[] = {
	{
		// Temperature characteristic
		BLE_UUID_TES_CONFIG_CHAR,
		BLE_TES_MAX_DATA_LEN,
		BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_VARLEN,
		s_EnvTempCharDescString,    // char UTF-8 description string
		NULL,                       // Callback for write char, set to NULL for read char
		NULL,                       // Callback on set notification
		NULL,                       // Tx completed callback
		NULL,                       // pointer to char default values
		0,                          // Default value length in bytes
	},
    {
        // Temperature characteristic
        BLE_UUID_TES_TEMPERATURE_CHAR,
        BLE_TES_MAX_DATA_LEN,
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
        s_EnvTempCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0,                          // Default value length in bytes
    },
    {
        // Pressure characteristic
        BLE_UUID_TES_PRESSURE_CHAR, // char UUID
        BLE_TES_MAX_DATA_LEN,       // char max data length
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
        s_EnvPresCharDescString,    // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0                           // Default value length in bytes
    },
    {
        // Humidity characteristic
        BLE_UUID_TES_HUMIDITY_CHAR, // char UUID
        BLE_TES_MAX_DATA_LEN,       // char max data length
        BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
        s_EnvHumCharDescString,     // char UTF-8 description string
        NULL,                       // Callback for write char, set to NULL for read char
        NULL,                       // Callback on set notification
        NULL,                       // Tx completed callback
        NULL,                       // pointer to char default values
        0                           // Default value length in bytes
    },
};

/// Service definition
const BLESRVC_CFG s_EnvSrvcCfg = {
    BLESRVC_SECTYPE_NONE,       // Secure or Open service/char
    THINGY_BASE_UUID,           // Base UUID
    BLE_UUID_TES_SERVICE,       // Service UUID
    sizeof(g_EnvChars) / sizeof(BLESRVC_CHAR),  // Total number of characteristics for the service
    g_EnvChars,                 // Pointer a an array of characteristic
    NULL,                       // pointer to user long write buffer
    0,                          // long write buffer size
	TesAuthReqHandler
};

BLESRVC g_EnvSrvc;

BLESRVC *GetEnvServiceInstance()
{
	return &g_EnvSrvc;
}

uint32_t InitEnvService()
{
    return BleSrvcInit(&g_EnvSrvc, &s_EnvSrvcCfg);
}

void EnvSrvcNotifTemp(float Temp)
{
    ble_tes_temperature_t t;


    t.integer = (int)Temp;
    t.decimal = (uint8_t)((Temp - (float)t.integer) * 100.0);

	BleSrvcCharNotify(GetEnvServiceInstance(), THINGY_TES_TEMPCHAR_IDX, (uint8_t*)&t, sizeof(t));
}

void EnvSrvcNotifPressure(float Press)
{
    ble_tes_pressure_t b;

    b.integer = (int)Press;
    b.decimal = (uint8_t)((Press - (float)b.integer) * 100.0);

	BleSrvcCharNotify(GetEnvServiceInstance(), THINGY_TES_PRESCHAR_IDX, (uint8_t*)&b, sizeof(b));
}

void EnvSrvcNotifHumi(uint8_t Humi)
{
	BleSrvcCharNotify(GetEnvServiceInstance(), THINGY_TES_HUMICHAR_IDX, &Humi, sizeof(Humi));
}

void TesAuthReqHandler(BLESRVC *pSrvc, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_rw_authorize_request_t * p_evt_rw_authorize_request = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    uint32_t err_code;

    if (p_evt_rw_authorize_request->type  == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if (p_evt_rw_authorize_request->request.write.handle == pSrvc->pCharArray[THINGY_TES_CONFIGCHAR_IDX].Hdl.value_handle)
        {
            ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
            bool                                  valid_data = true;

            // Check for valid data
            if(p_evt_rw_authorize_request->request.write.len != sizeof(ble_tes_config_t))
            {
                valid_data = false;
            }
            else
            {
                ble_tes_config_t * p_config = (ble_tes_config_t *)p_evt_rw_authorize_request->request.write.data;

                if ( (p_config->temperature_interval_ms < BLE_TES_CONFIG_TEMPERATURE_INT_MIN)    ||
                     (p_config->temperature_interval_ms > BLE_TES_CONFIG_TEMPERATURE_INT_MAX)    ||
                     (p_config->pressure_interval_ms < BLE_TES_CONFIG_PRESSURE_INT_MIN)          ||
                     (p_config->pressure_interval_ms > BLE_TES_CONFIG_PRESSURE_INT_MAX)          ||
                     (p_config->humidity_interval_ms < BLE_TES_CONFIG_HUMIDITY_INT_MIN)          ||
                     (p_config->humidity_interval_ms > BLE_TES_CONFIG_HUMIDITY_INT_MAX)          ||
                     (p_config->color_interval_ms < BLE_TES_CONFIG_COLOR_INT_MIN)         ||
                     (p_config->color_interval_ms > BLE_TES_CONFIG_COLOR_INT_MAX)         ||
                     (p_config->gas_interval_mode < BLE_TES_CONFIG_GAS_MODE_MIN)                 ||
                     ((int)p_config->gas_interval_mode > (int)BLE_TES_CONFIG_GAS_MODE_MAX))
                {
                    valid_data = false;
                }
            }

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

/*            if ( valid_data && (pSrvc->pCharArray[THINGY_TES_CONFIGCHAR_IDX]->evt_handler != NULL))
            {
                p_tes->evt_handler(p_tes,
                                   BLE_TES_EVT_CONFIG_RECEIVED,
                                   p_evt_rw_authorize_request->request.write.data,
                                   p_evt_rw_authorize_request->request.write.len);
            }*/
        }
    }
}

