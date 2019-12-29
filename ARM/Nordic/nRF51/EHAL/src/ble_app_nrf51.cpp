/*--------------------------------------------------------------------------
File   : ble_app.cpp

Author : Hoang Nguyen Hoan          Dec 26, 2016

Desc   : Nordic SDK based BLE peripheral application creation helper

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <inttypes.h>
#include <atomic>

#include "nordic_common.h"
#include "ble_hci.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "ble_gatt.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "nrf_ble_gatt.h"
#include "peer_manager.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "app_scheduler.h"
#include "fds.h"
extern "C" {
#include "softdevice_handler_appsh.h"
}
#include "fstorage.h"
#include "nrf_dfu_settings.h"
#include "nrf_crypto.h"

#include "app_timer_appsh.h"

#include "istddef.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "ble_app.h"

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define APP_TIMER_PRESCALER				0
#define APP_TIMER_OP_QUEUE_SIZE         10                                           /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE sizeof(app_timer_event_t) /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE          		10                        /**< Maximum number of events in the scheduler queue. */
#endif

//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */

#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// ORable application role
#define BLEAPP_ROLE_PERIPHERAL			1
#define BLEAPP_ROLE_CENTRAL				2

#pragma pack(push, 4)

typedef struct __BleApp_PeripheralData {
	uint16_t ConnHdl;	// Connection handle
    uint8_t SrvcCnt;		// Number of services
    ble_gatt_db_srv_t Srvc[BLE_DB_DISCOVERY_MAX_SRV];  // service data
} BLEAPP_PERIPH;

typedef struct _BleAppData {
	BLEAPP_MODE AppMode;
	int AppRole;
	uint16_t ConnHdl;	// BLE connection handle
	int ConnLedPort;
	int ConnLedPin;
	int PeriphDevCnt;
	BLEAPP_PERIPH *pPeriphDev;
	ble_advdata_t AdvData;
	ble_advdata_t SRData;
	ble_adv_modes_config_t AdvConf;
} BLEAPP_DATA;

#pragma pack(pop)

static ble_gap_adv_params_t s_AdvParams;                                 /**< Parameters to be passed to the stack when starting advertising. */
static ble_advdata_manuf_data_t s_BleAppManData;

BLEAPP_DATA g_BleAppData = {
	BLEAPP_MODE_LOOP, 0, BLE_CONN_HANDLE_INVALID, -1, -1
};

pm_peer_id_t g_PeerMngrIdToDelete = PM_PEER_ID_INVALID;
static nrf_ble_gatt_t s_Gatt;                                     /**< GATT module instance. */
static ble_db_discovery_t s_DbDiscovery;

/**@brief Bluetooth SIG debug mode Private Key */
__ALIGN(4) __WEAK extern const uint8_t g_lesc_private_key[32] = {
    0xbd,0x1a,0x3c,0xcd,0xa6,0xb8,0x99,0x58,0x99,0xb7,0x40,0xeb,0x7b,0x60,0xff,0x4a,
    0x50,0x3f,0x10,0xd2,0xe3,0xb3,0xc9,0x74,0x38,0x5f,0xc5,0xa3,0xd4,0xf6,0x49,0x3f,
};

__ALIGN(4) static ble_gap_lesc_p256_pk_t    s_lesc_public_key;      /**< LESC ECC Public Key */
__ALIGN(4) static ble_gap_lesc_dhkey_t      s_lesc_dh_key;          /**< LESC ECC DH Key*/
static ble_gap_conn_sec_mode_t s_gap_conn_mode;

static nrf_crypto_key_t m_crypto_key_sk =
{
    .p_le_data = (uint8_t *) g_lesc_private_key,
    .len = sizeof(g_lesc_private_key)
};

static nrf_crypto_key_t m_crypto_key_pk =
{
    .p_le_data = (uint8_t *) s_lesc_public_key.pk,
    .len = sizeof(s_lesc_public_key.pk)
};
static nrf_crypto_key_t m_crypto_key_dhkey =
{
    .p_le_data = (uint8_t *) s_lesc_dh_key.key,
    .len = sizeof(s_lesc_dh_key.key)
};

static inline void BleConnLedOff() {
	if (g_BleAppData.ConnLedPort < 0 || g_BleAppData.ConnLedPin < 0)
		return;

	IOPinSet(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
}

static inline void BleConnLedOn() {
	if (g_BleAppData.ConnLedPort < 0 || g_BleAppData.ConnLedPin < 0)
		return;

	IOPinClear(g_BleAppData.ConnLedPort, g_BleAppData.ConnLedPin);
}

void BleAppDfuCallback(fs_evt_t const * const evt, fs_ret_t result)
{
	if (result == FS_SUCCESS)
    {
        NVIC_SystemReset();
    }
}

void BleAppEnterDfu()
{
    uint32_t err_code = nrf_dfu_flash_init(true);

    nrf_dfu_settings_init();

    s_dfu_settings.enter_buttonless_dfu = true;

    err_code = nrf_dfu_settings_write(BleAppDfuCallback);

    if (err_code != NRF_SUCCESS)
    {
    }
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

void BleAppGapDeviceNameSet( const char* ppDeviceName )
{
    uint32_t                err_code;

    err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *)ppDeviceName,
                                          strlen( ppDeviceName ));
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */

static void BleAppGapParamInit(const BLEAPP_CFG *pBleAppCfg)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;

    switch (pBleAppCfg->SecType)
    {
    	case BLEAPP_SECTYPE_NONE:
    	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_STATICKEY_NO_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_STATICKEY_MITM:
		    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_LESC_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_SIGNED_NO_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(&s_gap_conn_mode);
    	    break;
		case BLEAPP_SECTYPE_SIGNED_MITM:
			BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(&s_gap_conn_mode);
    	    break;
    }
/*
    if (pBleAppCfg->pDevName != NULL)
    {
    	err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *) pBleAppCfg->pDevName,
                                          strlen(pBleAppCfg->pDevName));
    	APP_ERROR_CHECK(err_code);
    }
*/
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
    {
		gap_conn_params.min_conn_interval = pBleAppCfg->ConnIntervalMin;// MIN_CONN_INTERVAL;
		gap_conn_params.max_conn_interval = pBleAppCfg->ConnIntervalMax;//MAX_CONN_INTERVAL;
		gap_conn_params.slave_latency     = SLAVE_LATENCY;
		gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

		err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
		APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(g_BleAppData.ConnHdl, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
           // sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t const * p_ble_evt)
{
    uint32_t                         err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	BleConnLedOn();
        	g_BleAppData.ConnHdl = p_ble_evt->evt.gap_evt.conn_handle;

/*            memset(&s_DbDiscovery, 0x00, sizeof(ble_db_discovery_t));
            err_code = ble_db_discovery_start(&s_DbDiscovery, g_BleAppData.ConnHdl);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }
*/
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        	BleConnLedOff();
        	g_BleAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;
        	//err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
           // APP_ERROR_CHECK(err_code);
        	break;

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            //char passkey[8 + 1];
            //memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, 8);
            //passkey[8] = 0;
            // Don't send delayed Security Request if security procedure is already in progress.
           // err_code = app_timer_stop(m_sec_req_timer_id);
           // APP_ERROR_CHECK(err_code);

        	//g_Uart.printf("Passkey: %s\r\n", passkey);
        } break; // BLE_GAP_EVT_PASSKEY_DISPLAY

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            //err_code = sd_ble_gatts_sys_attr_set(g_ConnHdl, NULL, 0, 0);
            //APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
            		BleAppAdvTimeoutHandler();
/*            	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
            	{
                    err_code = ble_advdata_set(&g_BleAppData.AdvData, &g_BleAppData.SRData);
                    err_code = sd_ble_gap_adv_start(&s_AdvParams);
            		APP_ERROR_CHECK(err_code);
            	}*/
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

  //      case BLE_EVT_USER_MEM_REQUEST:
  //          err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
  //          APP_ERROR_CHECK(err_code);
  //          break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                	//printf("BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST\r\n");
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        //    NRF_LOG_INFO("%s: BLE_GAP_EVT_LESC_DHKEY_REQUEST\r\n", nrf_log_push(roles_str[role]));

            static nrf_crypto_key_t peer_pk;
            peer_pk.p_le_data = &p_ble_evt->evt.gap_evt.params.lesc_dhkey_request.p_pk_peer->pk[0];
            peer_pk.len = BLE_GAP_LESC_P256_PK_LEN;
            err_code = nrf_crypto_shared_secret_compute(NRF_CRYPTO_CURVE_SECP256R1, &m_crypto_key_sk, &peer_pk, &m_crypto_key_dhkey);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_lesc_dhkey_reply(g_BleAppData.ConnHdl, &s_lesc_dh_key);
            APP_ERROR_CHECK(err_code);
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
        //     NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x\r\n",
          //                nrf_log_push(roles_str[role]),
            //              p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
              //            p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                //          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                  //        *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                    //      *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        case BLE_EVT_TX_COMPLETE:
            break;


        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
			{
				// Start Security Request timer.
				//err_code = app_timer_start(g_SecReqTimerId, SECURITY_REQUEST_DELAY, NULL);
				//APP_ERROR_CHECK(err_code);
			}
        	break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
			{
				pm_conn_sec_status_t conn_sec_status;

				// Check if the link is authenticated (meaning at least MITM).
				err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
				APP_ERROR_CHECK(err_code);

				if (conn_sec_status.mitm_protected)
				{
				}
				else
				{
					// The peer did not use MITM, disconnect.
					err_code = pm_peer_id_get(g_BleAppData.ConnHdl, &g_PeerMngrIdToDelete);
					APP_ERROR_CHECK(err_code);
					err_code = sd_ble_gap_disconnect(g_BleAppData.ConnHdl,
													 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
				}
			}
			break;

        case PM_EVT_CONN_SEC_FAILED:
            err_code = sd_ble_gap_disconnect(g_BleAppData.ConnHdl,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        	{
        		// Reject pairing request from an already bonded peer.
        		pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
        		pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        	}
        	break;

        case PM_EVT_STORAGE_FULL:
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }

            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        	ble_advertising_start(BLE_ADV_MODE_FAST);
        	break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void BleAppDBDiscoveryHandler(ble_db_discovery_evt_t * p_evt)
{
    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the NUS was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
    {
    	g_BleAppData.pPeriphDev[g_BleAppData.PeriphDevCnt].ConnHdl = p_evt->conn_handle;
    	g_BleAppData.pPeriphDev[g_BleAppData.PeriphDevCnt].SrvcCnt = s_DbDiscovery.srv_count;
    	memcpy(g_BleAppData.pPeriphDev[g_BleAppData.PeriphDevCnt].Srvc, s_DbDiscovery.services, s_DbDiscovery.srv_count * sizeof(ble_gatt_db_srv_t));

        uint32_t i;

        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
        	if (p_chars[i].characteristic.char_props.notify)
        	{
        	    uint32_t                 err_code;
        	    ble_gattc_write_params_t write_params;
        	    uint8_t                  buf[BLE_CCCD_VALUE_LEN];

        	    buf[0] = BLE_GATT_HVX_NOTIFICATION;
        	    buf[1] = 0;

        	    write_params.write_op = BLE_GATT_OP_WRITE_REQ;
        	    write_params.handle   = p_chars[i].cccd_handle;
        	    write_params.offset   = 0;
        	    write_params.len      = sizeof(buf);
        	    write_params.p_value  = buf;

        	    err_code = sd_ble_gattc_write(p_evt->conn_handle, &write_params);
        	    APP_ERROR_CHECK(err_code);
        	}
        	/*
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case BLE_UUID_NUS_RX_CHARACTERISTIC:
                    nus_c_evt.handles.nus_rx_handle = p_chars[i].characteristic.handle_value;
                    break;

                case BLE_UUID_NUS_TX_CHARACTERISTIC:
                    nus_c_evt.handles.nus_tx_handle = p_chars[i].characteristic.handle_value;
                    nus_c_evt.handles.nus_tx_cccd_handle = p_chars[i].cccd_handle;
                    break;

                default:
                    break;
            }
        }
        if (p_ble_nus_c->evt_handler != NULL)
        {
            nus_c_evt.conn_handle = p_evt->conn_handle;
            nus_c_evt.evt_type    = BLE_NUS_C_EVT_DISCOVERY_COMPLETE;
            p_ble_nus_c->evt_handler(p_ble_nus_c, &nus_c_evt);
        }*/
        }
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t role = ble_conn_state_role(p_ble_evt->evt.gap_evt.conn_handle);

    ble_conn_state_on_ble_evt((ble_evt_t*)p_ble_evt);
    if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT) || g_BleAppData.AppRole & BLEAPP_ROLE_CENTRAL)
    {
        BleCentralEvtUserHandler((ble_evt_t *)p_ble_evt);
    }
    else
    {
        pm_on_ble_evt((ble_evt_t*)p_ble_evt);
        ble_advertising_on_ble_evt((ble_evt_t*)p_ble_evt);
        ble_conn_params_on_ble_evt((ble_evt_t*)p_ble_evt);
        BlePeriphEvtUserHandler((ble_evt_t *)p_ble_evt);
    }
    on_ble_evt(p_ble_evt);

    ble_db_discovery_on_ble_evt(&s_DbDiscovery, p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}



/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void BleAppPeerMngrInit(BLEAPP_SECTYPE SecType, uint8_t SecKeyExchg, bool bEraseBond)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (bEraseBond)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    sec_param.bond = 1;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    switch (SecType)
    {
    	case BLEAPP_SECTYPE_NONE:
			break;
		case BLEAPP_SECTYPE_STATICKEY_NO_MITM:
			break;
		case BLEAPP_SECTYPE_STATICKEY_MITM:
			break;
		case BLEAPP_SECTYPE_LESC_MITM:
	    	sec_param.mitm = 1;
		    sec_param.lesc = 1;
			break;
		case BLEAPP_SECTYPE_SIGNED_NO_MITM:
			break;
		case BLEAPP_SECTYPE_SIGNED_MITM:
			break;
    }

    if (SecType == BLEAPP_SECTYPE_STATICKEY_MITM ||
    	SecType == BLEAPP_SECTYPE_LESC_MITM ||
		SecType == BLEAPP_SECTYPE_SIGNED_MITM)
    {
    	sec_param.mitm = 1;
    }

    int type = SecKeyExchg & (BLEAPP_SECEXCHG_KEYBOARD | BLEAPP_SECEXCHG_DISPLAY);
    switch (type)
    {
    	case BLEAPP_SECEXCHG_KEYBOARD:
    		sec_param.keypress = 1;
    		sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_ONLY;
    		break;
    	case BLEAPP_SECEXCHG_DISPLAY:
    		sec_param.io_caps  = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
    		break;
    	case (BLEAPP_SECEXCHG_KEYBOARD | BLEAPP_SECEXCHG_DISPLAY):
			sec_param.keypress = 1;
			sec_param.io_caps  = BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
			break;
    }

    if (SecKeyExchg & BLEAPP_SECEXCHG_OOB)
    {
    	sec_param.oob = 1;
    }

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    /* Set the public key */
    err_code = pm_lesc_public_key_set(&s_lesc_public_key);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    uint32_t err_code;

    if (g_BleAppData.ConnHdl != BLE_CONN_HANDLE_INVALID)
    {
        // Initiate bonding.
        //NRF_LOG_DEBUG("Start encryption\r\n");
        err_code = pm_conn_secure(g_BleAppData.ConnHdl, false);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

bool BleAppAdvManDataSet(uint8_t *pAdvData, int AdvLen, uint8_t *pSrData, int SrLen)
{
   uint32_t ret = ble_advdata_set(&g_BleAppData.AdvData, &g_BleAppData.SRData);
   APP_ERROR_CHECK(ret);

   return false;
}

void BleAppAdvStart(BLEAPP_ADVMODE AdvMode)
{
	uint32_t err_code;

	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
	{
		 err_code = sd_ble_gap_adv_start(&s_AdvParams);
	}
	else
	{
		err_code = ble_advertising_start((ble_adv_mode_t)AdvMode);
	}
    APP_ERROR_CHECK(err_code);
}

/**@brief Overloadable function for initializing the Advertising functionality.
 */
__WEAK void BleAppAdvInit(const BLEAPP_CFG *pCfg)
{
    uint32_t               err_code;

    memset(&g_BleAppData.AdvData, 0, sizeof(g_BleAppData.AdvData));
    memset(&g_BleAppData.SRData, 0, sizeof(g_BleAppData.SRData));

    s_BleAppManData.company_identifier = pCfg->VendorID;
    s_BleAppManData.data.p_data = (uint8_t*)pCfg->pAdvManData;
    s_BleAppManData.data.size = pCfg->AdvManDataLen;

    g_BleAppData.AdvData.include_appearance = false;
    g_BleAppData.AdvData.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    if (pCfg->pDevName != NULL)
    {
    	if (strlen(pCfg->pDevName) < 14)
    	{
    		g_BleAppData.AdvData.name_type      = BLE_ADVDATA_SHORT_NAME;
    		g_BleAppData.AdvData.short_name_len = strlen(pCfg->pDevName);
    	}
    	else
    		g_BleAppData.AdvData.name_type = BLE_ADVDATA_FULL_NAME;
    }
    else
    {
    	g_BleAppData.AdvData.name_type = BLE_ADVDATA_NO_NAME;
    }

    if (g_BleAppData.AdvData.name_type == BLE_ADVDATA_NO_NAME)
    {
        if (pCfg->NbAdvUuid > 0 && pCfg->pAdvUuids != NULL)
        {
        	g_BleAppData.AdvData.uuids_complete.uuid_cnt = pCfg->NbAdvUuid;
        	g_BleAppData.AdvData.uuids_complete.p_uuids  = (ble_uuid_t*)pCfg->pAdvUuids;
			if (pCfg->pAdvManData != NULL)
			{
				g_BleAppData.SRData.p_manuf_specific_data = &s_BleAppManData;
			}
        }
        else
        {
			if (pCfg->pAdvManData != NULL)
			{
				g_BleAppData.AdvData.p_manuf_specific_data = &s_BleAppManData;
			}
        }
    }
    else
    {
        if (pCfg->NbAdvUuid > 0 && pCfg->pAdvUuids != NULL)
        {
        	g_BleAppData.SRData.uuids_complete.uuid_cnt = pCfg->NbAdvUuid;
        	g_BleAppData.SRData.uuids_complete.p_uuids  = (ble_uuid_t*)pCfg->pAdvUuids;
        }
        if (pCfg->pAdvManData != NULL)
        {
        	g_BleAppData.AdvData.p_manuf_specific_data = &s_BleAppManData;
        }
    }

    if (pCfg->AppMode == BLEAPP_MODE_NOCONNECT || pCfg->AppMode == BLEAPP_MODE_IBEACON)
    {
        err_code = ble_advdata_set(&g_BleAppData.AdvData, &g_BleAppData.SRData);
        APP_ERROR_CHECK(err_code);

        // Initialize advertising parameters (used when starting advertising).
        memset(&s_AdvParams, 0, sizeof(s_AdvParams));

        s_AdvParams.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
        s_AdvParams.p_peer_addr = NULL;                             // Undirected advertisement.
        s_AdvParams.fp          = BLE_GAP_ADV_FP_ANY;
        s_AdvParams.interval    = pCfg->AdvInterval;
        s_AdvParams.timeout     = pCfg->AdvTimeout;
    }
    else
    {
		memset(&g_BleAppData.AdvConf, 0, sizeof(g_BleAppData.AdvConf));
    	g_BleAppData.AdvConf.ble_adv_fast_enabled  = true;
    	g_BleAppData.AdvConf.ble_adv_fast_interval = pCfg->AdvInterval;
    	g_BleAppData.AdvConf.ble_adv_fast_timeout  = pCfg->AdvTimeout;

		if (pCfg->AdvSlowInterval > 0)
		{
			g_BleAppData.AdvConf.ble_adv_slow_enabled  = true;
			g_BleAppData.AdvConf.ble_adv_slow_interval = pCfg->AdvSlowInterval;
			g_BleAppData.AdvConf.ble_adv_slow_timeout  = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
		}

    }

	err_code = ble_advertising_init(&g_BleAppData.AdvData, &g_BleAppData.SRData, &g_BleAppData.AdvConf, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);

	//memcpy(&g_BleAppData.AdvData, padvdata, sizeof(ble_advdata_t));
    //memcpy(&g_BleAppData.SRData, psrdata, sizeof(ble_advdata_t));
}

void BleAppDisInit(const BLEAPP_CFG *pBleAppCfg)
{
    ble_dis_init_t   dis_init;
    ble_dis_pnp_id_t pnp_id;

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    if (pBleAppCfg->pDevDesc)
    {
    	ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char*)pBleAppCfg->pDevDesc->ManufName);
    	ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char*)pBleAppCfg->pDevDesc->ModelName);
    	if (pBleAppCfg->pDevDesc->pSerialNoStr)
    		ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char*)pBleAppCfg->pDevDesc->pSerialNoStr);
    	if (pBleAppCfg->pDevDesc->pFwVerStr)
    		ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char*)pBleAppCfg->pDevDesc->pFwVerStr);
    	if (pBleAppCfg->pDevDesc->pHwVerStr)
    		ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char*)pBleAppCfg->pDevDesc->pHwVerStr);
    }

    pnp_id.vendor_id  = pBleAppCfg->VendorID;
    pnp_id.product_id = pBleAppCfg->ProductId;
    dis_init.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    uint32_t err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

}

uint16_t BleAppGetConnHandle()
{
	return g_BleAppData.ConnHdl;
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t * p_evt)
{
 /*   if ((g_BleAppData.ConnHdl == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
       // m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }*/
   // NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&s_Gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&s_Gatt, 64);
    APP_ERROR_CHECK(err_code);
}

bool BleAppConnectable(const BLEAPP_CFG *pBleAppCfg, bool bEraseBond)
{
	uint32_t err_code;


    err_code = nrf_crypto_public_key_compute(NRF_CRYPTO_CURVE_SECP256R1, &m_crypto_key_sk, &m_crypto_key_pk);
    APP_ERROR_CHECK(err_code);

    BleAppPeerMngrInit(pBleAppCfg->SecType, pBleAppCfg->SecExchg, bEraseBond);

    //BleAppInitUserData();

    BleAppGapParamInit(pBleAppCfg);

    BleAppInitUserServices();

    if (pBleAppCfg->bEnDevInfoService)
    	BleAppDisInit(pBleAppCfg);


    if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
    	conn_params_init();

    return true;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
/*static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);

    if (    (p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
        &&  (is_already_connected(&p_ble_evt->evt.gap_evt.params.connected.peer_addr)))
    {
        NRF_LOG_INFO("%s: Already connected to this device as %s (handle: %d), disconnecting.",
                     (role == BLE_GAP_ROLE_PERIPH) ? "PERIPHERAL" : "CENTRAL",
                     (role == BLE_GAP_ROLE_PERIPH) ? "CENTRAL"    : "PERIPHERAL",
                     conn_handle);

        (void)sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        // Do not process the event further.
        return;
    }

    on_ble_evt(p_ble_evt);

    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // Manages peripheral LEDs.
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        on_ble_central_evt(p_ble_evt);
    }
}
*/
/*
void sys_event_handler(uint32_t sys_evt, void * p_context)
{
    if (m_state != STATE_IDLE && m_state != STATE_ERROR)
    {
        switch (sys_evt)
        {
            case NRF_EVT_FLASH_OPERATION_SUCCESS:
                flash_operation_success_run();
                break;

            case NRF_EVT_FLASH_OPERATION_ERROR:
                if (!(m_flags & MASK_FLASH_API_ERR_BUSY))
                {
                    flash_operation_failure_run();
                }
                else
                {
                    // As our last flash operation request was rejected by the flash API reissue the
                    // request by doing same code execution path as for flash operation sucess
                    // event. This will promote code reuse in the implementation.
                    flash_operation_success_run();
                }
                break;

            default:
                // No implementation needed.
                break;
        }

    }
}
*/

/**
 * @brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
bool BleAppInit(const BLEAPP_CFG *pBleAppCfg, bool bEraseBond)
{
	ret_code_t err_code;

	g_BleAppData.ConnLedPort = pBleAppCfg->ConnLedPort;
	g_BleAppData.ConnLedPin = pBleAppCfg->ConnLedPin;

	if (pBleAppCfg->ConnLedPort != -1 && pBleAppCfg->ConnLedPin != -1)
    {
		IOPinConfig(pBleAppCfg->ConnLedPort, pBleAppCfg->ConnLedPin, 0,
    				IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    }

	// initializing the cryptography module
    nrf_crypto_init();

    g_BleAppData.AppMode = pBleAppCfg->AppMode;
    g_BleAppData.ConnHdl = BLE_CONN_HANDLE_INVALID;

    switch (pBleAppCfg->AppMode)
    {
		case BLEAPP_MODE_LOOP:
		case BLEAPP_MODE_NOCONNECT:
		case BLEAPP_MODE_IBEACON:
			APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
			//APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
			SOFTDEVICE_HANDLER_INIT((nrf_clock_lf_cfg_t*)&pBleAppCfg->ClkCfg, NULL);
			break;
		case BLEAPP_MODE_APPSCHED:
			APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
			APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
			SOFTDEVICE_HANDLER_APPSH_INIT((nrf_clock_lf_cfg_t*)&pBleAppCfg->ClkCfg, true);
			break;
		case BLEAPP_MODE_RTOS:
			if (pBleAppCfg->SDEvtHandler == NULL)
				return false;
			SOFTDEVICE_HANDLER_INIT((nrf_clock_lf_cfg_t*)&pBleAppCfg->ClkCfg, pBleAppCfg->SDEvtHandler);
			break;
    }


    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(pBleAppCfg->CentLinkCount,
                                                    pBleAppCfg->PeriLinkCount,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(pBleAppCfg->CentLinkCount, pBleAppCfg->PeriLinkCount);//CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    if (pBleAppCfg->PeriLinkCount > 0 && pBleAppCfg->AdvInterval > 0)
    {
    		g_BleAppData.AppRole |= BLEAPP_ROLE_PERIPHERAL;
    }
    if (pBleAppCfg->CentLinkCount > 0)
    {
		g_BleAppData.AppRole |= BLEAPP_ROLE_CENTRAL;
		ret_code_t err_code = ble_db_discovery_init(BleAppDBDiscoveryHandler);
		APP_ERROR_CHECK(err_code);
    }

    if (pBleAppCfg->AppMode != BLEAPP_MODE_NOCONNECT)
    {
    		BleAppConnectable(pBleAppCfg, bEraseBond);
    }

    BleAppInitUserData();

    if (pBleAppCfg->pDevName != NULL)
    {
        err_code = sd_ble_gap_device_name_set(&s_gap_conn_mode,
                                          (const uint8_t *) pBleAppCfg->pDevName,
                                          strlen(pBleAppCfg->pDevName));
        APP_ERROR_CHECK(err_code);
    }

    BleAppAdvInit(pBleAppCfg);

    return true;
}

void BleAppRun()
{

	if (g_BleAppData.AppMode == BLEAPP_MODE_NOCONNECT)
	{
		uint32_t err_code = sd_ble_gap_adv_start(&s_AdvParams);
		APP_ERROR_CHECK(err_code);
	}
	else
	{
		if (g_BleAppData.AppRole & BLEAPP_ROLE_PERIPHERAL)
		{
			uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
		}
	}

    while (1)
    {
		if (g_BleAppData.AppMode == BLEAPP_MODE_RTOS)
		{
			BleAppRtosWaitEvt();
			intern_softdevice_events_execute();
		}
		else
		{
			if (g_BleAppData.AppMode == BLEAPP_MODE_APPSCHED)
			{
				app_sched_execute();
			}
			sd_app_evt_wait();
		}
    }
}




