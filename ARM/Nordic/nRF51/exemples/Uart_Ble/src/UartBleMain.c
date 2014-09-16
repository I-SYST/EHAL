/*--------------------------------------------------------------------------
File   : UartBleMain.c

Author : Hoang Nguyen Hoan          Sep. 13, 2014

Desc   : UART over custom BLE exemple

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
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_hci.h"
#include "ble_conn_params.h"
#include "device_manager.h"
#include "softdevice_handler.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf_gpio.h"
#include "pstorage.h"
#include "nrf_gpio.h"

#include "UartBleService.h"

#define PRODUCT_NAME              "IMM-NRF51822-UART"    /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME         "I-SYST inc."		/**< Manufacturer. Will be passed to Device Information Service. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT      	1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define BASE_USB_HID_SPEC_VERSION        		0x0101                                         /**< Version number of base USB HID Specification implemented by this application. */

#define ASSERT_LED_PIN_NO					18

#define APP_GPIOTE_MAX_USERS			1
#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE,\
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE)       /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_ADV_INTERVAL_FAST           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_INTERVAL_SLOW           0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_FAST_ADV_TIMEOUT            30                                          /**< The duration of the fast advertising period (in seconds). */
#define APP_SLOW_ADV_TIMEOUT            180                                         /**< The duration of the slow advertising period (in seconds). */
#define APP_DIRECTED_ADV_TIMEOUT        5                                           /**< Number of direct advertisement (each lasting 1.28seconds). */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */

#define APP_ADV_INTERVAL                    40                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS          180                                       /**< The advertising timeout in units of seconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
#define SLAVE_LATENCY                   25                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (300 ms). */

#define SEC_PARAM_TIMEOUT                30                                             /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                   1                                              /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                              /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                           /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                              /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                              /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                             /**< Maximum encryption key size. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)    /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                              /**< Number of attempts before giving up the connection parameter negotiation. */

#define FLASH_PAGE_SYS_ATTR                  (PSTORAGE_FLASH_PAGE_END - 3)                  /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                      (PSTORAGE_FLASH_PAGE_END - 1)                  /**< Flash page used for bond manager bonding information. */

#define UART_TX_PIN			7
#define UART_RX_PIN			8
#define UART_RTS_PIN		9
#define UART_CTS_PIN		10

app_timer_id_t						g_BatTimerId;
app_gpiote_user_id_t				g_GpioteId = 0;
static ble_uarts_t					g_UartServ;
ble_bas_t                        	g_BatServ;                                         /**< Structure used to identify the battery service. */
static ble_gap_sec_params_t         g_GAPSecParams;                                  /**< Security requirements for this application. */
static dm_application_instance_t    g_AppHandle; /**< Application identifier allocated by device manager */
uint16_t 							g_AppUartId = 0;


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	#define DEAD_BEEF	0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,  (const uint8_t *)PRODUCT_NAME, strlen(PRODUCT_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ble_advdata_manuf_data_t data;
    uint64_t		id;

    ble_uuid_t blueio_uuids[] =
    {
        {UART_UUID_SERVICE,	g_UartServ.uuid_type},
    };
    ble_uuid_t std_uuids[] =
    {
//		{BLE_UUID_BATTERY_SERVICE,            		BLE_UUID_TYPE_BLE},
		{BLE_UUID_DEVICE_INFORMATION_SERVICE, 		BLE_UUID_TYPE_BLE}

    };

    id = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | (uint64_t)NRF_FICR->DEVICEID[0];

    data.company_identifier = 0x2082;
    data.data.p_data = (uint8_t*)&id;
    data.data.size = sizeof(id);

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(std_uuids) / sizeof(std_uuids[0]);
    advdata.uuids_complete.p_uuids  = std_uuids;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(blueio_uuids) / sizeof(blueio_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = blueio_uuids;
    scanrsp.p_manuf_specific_data = &data;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    uint32_t         err_code;
    ble_dis_init_t   discfg;

    memset(&discfg, 0, sizeof(discfg));

    ble_srv_ascii_to_utf8(&discfg.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&discfg.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&discfg.dis_attr_md.write_perm);

    err_code = ble_dis_init(&discfg);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bascfg;

    memset(&bascfg, 0, sizeof(bascfg));

    bascfg.evt_handler          = NULL;
    bascfg.support_notification = true;
    bascfg.p_report_ref         = NULL;
    bascfg.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bascfg.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bascfg.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bascfg.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bascfg.battery_level_report_read_perm);

    err_code = ble_bas_init(&g_BatServ, &bascfg);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the security parameters.
 */
static void sec_params_init(void)
{
	g_GAPSecParams.timeout      = SEC_PARAM_TIMEOUT;
	g_GAPSecParams.bond         = SEC_PARAM_BOND;
	g_GAPSecParams.mitm         = SEC_PARAM_MITM;
	g_GAPSecParams.io_caps      = SEC_PARAM_IO_CAPABILITIES;
	g_GAPSecParams.oob          = SEC_PARAM_OOB;
	g_GAPSecParams.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	g_GAPSecParams.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(g_UartServ.conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t conparam;

    memset(&conparam, 0, sizeof(conparam));

    conparam.p_conn_params                  = NULL;
    conparam.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    conparam.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    conparam.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    conparam.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    conparam.disconnect_on_fail             = true;
    conparam.evt_handler                    = on_conn_params_evt;
    conparam.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&conparam);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           api_result_t           event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t                err_code;
    dm_init_param_t         init_data;
    dm_application_param_t  register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    init_data.clear_persistent_data = true;//(nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_ID) == 0);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_NONE;

    err_code = dm_register(&g_AppHandle, &register_param);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_clear(24);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t        err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(30);

            g_UartServ.conn_handle  = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(30);
            g_UartServ.conn_handle = BLE_CONN_HANDLE_INVALID;

            advertising_start();

            // Go to system-off mode, should not return from this function, wakeup will trigger
            // a reset.
            //system_off_mode_enter();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(g_UartServ.conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &g_GAPSecParams);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
            	nrf_gpio_pin_set(24);

                    // Go to system-off mode.
                    // (this function will not return; wakeup will cause a reset).
                    err_code = sd_power_system_off();
                    APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(g_UartServ.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_uarts_on_ble_evt(&g_UartServ, p_ble_evt);
    ble_bas_on_ble_evt(&g_BatServ, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, true);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static void uarts_write_handler(ble_uarts_t * p_uarts, uint8_t *data, int len)
{
	uint32_t err;

	for (int i = 0; i < len; i++)
	{
		err = app_uart_put(data[i]);
		APP_ERROR_CHECK(err);
	}
	app_uart_put('\n');
	//app_uart_flush();
}

/**@brief Function for initializing the services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
	ble_uarts_init_t uartscfg;

	uartscfg.write_handler = uarts_write_handler;
	ble_uarts_init(&g_UartServ, &uartscfg);
}

void BLEStart(void)
{
	// Initialize Bluetooth Stack parameters
	g_UartServ.conn_handle = BLE_CONN_HANDLE_INVALID;

	ble_stack_init();
	services_init();
	device_manager_init();
	gap_params_init();
    //bas_init();
    dis_init();
	advertising_init();
	conn_params_init();
	sec_params_init();

	// Start advertising
	advertising_start();
}

static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
  //  err_code = app_timer_create(&m_battery_timer_id,
  //                              APP_TIMER_MODE_REPEATED,
  //                              battery_level_meas_timeout_handler);
   // APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&m_heart_rate_timer_id,
    //                            APP_TIMER_MODE_REPEATED,
    //                            heart_rate_meas_timeout_handler);
    //APP_ERROR_CHECK(err_code);
}

void blink()
{
	while (1)
	{
		nrf_gpio_pin_set(30);
		for (int i=0; i < 10000; i++)
		__NOP();
		nrf_gpio_pin_clear(30);
		for (int i=0; i < 10000; i++)
		__NOP();
	}

}

void uart_evt_handler(app_uart_evt_t *p_evt)
{
	uint8_t buff[24];
	uint32_t err = 0;
	uint16_t len;
    ble_gatts_hvx_params_t params;

	switch (p_evt->evt_type)
	{
		case APP_UART_TX_EMPTY:
			break;
		case APP_UART_DATA_READY:
			for (len = 0; len < 24 && err == 0; len++)
				err = app_uart_get(&buff[len]);

		    memset(&params, 0, sizeof(params));
		    params.type = BLE_GATT_HVX_NOTIFICATION;
		    params.handle = g_UartServ.data_char_handles.value_handle;
		    params.p_data = buff;
		    params.p_len = &len;

		    return sd_ble_gatts_hvx(g_UartServ.conn_handle, &params);

			//ble_uarts_on_data_change(buff, len);
			break;
		case APP_UART_DATA:
			break;
	}
}

void uart_init()
{
	uint32_t err_code;
	app_uart_comm_params_t cfg;

	cfg.rx_pin_no = UART_RX_PIN;
	cfg.tx_pin_no = UART_TX_PIN;
	cfg.rts_pin_no = UART_RTS_PIN;
	cfg.cts_pin_no = UART_CTS_PIN;
	cfg.flow_control = APP_UART_FLOW_CONTROL_DISABLED;
	cfg.use_parity = false;
	cfg.baud_rate = UART_BAUDRATE_BAUDRATE_Baud230400;

	APP_UART_FIFO_INIT(&cfg, 32, 32, uart_evt_handler, APP_IRQ_PRIORITY_LOW, err_code);
}

int main()
{
	//APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
	nrf_gpio_cfg_output(24);
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
	timers_init();
	uart_init();
//	blink();
	nrf_gpio_pin_set(24);

	BLEStart();
//	timers_start();
//	blink();


	while (1)
	{
		app_sched_execute();
		uint32_t err_code = sd_app_evt_wait();
		APP_ERROR_CHECK(err_code);
		//BlueIOADCStart();
	}
}
