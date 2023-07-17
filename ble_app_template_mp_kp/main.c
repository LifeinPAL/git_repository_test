/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "ble_nus.h"
#include "app_uart.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "event_id.h"
#include "main.h"
#include "user_gpio.h"


#define DEVICE_NAME                     "MultiPeripheral_Template"              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                1600                                    /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                0                                		/**< The advertising duration (unlimited) in units of 10 milliseconds. */

/**@brief	Priority of the application BLE event handler.
 * @note	You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


#define ADVERTISING_LED					BSP_BOARD_LED_0							/*Is on when device is advertising*/
#define CONNECTED_FIRST_LED				BSP_BOARD_LED_1							/*Is on when the first device has connected*/
#define CONNECTED_SECOND_LED			BSP_BOARD_LED_2							/*Is on when the second device has connected*/

#define RESERVING

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
/*多从机模式，即从机被多个主机连接时，初始化多个（即数组）队列写模块*/
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT);                     /**< Context for the Queued Write module.*/
#ifndef RESERVING
/*多从机例程中没有注册广播实例，可能是因为注册的回调函数中处理disconnect事件时用到的重连机制只能用于1vs1连接*/
//BLE_ADVERTISING_DEF(m_advertising);/**< Advertising module instance. */
#endif //RESERVING
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;					/**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];					/**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];			/**< Buffer for storing an encoded scan data. */
/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data = {
	.adv_data = {
		.p_data = m_enc_advdata,
		.len 	= BLE_GAP_ADV_SET_DATA_SIZE_MAX
	},
	.scan_rsp_data = {
		.p_data = m_enc_scan_response_data,
		.len 	= BLE_GAP_ADV_SET_DATA_SIZE_MAX
	},
};


//static uint16_t m_conn_handle[NRF_SDH_BLE_PERIPHERAL_LINK_COUNT] = {BLE_CONN_HANDLE_INVALID};      /**< Handle of the current connection. */
/*定义一个全局用的连接句柄状态器*/
static ble_conn_state_conn_handle_list_t m_conn_handles = {
	.len = 0,
	{BLE_CONN_STATUS_INVALID}
};


/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_PERIPHERAL_LINK_COUNT);                               /**< BLE NUS service instance. */ 
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;        /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


static ble_gap_sec_params_t m_sec_param;	//存储关于绑定配对安全连接的参数

static void advertising_start(void);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



#include "nrf_drv_gpiote.h"
APP_TIMER_DEF(m_app_timer_test);	/**< 注册一个测试用的app_timer */
static bool flag_test = false;

void app_timer_handler_test(void* p_context)
{
	UNUSED_PARAMETER(p_context);
	flag_test = true;
	nrf_drv_gpiote_out_toggle(USER_PIN_LED_2);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
	
	err_code = app_timer_create(&m_app_timer_test, APP_TIMER_MODE_SINGLE_SHOT, app_timer_handler_test);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
	NRF_LOG_INFO("nus event id = %d: %s", p_evt->type, nus_event_type[p_evt->type]);
	switch (p_evt->type)
	{
		case BLE_NUS_EVT_RX_DATA:
		{
			uint32_t err_code;
			char* p_received = "RECIEVED";
			uint16_t sz_received = sizeof("RECIEVED");
			err_code = ble_nus_data_send(&m_nus, (uint8_t*)p_received, &sz_received, p_evt->conn_handle);
			APP_ERROR_CHECK(err_code);
			NRF_LOG_INFO("send recieved");
//			err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//			APP_ERROR_CHECK(err_code);

//			NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
//			NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

//			for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
//			{
//				do
//				{
//					err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
//					if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
//					{
//						NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
//						APP_ERROR_CHECK(err_code);
//					}
//				} while (err_code == NRF_ERROR_BUSY);
//			}
//			if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
//			{
//				while (app_uart_put('\n') == NRF_ERROR_BUSY);
//			}
		}break;
		case BLE_NUS_EVT_COMM_STARTED:
		{
			char* p_pond = "POND";
			uint16_t sz_pond = sizeof("POND");
			ble_nus_data_send(&m_nus, (uint8_t*)p_pond, &sz_pond, p_evt->conn_handle);
			NRF_LOG_INFO("send pond");
		}break;
		default:
			break;
	}	

}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
	ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module instances.
    qwr_init.error_handler = nrf_qwr_error_handler;
	
	/*多从机下初始化队列写模块数组*/
	for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
	 
	// Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t 	data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    uint32_t       	err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

#ifndef RESERVING					
                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
#endif //RESERVING
					for(size_t i = 0; i < m_conn_handles.len; i++)
					{
						if (BLE_CONN_STATUS_CONNECTED == ble_conn_state_status(m_conn_handles.conn_handles[i]))
						{
							do
							{
								err_code = ble_nus_data_send(&m_nus, data_array, &index, m_conn_handles.conn_handles[i]);
								if ((err_code != NRF_ERROR_INVALID_STATE) &&
									(err_code != NRF_ERROR_RESOURCES) &&
									(err_code != NRF_ERROR_NOT_FOUND))
								{
									APP_ERROR_CHECK(err_code);
								}
							}while (err_code == NRF_ERROR_RESOURCES);
						}
					}
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = USER_PIN_UART_RX,
        .tx_pin_no    = USER_PIN_UART_TX,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
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
    ret_code_t             err_code;
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

#ifndef RESERVING
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;
	NRF_LOG_INFO("advertising event id = %d: %s", ble_adv_evt, ble_adv_evt_type[ble_adv_evt]);
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}
#endif	//RESERVING


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
	
	/*获取当前连接分配的连接句柄*/
    uint16_t conn_handle	= p_ble_evt->evt.gap_evt.conn_handle;
	/*根据连接句柄查询当前连接本地设备的角色*/
	uint16_t role			= ble_conn_state_role(conn_handle);
	/*获取当前作为从机的连接数量*/
	uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count();
	
	NRF_LOG_INFO("ble event id = %d: %s", p_ble_evt->header.evt_id, ble_evt_id[p_ble_evt->header.evt_id]);
	NRF_LOG_INFO("ble gap event connect handle = %d", conn_handle);
	NRF_LOG_INFO("local device role id = %d: %s", role, gap_roles[role]);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
		{
			NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x %s",
                         conn_handle,
                         p_ble_evt->evt.gap_evt.params.disconnected.reason,
						 hci_status_codes[p_ble_evt->evt.gap_evt.params.disconnected.reason]);
			if (0 == periph_link_cnt)
			{

				
#ifndef RESERVING
				bsp_board_led_off(CONNECTED_FIRST_LED);
				bsp_board_led_off(CONNECTED_SECOND_LED);
#endif //RESERVING
			}
			if (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1 == periph_link_cnt)
			{
				//Advertising is not running when all connections are taken, and must therefore be started.
				advertising_start();
			
#ifndef RESERVING			
				bsp_board_led_off(CONNECTED_SECOND_LED);
#endif //RESERVING				
			}

			/*更新连接句柄状态*/
			m_conn_handles = ble_conn_state_periph_handles();
		}break;

        case BLE_GAP_EVT_CONNECTED:
		{
			// Assign connection handle to available instance of QWR module.
			for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
			{
				if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
				{
					err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
					APP_ERROR_CHECK(err_code);
					break;
				}
			}
			
			//Update LEDs.
			if (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT == periph_link_cnt)
            {
				USER_GPIO_LOW(USER_PIN_LED_1);
			
#ifndef RESERVING	
            	bsp_board_led_on(CONNECTED_SECOND_LED);
				bsp_board_led_off(ADVERTISING_LED);
#endif //RESERVING			
            }
			else
			{
				
#ifndef RESERVING					
				bsp_board_led_on(CONNECTED_FIRST_LED);
#endif //RESERVING					
				advertising_start();
			}
			/*更新连接句柄状态*/
			m_conn_handles = ble_conn_state_periph_handles();
		}break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
		
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		{
			NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST.");
//            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, &m_sec_param, NULL);
//            APP_ERROR_CHECK(err_code);
		}break;
		
		case BLE_GAP_EVT_AUTH_STATUS:
		{
			if (p_ble_evt->evt.gap_evt.params.auth_status.auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
            {
            	NRF_LOG_INFO("AUTH SUCCESS with status : %d.", p_ble_evt->evt.gap_evt.params.auth_status.auth_status);
            }
            else
            {
            	NRF_LOG_INFO("AUTH FAILURE with status : %d.", p_ble_evt->evt.gap_evt.params.auth_status.auth_status);
            }
		}break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
		
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
	NRF_LOG_INFO("PM event id = %d: %s", p_evt->evt_id, pm_evt_id[p_evt->evt_id]);
	/*打印peer_manager日志
	 *如果已连接已经绑定的peer，则启动加密
	 *有错误调用错误处理
	 */
    pm_handler_on_pm_evt(p_evt);
	
	/*用于断连的辅助性标准函数，当连接不再是安全的
	 *例如收到PM_EVT_CONN_SEC_FAILED
	 *通常情况该函数在每次Peer Manager event都调用
	 *是pm_handler_on_pm_evt()的补充而非代替
	 */
    pm_handler_disconnect_on_sec_failure(p_evt);
	
	/*
	 *根据上次连接的时间对peer排名
	 *必要时垃圾桶收集FLASH内存
	 *垃圾收集不够时清除最低排名的peer
	 */
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
//            advertising_start();
            break;
		
		case PM_EVT_CONN_SEC_START:
			NRF_LOG_INFO("connect secure start.");
			break;
		
		case PM_EVT_CONN_SEC_PARAMS_REQ:
			NRF_LOG_INFO("connect secure parameters request.");
			break;

        default:
            break;
    }
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
//    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&m_sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    m_sec_param.bond           = SEC_PARAM_BOND;				//是否支持绑定bonding
    m_sec_param.mitm           = SEC_PARAM_MITM;				//是否支持MITM保护
    m_sec_param.lesc           = SEC_PARAM_LESC;				//是否支持LESC
    m_sec_param.keypress       = SEC_PARAM_KEYPRESS;			//是否使能keypress指示
    m_sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;	//IO输入输出能力
    m_sec_param.oob            = SEC_PARAM_OOB;				//是否支持OOB
    m_sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    m_sec_param.kdist_own.enc  = 1;							//分发自己的LTK
    m_sec_param.kdist_own.id   = 1;							//分发自己的IRK
    m_sec_param.kdist_peer.enc = 1;							//要求对方分发LTK
    m_sec_param.kdist_peer.id  = 1;							//要求对方分发IRK

    err_code = pm_sec_params_set(&m_sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);					//注册peer_manager模块的回调函数
    APP_ERROR_CHECK(err_code);
}




#ifndef RESERVING
/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}
#endif //RESERVING

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             	err_code;
	ble_advdata_t			advdata;
	ble_advdata_t			srdata;
	ble_gap_adv_params_t	adv_params;
	
	//Build and set advertising data and scan response data.
	memset(&advdata, 0, sizeof(advdata));
	
	advdata.name_type 				= BLE_ADVDATA_FULL_NAME;	
	advdata.include_appearance		= true;
	advdata.flags					= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	advdata.uuids_complete.p_uuids  = m_adv_uuids;
	
	err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
	APP_ERROR_CHECK(err_code);
	
	memset(&srdata, 0, sizeof(srdata));
	
	srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	srdata.uuids_complete.p_uuids  = m_adv_uuids;	
	
	err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
	APP_ERROR_CHECK(err_code);
	
	//Advertising parameters set.
	memset(&adv_params, 0, sizeof(adv_params));
	
	adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
	adv_params.p_peer_addr = NULL;
	adv_params.interval = APP_ADV_INTERVAL;
	adv_params.duration = APP_ADV_DURATION;
	adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
	adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
	
	err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
	APP_ERROR_CHECK(err_code);

//    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


#ifndef RESERVING
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
#endif //RESERVING

/**@brief Function for initializing leds.
 */
static void leds_init(void)
{

	uint32_t m_leds_list[2] = {USER_PIN_LED_1, USER_PIN_LED_2};
	for (size_t i = 0; i < 2; i++)
	{
		user_gpio_output_init(m_leds_list[i]);
		USER_GPIO_LOW(m_leds_list[i]);
	}
	
	user_gpio_output_init(USER_PIN_CE_ADC);
	USER_GPIO_LOW(USER_PIN_CE_ADC);

#ifndef RESERVING
	bsp_board_init(BSP_INIT_LEDS);
#endif //RESERVING

}

static uint32_t get_rtc_counter(void)
{
	return NRF_RTC1->COUNTER;
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
	ret_code_t	err_code;
	
	err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("Advertising start.");
	USER_GPIO_HIGH(USER_PIN_LED_1);

#ifndef RESERVING
    bsp_board_led_on(ADVERTISING_LED);
#endif //RESERVING
	
//    if (erase_bonds == true)
//    {
//        delete_bonds();
//        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
//    }
//    else
//    {
//        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

//        APP_ERROR_CHECK(err_code);
//    }
}

/**< 以下为硬件驱动测试部分。 */


//#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
static const nrf_drv_timer_t TIMER_DOOR = NRF_DRV_TIMER_INSTANCE(1);
/*< GPIO事件通知驱动*/


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//    nrf_drv_gpiote_out_toggle(USER_PIN_LED_4);
	nrf_drv_timer_enable(&TIMER_DOOR);
}

static void gpio_init(void)
{
	ret_code_t err_code;
	
	err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(USER_PIN_MAGNET, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(USER_PIN_MAGNET, true);
	
	
}


/**< saadc驱动测试 */
#include "nrf_drv_saadc.h"

#define SAADC_DEFAULT_CHANNEL
#ifdef SAADC_DEFAULT_CHANNEL
#define REFERANCE_VOLTAGE_IN_MILIVOLTS 600
#define SAADC_GAIN_RECIPROCAL 6
#define SAADC_RESOLUTION 10
#endif //SAADC_DEFAULT_CHANNEL


#define SAMPLES_IN_BUFFER 1
//static nrf_saadc_value_t	m_saadc_buffer[SAMPLES_IN_BUFFER];

//void ssadc_callback(nrf_drv_saadc_evt_t const * p_event)
//{
//	float adc_value_mean = 0.0;
//	int16_t voltage = 0;
//	ret_code_t err_code;
//	
//	NRF_LOG_INFO("saadc event = %d: %s", p_event->type, nrfx_saadc_event_type[p_event->type]);
//	switch (p_event->type)
//	{
//		case NRFX_SAADC_EVT_DONE:
//		{
//			for(size_t i = 0; i < SAMPLES_IN_BUFFER; i++)
//			{
//				NRF_LOG_INFO("adc: %d", p_event->data.done.p_buffer[i]);
//				voltage = (p_event->data.done.p_buffer[i] * ( 600 * 6 )) >> 10;
//				NRF_LOG_INFO("voltage: %d", voltage);
//			}
//			err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
//			APP_ERROR_CHECK(err_code);
//		}break;
//		default:
//			break;
//	}
//}

void saadc_init(void)
{
	ret_code_t err_code;
	nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
	
	err_code = nrf_drv_saadc_init(NULL, NULL);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_channel_init(0, &channel_config);
	APP_ERROR_CHECK(err_code);
	
//	err_code = nrf_drv_saadc_buffer_convert(m_saadc_buffer, SAMPLES_IN_BUFFER);	/**< 看sdk中写的执行此函数时已经开始转换 */
//	APP_ERROR_CHECK(err_code);
	
}

int32_t saadc_mear(uint8_t num)
{
	ret_code_t err_code;
	nrf_saadc_value_t	adc_value = 0;
	int32_t	adc_value_mean = 0;
	
	for (size_t i = 0; i < num; i++)
	{
		err_code = nrfx_saadc_sample_convert(0, &adc_value);
		APP_ERROR_CHECK(err_code);
		NRF_LOG_INFO("adc_test: %d, %d", i, adc_value);
		adc_value_mean += (int32_t)adc_value;
	}
	
	/**< VOLTAGE(P) - VOLTAGE(N) = RESULT * (REFERENCE / GAIN) / 2^RESOLUTION */
	return ((adc_value_mean / num) * (REFERANCE_VOLTAGE_IN_MILIVOLTS * SAADC_GAIN_RECIPROCAL)) >> SAADC_RESOLUTION; 
	
}


/**< TIMER驱动测试。*/



#define USER_TIMER_MS 100


//#include "nrf_drv_timer.h"

void timer_event_handler(nrf_timer_event_t event, void* p_context)
{
	static size_t count_num = 0;
	
	nrf_drv_gpiote_out_toggle(USER_PIN_LED_2);
	count_num++;
	if (count_num < 10)
	{
		
	}
	else
	{
		NRF_LOG_INFO("adc in timer: %d\n", saadc_mear(8));
		count_num = 0;
		nrf_drv_timer_disable(&TIMER_DOOR);
	}
}

void timer_init(void)
{
	ret_code_t err_code;
	nrf_drv_timer_config_t timer_cfg = {
		.frequency 			= NRF_TIMER_FREQ_1MHz,						/**< 频率为1MHz */
		.mode				= NRF_TIMER_MODE_TIMER,						/**< 定时器模式 */	
		.bit_width			= NRF_TIMER_BIT_WIDTH_32,					/**< 使用32位宽度 */
		.interrupt_priority	= NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,	/**< 默认优先级为6 */
		.p_context			= NULL
	};
	
	err_code = nrf_drv_timer_init(&TIMER_DOOR, &timer_cfg, timer_event_handler);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_timer_extended_compare(&TIMER_DOOR, 
									NRF_TIMER_CC_CHANNEL1, 									/**< 通道0 */
									nrf_drv_timer_ms_to_ticks(&TIMER_DOOR, USER_TIMER_MS),	/**< 将时间转换为tick */
									NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK,					/**< 自动触发CLEAR任务 */
									true);													/**< 使能中断 */
	
	
}


/**< fds 模块 */

static bool m_fds_initialized = false;
static bool m_fds_file_del = false;
static uint8_t m_fds_write_success = 0;

#define FILE_ID 	0x0000

g_data_collection data_test[5] = {0};




static fds_record_desc_t m_record_desc;


static fds_find_token_t m_ftok;

static fds_flash_record_t fds_record_temp = {0};

static void fds_evt_handler(fds_evt_t const * p_evt)
{
	NRF_LOG_INFO("fds event id %d: %s", p_evt->id, fds_event_type[p_evt->id]);
	
	switch (p_evt->id)
    {
    	case FDS_EVT_INIT:
		{
			if (p_evt->result == NRF_SUCCESS)
			{
				m_fds_initialized = true;
				NRF_LOG_INFO("fds init success.");
			}
			else
			{
				NRF_LOG_INFO("fds error type %d: %s", p_evt->result - NRF_ERROR_FDS_ERR_BASE, fds_error_type[p_evt->result - NRF_ERROR_FDS_ERR_BASE]);
			}
		}break;
    	case FDS_EVT_WRITE:
		{
			if (p_evt->result == NRF_SUCCESS)
			{
				m_fds_write_success++;
//                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
//                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
//                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
			}
			else
			{
				NRF_LOG_INFO("fds error type %d: %s", p_evt->result - NRF_ERROR_FDS_ERR_BASE, fds_error_type[p_evt->result - NRF_ERROR_FDS_ERR_BASE]);
			}
    	}break;
		case FDS_EVT_DEL_RECORD:
		{
			if (p_evt->result == NRF_SUCCESS)
			{
//                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
//                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
//                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
			}
			else
			{
				NRF_LOG_INFO("fds error type %d: %s", p_evt->result - NRF_ERROR_FDS_ERR_BASE, fds_error_type[p_evt->result - NRF_ERROR_FDS_ERR_BASE]);
			}
		}break;
		case FDS_EVT_DEL_FILE:
		{
			if (p_evt->result == NRF_SUCCESS)
			{
				m_fds_file_del = true;
//                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
//                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
//                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
			}
			else
			{
				NRF_LOG_INFO("fds error type %d: %s", p_evt->result - NRF_ERROR_FDS_ERR_BASE, fds_error_type[p_evt->result - NRF_ERROR_FDS_ERR_BASE]);
			}
		}break;
    	default:
    		break;
    }
}

/**@brief fds存数据驱动测试
 *
 * @note 
 */

void data_test_print(g_data_collection* p_data_test, uint8_t num)
{
	for(size_t i = 0; i < num; i++)
    {
		NRF_LOG_INFO("global data print: %d", i);
		NRF_LOG_INFO("frame head: %s", (p_data_test + i)->p_frame_head);
		NRF_LOG_INFO("door state: %d", (p_data_test + i)->door_state);
		NRF_LOG_INFO("battery voltage: %d", (p_data_test + i)->battery_voltage);
		NRF_LOG_INFO("wake up source: %d", (p_data_test + i)->wkp_source);
		NRF_LOG_INFO("time stamp: %d", (p_data_test + i)->timestamp);
		NRF_LOG_INFO("number: %d", (p_data_test + i)->g_packet_no);
		NRF_LOG_INFO("error code: %d", (p_data_test + i)->err_code);
		NRF_LOG_INFO("frame end: %s", (p_data_test + i)->p_frame_end);
    }
}






/**@brief Function for application main entry.
 */
int main(void)
{
	ret_code_t err_code;
	fds_record_t	m_record = {0};
	g_data_collection data_temp[5] = {0};
	fds_stat_t m_stat = {0};
#ifndef RESERVING
    bool erase_bonds;
#endif //RESERVING
    // Initialize.
    log_init();
    timers_init();
//	uart_init();
#ifndef RESERVING
    buttons_leds_init(&erase_bonds);
#endif //RESERVING
	leds_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();
	delete_bonds();

    // Start execution.
    NRF_LOG_INFO("Template example started.");

	printf("MuiltPeripheral started.\r\n");
    application_timers_start();

    advertising_start();
	

	gpio_init();
	
	saadc_init();
//	err_code = nrf_drv_saadc_sample();
//	APP_ERROR_CHECK(err_code);
//	NRF_LOG_INFO("ADC TEST: %d", saadc_mear(8));

	timer_init();
//	app_timer_start(m_app_timer_test, APP_TIMER_TICKS(5000), NULL);
//	while (!flag_test)
//    {
//		printf("i");
//    }
	//初始化测试数组
	for(size_t i = 0; i < 5; i++)
    {
		data_test[i].p_frame_head = "225355";
		data_test[i].door_state = i%2;
		data_test[i].battery_voltage = 3600 + i;
		data_test[i].wkp_source = 0x65;
		data_test[i].timestamp = 3600 * i;
		data_test[i].g_packet_no = i;
		data_test[i].err_code = 0;
		data_test[i].p_frame_end = "AAOK";
    }
	data_test_print(data_test, 5);
	NRF_LOG_INFO("size of data_test: %d", sizeof(data_test));

	/**< 注册fds事件处理函数 */
	err_code = fds_register(fds_evt_handler);
	APP_ERROR_CHECK(err_code);
	
	/**<初始化fds模块 */
	err_code = fds_init();
	APP_ERROR_CHECK(err_code);
	while (!m_fds_initialized)
	{
		nrf_pwr_mgmt_run();
	}
	/*先清除一下file*/
	m_fds_file_del = false;
	err_code = fds_file_delete(FILE_ID);
	if (err_code != NRF_SUCCESS)
	{	
		NRF_LOG_INFO("file delete result: 0x%x, %s.", err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
	}
	while (!m_fds_file_del)
    {
		nrf_pwr_mgmt_run();
    }

	err_code = fds_gc();
	if (err_code != NRF_SUCCESS)
	{	
		NRF_LOG_INFO("garbage collect result: 0x%x, %s.", err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
	}
	
	err_code = fds_stat(&m_stat);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("stat->pages_available: %d.", m_stat.pages_available);
	NRF_LOG_INFO("stat->open_records: %d.", m_stat.open_records);
	NRF_LOG_INFO("stat->valid_records: %d.", m_stat.valid_records);
	NRF_LOG_INFO("stat->dirty_records: %d.", m_stat.dirty_records);
	NRF_LOG_INFO("stat->words_reserved: %d.", m_stat.words_reserved);
	NRF_LOG_INFO("stat->words_used: %d.", m_stat.words_used);
	NRF_LOG_INFO("stat->largest_contig: %d.", m_stat.largest_contig);
	NRF_LOG_INFO("stat->freeable_words: %d.", m_stat.freeable_words);
	NRF_LOG_INFO("stat->corruption: %d.", m_stat.corruption);
	
	memset(&m_record_desc, 0, sizeof(m_record_desc));

	
	/**< 写数据测试 */
	m_fds_write_success = 0;
	for(uint16_t i = 0x0001; i < 6; i++)
    {
		m_record.file_id = FILE_ID;
		m_record.key = i;
		m_record.data.p_data = &data_test[i-1];
		m_record.data.length_words = (sizeof(data_test[i-1]) + 3) / 4;
		err_code = fds_record_write(&m_record_desc, &m_record);
		if (err_code != NRF_SUCCESS)
		{
			NRF_LOG_INFO("write %d result: 0x%x, %s.", i-1, err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
		}
    }

	while (m_fds_write_success != 5)
    {
		nrf_pwr_mgmt_run();
    }
	
	/**< 统计数量信息 */
	err_code = fds_stat(&m_stat);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("stat->pages_available: %d.", m_stat.pages_available);
	NRF_LOG_INFO("stat->open_records: %d.", m_stat.open_records);
	NRF_LOG_INFO("stat->valid_records: %d.", m_stat.valid_records);
	NRF_LOG_INFO("stat->dirty_records: %d.", m_stat.dirty_records);
	NRF_LOG_INFO("stat->words_reserved: %d.", m_stat.words_reserved);
	NRF_LOG_INFO("stat->words_used: %d.", m_stat.words_used);
	NRF_LOG_INFO("stat->largest_contig: %d.", m_stat.largest_contig);
	NRF_LOG_INFO("stat->freeable_words: %d.", m_stat.freeable_words);
	NRF_LOG_INFO("stat->corruption: %d.", m_stat.corruption);
	
	
	/*It is required to zero the token before first use. */
	memset(&m_ftok, 0, sizeof(m_ftok));
	for(uint16_t i = 0x0001;i < 6; i++)
    {
		err_code = fds_record_find(FILE_ID, i, &m_record_desc, &m_ftok);
		if (err_code != NRF_SUCCESS)
		{	
			NRF_LOG_INFO("find result: 0x%x, %s.", err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
		}
		
		err_code = fds_record_open(&m_record_desc, &fds_record_temp);
		if (err_code != NRF_SUCCESS)
		{	
			NRF_LOG_INFO("open result: 0x%x, %s.", err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
		}
		
		memcpy(&data_temp[i-1], fds_record_temp.p_data, sizeof(data_temp[i-1]));
		
		err_code = fds_record_close(&m_record_desc);
		if (err_code != NRF_SUCCESS)
		{	
			NRF_LOG_INFO("close result: 0x%x, %s.", err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
		}
    }
	
	data_test_print(data_temp, 5);
	
	
	m_fds_file_del = false;
	err_code = fds_file_delete(FILE_ID);
	if (err_code != NRF_SUCCESS)
	{	
		NRF_LOG_INFO("file delete result: 0x%x, %s.", err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
	}
	while (!m_fds_file_del)
    {
		nrf_pwr_mgmt_run();
    }	

	err_code = fds_gc();
	if (err_code != NRF_SUCCESS)
	{	
		NRF_LOG_INFO("garbage collect result: 0x%x, %s.", err_code, fds_error_type[err_code - NRF_ERROR_FDS_ERR_BASE]);
	}	
	
	/**< 统计数量信息 */
	err_code = fds_stat(&m_stat);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("stat->pages_available: %d.", m_stat.pages_available);
	NRF_LOG_INFO("stat->open_records: %d.", m_stat.open_records);
	NRF_LOG_INFO("stat->valid_records: %d.", m_stat.valid_records);
	NRF_LOG_INFO("stat->dirty_records: %d.", m_stat.dirty_records);
	NRF_LOG_INFO("stat->words_reserved: %d.", m_stat.words_reserved);
	NRF_LOG_INFO("stat->words_used: %d.", m_stat.words_used);
	NRF_LOG_INFO("stat->largest_contig: %d.", m_stat.largest_contig);
	NRF_LOG_INFO("stat->freeable_words: %d.", m_stat.freeable_words);
	NRF_LOG_INFO("stat->corruption: %d.", m_stat.corruption);


	
	
	
	

	
	
	
	

    // Enter main loop.
    for (;;)
    {
        
		nrf_pwr_mgmt_run();
//		idle_state_handle();
    }
}


/**
 * @}
 */
