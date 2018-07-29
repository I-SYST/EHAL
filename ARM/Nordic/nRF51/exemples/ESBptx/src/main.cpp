/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_drv_timer.h"
//#include "nrf_delay.h"
//#include "nrf_gpio.h"
//#include "boards.h"
//#include "nrf_delay.h"
#include "app_util.h"
#define NRF_LOG_MODULE_NAME "APP"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
#include "idelay.h"
#include "istddef.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"

nrf_drv_timer_t g_TimerPulse = {
	NRF_TIMER0,
	0,
	1
};

//        {.pipe = _pipe, .length = NUM_VA_ARGS(__VA_ARGS__), .data = {__VA_ARGS__}};
 //       STATIC_ASSERT(NUM_VA_ARGS(__VA_ARGS__) > 0 && NUM_VA_ARGS(__VA_ARGS__) <= 63)
static nrf_esb_payload_t tx_payload = { //NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);
	4, 0, 0
};

static nrf_esb_payload_t rx_payload;

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// UART configuration data

static const IOPINCFG s_UartPins[] = {
	{BLUEIO_UART_RX_PORT, BLUEIO_UART_RX_PIN, BLUEIO_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{BLUEIO_UART_TX_PORT, BLUEIO_UART_TX_PIN, BLUEIO_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{BLUEIO_UART_CTS_PORT, BLUEIO_UART_CTS_PIN, BLUEIO_UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{BLUEIO_UART_RTS_PORT, BLUEIO_UART_RTS_PIN, BLUEIO_UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

const UARTCFG g_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPINCFG),
	1000000,
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
	UART_FLWCTRL_NONE,
	true,
	7,
	nRFUartEvthandler,
	false,
};

// UART object instance
UART g_Uart;

static const IOPINCFG s_ButPin[] = {
	{BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, BLUEIO_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// BUT1
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// BUT2
};

int g_DelayCnt = 0;
volatile bool g_Flag = false;
uint32_t g_Count = 0;

int32_t ESBWritePayload()
{
    tx_payload.noack = true;
    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
    {
        // Toggle one of the LEDs.
        //nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
        //nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
        //nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
        //nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));

    }
    else
    {
        printf("Sending packet failed\r\n");
    }
}

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
		//	ESBWritePayload();
			//app_sched_event_put(NULL, 0, UartRxChedHandler);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    //static uint32_t i;
    //uint32_t led_to_invert = ((i++) % LEDS_NUMBER);

    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
           // bsp_board_led_invert(led_to_invert);
            tx_payload.noack = false;
            tx_payload.data[0] = g_Count & 0xFF;
            tx_payload.data[1] = (g_Count >> 8L) & 0xFF;
            tx_payload.data[2] = (g_Count >> 16L) & 0xFF;
            tx_payload.data[3] = (g_Count >> 24L) & 0xFF;
            if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
            {
                // Toggle one of the LEDs.
                //nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
                //nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
                //nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
                //nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
                g_Count++;
//                tx_payload.data[1]++;
            }
            else
            {
                printf("Sending packet failed\r\n");
            }

        	g_Flag = true;
          break;

        default:
            //Do nothing.
            break;
    }
}

void ButInterHandler(int IntNo)
{
//	ESBWritePayload();
    tx_payload.noack = true;
    if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
    {
        // Toggle one of the LEDs.
        //nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
        //nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
        //nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
        //nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
        tx_payload.data[1]++;
    }
    else
    {
        printf("Sending packet failed\r\n");
    }

	g_Flag = true;
	//BleSrvcCharNotify(&g_UartBleSrvc, 0, &buff, 1);
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
           //g_Uart.printf("TX SUCCESS EVENT\r\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
        	g_Uart.printf("TX FAILED EVENT\r\n");
            (void) nrf_esb_flush_tx();
            (void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
        	g_Uart.printf("RX RECEIVED EVENT\r\n");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    //NRF_LOG_DEBUG("RX RECEIVED PAYLOAD\r\n");
                }
            }
            break;
    }
    NRF_GPIO->OUTCLR = 0xFUL << 12;
    NRF_GPIO->OUTSET = (p_event->tx_attempts & 0x0F) << 12;
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    //nrf_gpio_range_cfg_output(8, 15);
    //bsp_board_leds_init();
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE6, 0xE6, 0xE6, 0xE6};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.tx_output_power			= NRF_ESB_TX_POWER_4DBM;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 200;
    nrf_esb_config.retransmit_count			= 3;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = true;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

void HardwareInit()
{
    uint32_t time_ms = 1000; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    g_Uart.Init(g_UartCfg);

	IOPinCfg(s_ButPin, 2);

	IOPinEnableInterrupt(0, 7, BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, IOPINSENSE_LOW_TRANSITION , ButInterHandler);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&g_TimerPulse, &timer_cfg, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_us_to_ticks(&g_TimerPulse, time_ms);

    nrf_drv_timer_extended_compare(
         &g_TimerPulse, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&g_TimerPulse);

}

int main(void)
{
    ret_code_t err_code;

    HardwareInit();

    g_Uart.printf("ESBptx\n");

    //err_code = NRF_LOG_INIT(NULL);
    //APP_ERROR_CHECK(err_code);

    clocks_start();

    err_code = esb_init();
    //APP_ERROR_CHECK(err_code);

    //bsp_board_leds_init();

    //NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example running.\r\n");

    while (true)
    {
        //NRF_LOG_DEBUG("Transmitting packet %02x\r\n", tx_payload.data[1]);
    	__WFE();
    	if (g_Flag)
    	{
    		g_Flag = false;
    	//	ESBWritePayload();
    	}
    	/*
        tx_payload.noack = false;
        if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        {
            // Toggle one of the LEDs.
            //nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
            //nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
            //nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
            //nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
            tx_payload.data[1]++;
        }
        else
        {
            //NRF_LOG_WARNING("Sending packet failed\r\n");
        }

        usDelay(50000);*/
    }
}
/*lint -restore */
