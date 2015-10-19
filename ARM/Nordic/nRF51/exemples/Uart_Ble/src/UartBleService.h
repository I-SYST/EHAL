/*--------------------------------------------------------------------------
File   : UartBleService.h

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

#ifndef __UARTBLESERVICE_H__
#define __UARTBLESERVICE_H__

#include <stdint.h>

// UUID : 00000000-59a4-4d2b-8479-8bbdbcf77fcd
//#define BLINKY_UUID_BASE {0xcd, 0x7f, 0xf7, 0xbc, 0xbd, 0x8b, 0x79, 0x84, 0x2b, 0x4d, 0xa4, 0x59, 0x0, 0x0, 0x0, 0x0}
//#define BLINKY_UUID_SERVICE			1
//#define BLINKY_UUID_CHAR			2

#define UART_UUID_BASE 		{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}

#define UART_UUID_SERVICE		1
#define UART_UUID_DATA_CHAR  	2
#define UART_UUID_CTRL_CHAR		3


#define UART_MAX_DATA_LEN            (GATT_MTU_SIZE_DEFAULT - 3)  /**< Maximum length of data (in bytes) that can be transmitted by the Nordic UART service module to the peer. */

#define UART_MAX_RX_CHAR_LEN         UART_MAX_DATA_LEN         		/**< Maximum length of the RX Characteristic (in bytes). */
#define UART_MAX_TX_CHAR_LEN         20                           /**< Maximum length of the TX Characteristic (in bytes). */

typedef struct ble_uarts_s ble_uarts_t;

typedef void (*ble_uarts_write_handler_t) (ble_uarts_t * p_uarts, uint8_t *data, int len);

struct ble_uarts_s {
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    data_char_handles;
    ble_gatts_char_handles_t    ctrl_char_handles;
    uint8_t                     uuid_type;
    uint16_t                    conn_handle;
    bool                        is_notifying;
    ble_uarts_write_handler_t	write_handler;
};

typedef struct
{
    ble_uarts_write_handler_t	write_handler;
} ble_uarts_init_t;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t ble_uarts_init(ble_uarts_t * p_uarts, const ble_uarts_init_t * p_uarts_init);
void ble_uarts_on_ble_evt(ble_uarts_t * p_uarts, ble_evt_t * p_ble_evt);
uint32_t ble_uarts_data_send(ble_uarts_t * p_uarts, uint8_t *pData, int16_t Len);


#ifdef __cplusplus
}
#endif

#endif // __BLINKYBLESERVICE_H__

