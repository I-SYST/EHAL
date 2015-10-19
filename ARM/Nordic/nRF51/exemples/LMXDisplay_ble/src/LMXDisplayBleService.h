/*--------------------------------------------------------------------------
File   : LMXDisplayBleService.h

Author : Hoang Nguyen Hoan          Sep. 18, 2014

Desc   : LMXDisplay over custom BLE exemple.
		 Custom BLE service implementation

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

#ifndef __LMXDISPLAYBLESERVICE_H__
#define __LMXDISPLAYBLESERVICE_H__

#include <stdint.h>
#include "imm-nrf51.h"

//UUID : 9df5bd40-4070-11e4-84d0-0002a5d5c51b
#define LMXBLUE_UUID_BASE { 0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xd0, 0x84, \
							0xe4, 0x11, 0x70, 0x40, 0x40, 0xbd, 0xf5, 0x9d }
#define LMXBLUE_UUID_SERVICE		0xbd41
#define LMXBLUE_UUID_CTRLCHAR		0xbd42

#define	LMX_ATTR_LEFT_JUSTIFY	0
#define	LMX_ATTR_CENTER_JUSTIFY	1
#define	LMX_ATTR_RIGHT_JUSTIFY	2
#define	LMX_ATTR_SCROLL_LEFT	3
#define	LMX_ATTR_SCROLL_RIGHT	4

#define GATTWRBUFF_SIZE_MAX		256

typedef struct lmx_string {
	uint8_t		Attr;		// Attribute
	char		str[1];		// String
} LMX_STR;

typedef struct ble_lmxs_s ble_lmxs_t;

typedef void (*ble_write_handler_t) (ble_lmxs_t * p_lmxs, uint8_t *data, int offset, int len);

struct ble_lmxs_s {
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    char_handles;
    uint8_t                     uuid_type;
    uint8_t                     current_led_state;
    uint16_t                    conn_handle;
    bool                        is_notifying;
    ble_write_handler_t			write_handler;
};

typedef struct
{
    ble_write_handler_t	write_handler;
} ble_lmxs_init_t;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t ble_lmxs_init(ble_lmxs_t * p_lmxs, const ble_lmxs_init_t * p_lmxs_init);
void ble_lmxs_on_ble_evt(ble_lmxs_t * p_lmxs, ble_evt_t * p_ble_evt);


#ifdef __cplusplus
}
#endif

#endif // __LMXDISPLAYBLESERVICE_H__

