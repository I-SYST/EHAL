/*--------------------------------------------------------------------------
File   : BlinkyBleService.h

Author : Hoang Nguyen Hoan          Aug. 18, 2014

Desc   : Blinky over custom BLE exemple.
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

#ifndef __BLINKYBLESERVICE_H__
#define __BLINKYBLESERVICE_H__

#include <stdint.h>

// UUID : 00000000-59a4-4d2b-8479-8bbdbcf77fcd
#define BLINKY_UUID_BASE {0xcd, 0x7f, 0xf7, 0xbc, 0xbd, 0x8b, 0x79, 0x84, 0x2b, 0x4d, 0xa4, 0x59, 0x0, 0x0, 0x0, 0x0}
#define BLINKY_UUID_SERVICE			1
#define BLINKY_UUID_CHAR			2


#define BLINKY_CMD_OFF			0
#define BLINKY_CMD_ON			1
#define BLINKY_CMD_CFGOUTPUT	2
#define BLINKY_CMD_CFGINPUT 	3

// Data info
typedef struct blinky_cmd {
	uint8_t		iopin;		// I/O pin # to control
	uint8_t		cmd;		// Command
} BLINKY_CMD;

typedef struct ble_blinkys_s ble_blinkys_t;

typedef void (*ble_blinkys_write_handler_t) (ble_blinkys_t * p_blinkys, uint8_t *data, int len);

struct ble_blinkys_s {
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    blinky_char_handles;
    uint8_t                     uuid_type;
    uint8_t                     current_led_state;
    uint16_t                    conn_handle;
    bool                        is_notifying;
    ble_blinkys_write_handler_t	write_handler;
};

typedef struct
{
    ble_blinkys_write_handler_t	write_handler;
} ble_blinkys_init_t;

#ifdef __cplusplus
extern "C" {
#endif

uint32_t ble_blinkys_init(ble_blinkys_t * p_blinkys, const ble_blinkys_init_t * p_blinkys_init);
void ble_blinkys_on_ble_evt(ble_blinkys_t * p_blinkys, ble_evt_t * p_ble_evt);
uint32_t ble_blinkys_on_data_change(ble_blinkys_t * p_blinkys, uint8_t *data, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif // __BLINKYBLESERVICE_H__

