/**-------------------------------------------------------------------------
@example	BlueIOThingy

@brief	Environmental Sensor BLE demo (Supports BME280, BME680, MS8607).

This application demo shows Thingy compatible environmental sensor using EHAL library.
It sends Temperature, Pressure, Humidity (TPH) data in Thingy compatible format.
Support I2C and SPI interface


NOTE : The BME680 Air Quality Index is undocumented.  It requires the library
Bosch Sensortec Environmental Cluster (BSEC) Software. Download from
https://www.bosch-sensortec.com/bst/products/all_products/bsec and put in
external folder as indicated on the folder tree.

The BSEC library must be initialized in the main application prior to initializing
this driver by calling the function

bsec_library_return_t res = bsec_init();

@author Hoang Nguyen Hoan
@date	Jan 24, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/

#ifndef __BLUEIOTHINGY_H__
#define __BLUEIOTHINGY_H__

#include "coredev/spi.h"
#include "timer_nrf5x.h"

#ifdef NRF51
#define BLE_GATT_ATT_MTU_DEFAULT 23
#endif

/// Thingy custom service UUID
#define THINGY_BASE_UUID    {{0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF}} //!< Used vendor specific UUID

#define BLE_UUID_TCS_SERVICE				0x0100                      /**< The UUID of the Thingy Configuration Service. */


BLESRVC *GetConfSrvcInstance();
uint32_t ConfSrvcInit();
BLESRVC *GetUISrvcInstance();
uint32_t UISrvcInit();
BLESRVC *GetEnvSrvcInstance();
uint32_t EnvSrvcInit();

void EnvSrvcNotifTemp(float Temp);
void EnvSrvcNotifPressure(float Press);
void EnvSrvcNotifHumi(uint8_t Humi);

BLESRVC *GetImuSrvcInstance();
uint32_t ImuSrvcInit();

bool ICM20948Init(DeviceIntrf * const pIntrf, Timer * const pTimer);

#endif // __BLUEIOTHINGY_H__

