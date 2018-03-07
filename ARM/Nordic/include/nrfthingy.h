/**-------------------------------------------------------------------------
@example	nrfthingy.h

@brief	Thingy common defintion

This file contains general common definition needed to create Nrodic Thingy
compatible firmware.

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
#ifndef __TPHTHINGY_H__
#define __TPHTHINGY_H__

#define DEVICE_NAME                     "EHAL_Thingy"                            /**< Name of device. Will be included in the advertising data. */

#define THINGY_BASE_UUID    {{0x42, 0x00, 0x74, 0xA9, 0xFF, 0x52, 0x10, 0x9B, 0x33, 0x49, 0x35, 0x9B, 0x00, 0x00, 0x68, 0xEF}} /**< Used vendor specific UUID. */

#define BLE_UUID_TCS_SERVICE 			0x0100                      /**< The UUID of the Thingy Configuration Service. */

BLESRVC *GetConfServiceInstance();
uint32_t InitConfService();

BLESRVC *GetEnvServiceInstance();
uint32_t InitEnvService();
void EnvSrvcNotifTemp(float Temp);
void EnvSrvcNotifPressure(float Press);
void EnvSrvcNotifHumi(uint8_t Humi);

#endif // __TPHTHINGY_H__

