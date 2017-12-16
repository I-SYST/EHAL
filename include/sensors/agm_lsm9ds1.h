/*--------------------------------------------------------------------------
File   : agm_lsm9ds1.h

Author : Hoang Nguyen Hoan          			Nov. 18, 2017

Desc   : Implementation of ST LSM9DS1 sensor
			Accel, Gyro, Mag


Copyright (c) 2017, I-SYST inc., all rights reserved

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

#ifndef __AGM_LSM9DS1_H__
#define __AGM_LSM9DS1_H__

#include <stdint.h>

#include "iopincfg.h"
#include "accel_sensor.h"
#include "gyro_sensor.h"
#include "mag_sensor.h"

#pragma pack(push, 1)

#pragma pack(pop)

class AgmLsm9ds1 : public AccelSensor, public GyroSensor, public MagSensor {
public:
	bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf *pIntrf, Timer *pTimer);
	bool Read(ACCELSENSOR_DATA *pData);

private:
};

#endif // __AGM_LSM9DS1_H__
