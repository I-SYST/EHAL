/*--------------------------------------------------------------------------
File   : accel_sensor.h

Author : Hoang Nguyen Hoan          			Nov. 18, 2017

Desc   : Generic accelerometer sensor abstraction


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

#ifndef __ACCEL_SENSOR_H__
#define __ACCEL_SENSOR_H__

#include <stdint.h>

#include "iopincfg.h"
#include "sensor.h"

#pragma pack(push, 1)
typedef struct __AccelSensor_Data {
	uint32_t Timestamp;	// Time stamp count in msec
	int16_t x;			// X axis
	int16_t y;			// Y axis
	int16_t z;			// Z axis
} ACCELSENSOR_DATA;

typedef void (*ACCELINTCB)(ACCELSENSOR_DATA *pData);

typedef struct __AccelSensor_Config {
	uint32_t		DevAddr;	// Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;		// Operating mode
	uint32_t		Freq;		// Sampling frequency in Hz if continuous mode is used
	bool 			bInter;		// true - enable interrupt
	int				IntPort;
	int				IntPin;
	ACCELINTCB		IntHandler;
} ACCELSENSOR_CFG;

#pragma pack(pop)


class AccelSensor : virtual public Sensor {
public:
	virtual bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf *pIntrf, Timer *pTimer) = 0;
	virtual bool Read(ACCELSENSOR_DATA *pData) = 0;
	virtual uint8_t Scale() { return vScale; }
	virtual uint8_t Scale(uint8_t Value) { vScale = Value; return vScale; }

protected:
	virtual bool UpdateData() = 0;

	ACCELSENSOR_DATA vData;

private:
	ACCELINTCB vIntHandler;
	uint8_t vScale;
};

#endif // __ACCEL_SENSOR_H__
