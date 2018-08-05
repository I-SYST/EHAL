/**-------------------------------------------------------------------------
@file	imu.h

@brief	Implementation of an Inertial Measurement Unit

This a generic abstraction layer for IMU sensor fusion.  It is a mean to
provide a common interface to different sensor fusion library out there.


@author	Hoang Nguyen Hoan
@date	Aug. 1, 2018

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
#ifndef __IMU_H__
#define __IMU_H__

#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"

#define IMU_FEATURE_EULER				(1<<0)
#define IMU_FEATURE_QUAT				(1<<1)

typedef struct __Imu_Quat {
	uint32_t Timestamp;	//!< Time stamp count in msec
	union {
		int16_t	Q[4];
		struct {
			int16_t Q1;
			int16_t Q2;
			int16_t Q3;
			int16_t Q4;
		};
	};
} IMU_QUAT;

typedef struct __Imu_Euler {
	int16_t Yaw;
	int16_t Pitch;
	int16_t Roll;
} IMU_EULER;

class Imu {
public:

	virtual bool Init(AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual uint32_t Feature() { return vFeatures; }
	virtual uint32_t Feature(uint32_t Feature, bool bEnDis);
	virtual bool UpdateData() = 0;
	virtual bool Read(IMU_QUAT &Data) { Data = vQuat; return true; }
	virtual bool Read(IMU_EULER &Data) { Data = vEuler; return true; }

protected:
	AccelSensor *vpAccel;	//!< Pointer to accelerometer sensor
	GyroSensor *vpGyro;		//!< Pointer to gyro sensor
	MagSensor *vpMag;		//!< Pointer to magnetometer Sensor
	uint32_t vFeatures;		//!< Orable feature enabled bits - Bit set to 1 : Enabled, 0 : Disabled
	IMU_QUAT vQuat;			//!< Last updated quaternion values
	IMU_EULER vEuler;		//!< Last updated euler value
};



#endif // __IMU_H__
