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

/// IMU processing features
#define IMU_FEATURE_RAW_ACCEL				(1<<0)		//!< Raw accel sensor data, relevant for when sensor is known
#define IMU_FEATURE_RAW_GYRO				(1<<1)		//!< Raw gyr sensor data, relevant for when sensor is known
#define IMU_FEATURE_RAW_MAG					(1<<2)		//!< Raw mag sensor data, relevant for when sensor is known
#define IMU_FEATURE_ACCEL					(1<<3)		//!< Converted accel data
#define IMU_FEATURE_GYRO					(1<<4)		//!< Converted gyro data
#define IMU_FEATURE_MAG						(1<<5)		//!< Converted mag data
#define IMU_FEATURE_EULER					(1<<6)		//!< Euler angles data
#define IMU_FEATURE_QUAT					(1<<7)		//!< Quaternion data
#define IMU_FEATURE_ORIENTATION				(1<<8)		//!< Orientation
#define IMU_FEATURE_TAP						(1<<9)		//!< Tap sensing
#define IMU_FEATURE_PEDOMETER				(1<<10)		//!< Pedometer

typedef uint32_t	IMU_FEATURE;

/// Quaternion data
/// The quaternion is a normalized number.  For more compact structure
/// convert it to 16 bits fixed point by multiplying with (1<<15) = 32768
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
	uint32_t Timestamp;	//!< Time stamp count in msec
	int16_t Yaw;
	int16_t Pitch;
	int16_t Roll;
} IMU_EULER;

typedef struct __Imu_Config {
	DEVEVTCB EvtHandler;
} IMU_CFG;

class Imu : virtual public Device {
public:

//	virtual bool Init(const IMU_CFG &Cfg, uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual bool Init(const IMU_CFG &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual bool UpdateData() = 0;
	virtual void IntHandler() = 0;
	virtual bool Read(IMU_QUAT &Data) { Data = vQuat; return true; }
	virtual bool Read(IMU_EULER &Data) { Data = vEuler; return true; }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(ACCELSENSOR_DATA &Data) { return vpAccel->Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(GYROSENSOR_DATA &Data) { return vpGyro->Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(MAGSENSOR_DATA &Data) { return vpMag->Read(Data); }

	virtual IMU_FEATURE Feature() { return vActiveFeature; }
	virtual IMU_FEATURE Feature(IMU_FEATURE FeatureBit, bool bEnDis);

	/**
	 * @brief	Set data rate in miliHz
	 *
	 * @param	DataRate : Data rate in miliHz
	 *
	 * @return	Actual data rate set
	 */
	virtual uint32_t Rate(uint32_t DataRate) { vRate = DataRate; return vRate; }

	/** @brief	Get data rate
	 *
	 * @return	Data rate in miliHz
	 */
	virtual uint32_t Rate() { return vRate; }

protected:
//	Timer *vpTimer;			//!< Pointer to Timer object for timestamping
	AccelSensor *vpAccel;	//!< Pointer to accelerometer sensor
	GyroSensor *vpGyro;		//!< Pointer to gyro sensor
	MagSensor *vpMag;		//!< Pointer to magnetometer Sensor
	IMU_FEATURE vActiveFeature;	//!< Orable feature enabled bits - Bit set to 1 : Enabled, 0 : Disabled
	IMU_QUAT vQuat;			//!< Last updated quaternion values
	IMU_EULER vEuler;		//!< Last updated euler value
	uint32_t vRate;			//!< Data rate in mHz (mili-Hz)
};



#endif // __IMU_H__
