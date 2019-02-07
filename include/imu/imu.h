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
//#define IMU_FEATURE_RAW_ACCEL				(1<<0)		//!< Raw accel sensor data, relevant for when sensor is known
//#define IMU_FEATURE_RAW_GYRO				(1<<1)		//!< Raw gyr sensor data, relevant for when sensor is known
//#define IMU_FEATURE_RAW_MAG					(1<<2)		//!< Raw mag sensor data, relevant for when sensor is known
//#define IMU_FEATURE_ACCEL					(1<<3)		//!< Converted accel data
//#define IMU_FEATURE_GYRO					(1<<4)		//!< Converted gyro data
//#define IMU_FEATURE_MAG						(1<<5)		//!< Converted mag data
#define IMU_FEATURE_EULER					(1<<0)		//!< Euler angles data
#define IMU_FEATURE_QUATERNION				(1<<1)		//!< Quaternion data
#define IMU_FEATURE_COMPASS					(1<<2)		//!< Compasss
#define IMU_FEATURE_GRAVITY					(1<<3)		//!< Gravity vector
#define IMU_FEATURE_EXTERNAL_ACCEL			(1<<4)		//!< External acceleration vector
#define IMU_FEATURE_TAP						(1<<5)		//!< Tap sensing
#define IMU_FEATURE_ROTATION				(1<<6)		//!< Rotation data
#define IMU_FEATURE_VIBRATION				(1<<7)		//!< Rotation data
#define IMU_FEATURE_PEDOMETER				(1<<8)		//!< Pedometer
#define IMU_FEATURE_CYCLING					(1<<9)		//!< Pedometer

typedef uint32_t	IMU_FEATURE;

#if 0
/// Quaternion data
/// The quaternion is a normalized number.  For more compact structure
/// convert it to 16 bits fixed point by multiplying with (1<<15) = 32768
typedef struct __Imu_Quat {
	uint32_t Timestamp;	//!< Time stamp count in usec
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
	uint32_t Timestamp;	//!< Time stamp count in usec
	int16_t Yaw;
	int16_t Pitch;
	int16_t Roll;
} IMU_EULER;
#endif

typedef struct __Imu_Quat {
	uint64_t Timestamp;	//!< Time stamp count in usec
	union {
		float Q[4];
		struct {
			float Q1;
			float Q2;
			float Q3;
			float Q4;
		};
	};
} IMU_QUAT;

typedef struct __Imu_Euler {
	uint64_t Timestamp;	//!< Time stamp count in usec
	float Yaw;
	float Pitch;
	float Roll;
} IMU_EULER;

typedef struct __Imu_Gravity {
	uint64_t Timestamp;	//!< Time stamp count in usec
	union {
		float Val[3];
		struct {
			float X;
			float Y;
			float Z;
		};
	};
} IMU_GRAVITY;

/// External acceleration vector
typedef struct __Imu_Extrn_Accel {
	uint64_t Timestamp;	//!< Time stamp count in usec
	union {
		float Val[3];
		struct {
			float X;
			float Y;
			float Z;
		};
	};
} IMU_EXT_ACCEL;

/// Pedometer
typedef struct __Imu_Pedometer {
	uint32_t Timestamp;		//!< Time stamp count in msec
    uint16_t StepCount;		//!< Number of step taken
    uint8_t Cadence; 		//!< in steps per minute
    float Direction; 		//!< Direction of the movement (yaw angle in degrees)
    uint16_t UpCount;		//!< Number of upstairs taken
    uint16_t DownCount; 	//!< Number of downstairs taken
    uint8_t StrideLength;	//!< in cm
    uint16_t TotalDistance;	//!< in dm
} IMU_PEDOMETER;

/// Rotation data
typedef struct __Imu_Rotation_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
    uint32_t Count; 		//!< Number of rotations
    uint16_t Rpm; 			//!< Revolutions per minute
} IMU_ROTATION;


typedef struct __Imu_Config {
	DEVEVTCB EvtHandler;
} IMU_CFG;

#ifdef __cplusplus

class Imu : virtual public Device {
public:

	virtual bool Init(const IMU_CFG &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual bool UpdateData() = 0;
	virtual void IntHandler() = 0;
	virtual bool Calibrate() = 0;
	virtual void SetAxisAlignmentMatrix(int8_t * const pMatrix) = 0;
	virtual bool Compass(bool bEn) = 0;
	virtual bool Pedometer(bool bEn) = 0;
	virtual bool Quaternion(bool bEn, int NbAxis) = 0;
	virtual bool Tap(bool bEn) = 0;
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
	virtual bool Read(ACCELSENSOR_RAWDATA &Data) { return vpAccel->Read(Data); }
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
	virtual bool Read(GYROSENSOR_RAWDATA &Data) { return vpGyro->Read(Data); }
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
	virtual bool Read(MAGSENSOR_RAWDATA &Data) { return vpMag->Read(Data); }
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

#endif // __cplusplus


#endif // __IMU_H__
