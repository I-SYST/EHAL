/**-------------------------------------------------------------------------
@file	accel_sensor.h

@brief	Generic accelerometer sensor abstraction

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

@license

MIT License

Copyright (c) 2017-2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#ifndef __ACCEL_SENSOR_H__
#define __ACCEL_SENSOR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

#pragma pack(push, 1)

/// Accelerometer raw sensor data
typedef struct __AccelSensor_Raw_Data {
	uint64_t Timestamp;	//!< Time stamp count in usec
	uint16_t Scale;		//!< Scale in G of the sensor
	uint16_t Range;		//!< Sensor ADC range
	union {
		int16_t Val[3];
		struct {
			int16_t X;			//!< X axis
			int16_t Y;			//!< Y axis
			int16_t Z;			//!< Z axis
		};
	};
} ACCELSENSOR_RAWDATA;

/// Accelerometer sensor data in G
typedef struct __AccelSensor_Data {
	uint64_t Timestamp;	//!< Time stamp count in usec
	union {
		float Val[3];
		struct {
		    float X;			//!< X axis
		    float Y;			//!< Y axis
		    float Z;			//!< Z axis
		};
	};
} ACCELSENSOR_DATA;

typedef void (*ACCELINTCB)(ACCELSENSOR_DATA *pData);

/// Accel configuration data
typedef struct __AccelSensor_Config {
	uint32_t		DevAddr;	//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;		//!< Operating mode
	uint32_t		Freq;		//!< Sampling frequency in mHz (miliHertz) if continuous mode is used
	uint16_t		Scale;		//!< Accelerometer sensor scale in g force (2g, 4g, ...
	uint32_t		FltrFreq;	//!< Filter cutoff frequency in mHz
	bool 			bInter;		//!< true - enable interrupt
	DEVINTR_POL		IntPol;		//!< interrupt polarity
	ACCELINTCB		IntHandler;
} ACCELSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

/// Accel. sensor base class
class AccelSensor : public Sensor {
public:

	/**
	 * @brief	Sensor initialization
	 *
	 * @param 	Cfg		: Sensor configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer) = 0;

    /**
     * @brief   Read last updated sensor raw data
     *
     * This function read the currently stored data last updated by UdateData().
     * Device implementation can add validation if needed and return true or false
     * in the case of data valid or not.  This default implementation only returns
     * the stored data with success.
     *
     * @param   Data : Reference to data storage for the returned data
     *
     * @return  True - Success.
     */
	virtual bool Read(ACCELSENSOR_RAWDATA &Data) {
		Data = vData;
		return true;
	}

	/**
	 * @brief	Read converted sensor data
	 *
	 * This function gets the currently stored raw data last updated by UdateData() and
	 * convert it to real G force unit.  Device implementation can add validation if needed and
	 * return true or false in the case of data valid or not.  This default implementation
	 * only returns the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(ACCELSENSOR_DATA &Data);

	/**
	 * @brief	Get the current G scale value.
	 *
	 * @return	G scale value
	 */
	virtual uint16_t Scale() { return vScale; }

	/**
	 * @brief	Set the current G scale value.
	 *
	 * NOTE : Implementer must overload this function to add require hardware implement then call
	 * this function to keep the scale value internally and return the real hardware scale value.
	 *
	 * @param 	Value : Wanted scale value
	 *
	 * @return	Real scale value
	 */
	virtual uint16_t Scale(uint16_t Value);

    virtual void SetCalibration(float (&Gain)[3][3], float (&Offset)[3]);
	virtual void ClearCalibration();
	virtual bool StartSampling() { return true; }

	AccelSensor() {
		Type(SENSOR_TYPE_ACCEL);
		ClearCalibration();
	}

protected:

	ACCELSENSOR_RAWDATA vData;		//!< Current sensor data updated with UpdateData()
	ACCELINTCB vIntHandler;

private:
	uint16_t vScale;			//!< Sensor data scale in g force (2g, 4g, ...)
	float vCalibGain[3][3];
	float vCalibOffset[3];
};

#endif // __cplusplus

#endif // __ACCEL_SENSOR_H__
