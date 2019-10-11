/**-------------------------------------------------------------------------
@file	gyro_sensor.h

@brief	Generic gyroscope sensor abstraction

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

@license

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

----------------------------------------------------------------------------*/

#ifndef __GYRO_SENSOR_H__
#define __GYRO_SENSOR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

#pragma pack(push, 1)

/// Gyroscope raw sensor data
typedef struct __GyroSensor_Raw_Data {
    uint64_t Timestamp; 	//!< Time stamp count in usec
    uint16_t Sensitivity;	//!< Scale in degree per second of the sensor
    uint16_t Range;     	//!< Sensor ADC range
    union {
        int16_t Val[3];
        struct {
            int16_t X;      //!< X axis
            int16_t Y;      //!< Y axis
            int16_t Z;      //!< Z axis
        };
    };
} GYROSENSOR_RAWDATA;

/// Gyroscope sensor data
typedef struct __GyroSensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	union {
	    float Val[3];
		struct {
	        float X;		//!< X axis
	        float Y;		//!< Y axis
	        float Z;		//!< Z axis
		};
	};
} GYROSENSOR_DATA;

typedef struct __GyroSensor_Config {
	uint32_t		DevAddr;		//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;			//!< Operating mode
	uint32_t		Freq;			//!< Sampling frequency in mHz (miliHertz) if continuous mode is used
	uint16_t		Sensitivity;	//!< Sensitivity level per degree per second
	uint32_t		FltrFreq;		//!< Filter cutoff frequency in mHz
	bool 			bInter;			//!< true - enable interrupt
	DEVINTR_POL		IntPol;			//!< Interrupt pin polarity
} GYROSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

class GyroSensor : public Sensor {
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
	virtual bool Init(const GYROSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer) = 0;

    virtual bool Read(GYROSENSOR_RAWDATA &Data) {
        Data = vData;
        return true;
    }

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
	virtual bool Read(GYROSENSOR_DATA &Data);

	/**
	 * @brief	Get the current sensitivity value.
	 *
	 * @return	Sensitivity value
	 */
	virtual uint16_t Sensitivity() { return vSensitivity; }

	/**
	 * @brief	Set the current sensitivity value.
	 *
	 * NOTE : Implementer must overload this function to add require hardware implement then call
	 * this function to keep the scale value internally and return the real hardware scale value.
	 *
	 * @param 	Value : Wanted sensitivity value
	 *
	 * @return	Real sensitivity value
	 */
	virtual uint16_t Sensitivity(uint16_t Value);

    virtual void SetCalibration(float (&Gain)[3][3], float (&Offset)[3]);
	virtual void ClearCalibration();

protected:

	uint16_t vSensitivity;	    //!< Sensitivity level per degree per second
	GYROSENSOR_RAWDATA vData;	//!< Current sensor data updated with UpdateData()
	float vCalibGain[3][3];
	float vCalibOffset[3];
};

#endif // __cplusplus

#endif // __GYRO_SENSOR_H__
