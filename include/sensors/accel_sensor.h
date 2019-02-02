/**-------------------------------------------------------------------------
@file	accel_sensor.h

@brief	Generic accelerometer sensor abstraction

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

#ifndef __ACCEL_SENSOR_H__
#define __ACCEL_SENSOR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

#pragma pack(push, 1)

/// Accelerometer raw sensor data
typedef struct __AccelSensor_Raw_Data {
	uint32_t Timestamp;	//!< Time stamp count in usec
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
	uint32_t Timestamp;	//!< Time stamp count in usec
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
	uint32_t		LPFreq;		//!< Low pass filter cutoff frequency in Hz
	bool 			bInter;		//!< true - enable interrupt
	DEVINTR_POL		IntPol;		//!< interrupt polarity
	ACCELINTCB		IntHandler;
} ACCELSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

/// Accel. sensor base class
class AccelSensor : virtual public Sensor {
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
	virtual bool Read(ACCELSENSOR_DATA &Data) {
		if (vData.Range == 0)
			return false;
		Data.Timestamp = vData.Timestamp;
		Data.X = (float)(vData.X * vData.Scale) / (float)vData.Range;
        Data.Y = (float)(vData.Y * vData.Scale) / (float)vData.Range;
        Data.Z = (float)(vData.Z * vData.Scale) / (float)vData.Range;
		return true;
	}

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
	virtual uint16_t Scale(uint16_t Value) { vScale = Value; return vScale; }

	virtual uint16_t Range() { return vRange; }
	virtual uint16_t Range(uint16_t Value) { vRange = Value; return vRange; }

	virtual uint32_t LowPassFreq() { return vLPFreq; }
	virtual uint32_t LowPassFreq(uint32_t Freq) { vLPFreq = Freq; return vLPFreq; }

protected:

	ACCELSENSOR_RAWDATA vData;		//!< Current sensor data updated with UpdateData()

private:
	ACCELINTCB vIntHandler;
	uint16_t vScale;			//!< Sensor data scale in g force (2g, 4g, ...)
	uint16_t vRange;            //!< ADC range of the sensor, contains max value for conversion factor
	uint32_t vLPFreq;			//!< Low pass filter cutoff frequency in Hz
};

#endif // __cplusplus

#endif // __ACCEL_SENSOR_H__
