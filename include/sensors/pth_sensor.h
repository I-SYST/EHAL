/*--------------------------------------------------------------------------
File   : pth_sensor.h

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : Generic environment sensor abstraction
			- Temperature, Humidity, Barometric pressure

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
#ifndef __PTH_SENSOR_H__
#define __PTH_SENSOR_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "device.h"

#pragma pack(push, 4)

//
// PTH sensor operating mode
//
typedef enum __PthSensor_OpMode {
	PTHSENSOR_OPMODE_SLEEP,
	PTHSENSOR_OPMODE_SINGLE,		// Single capture
	PTHSENSOR_OPMODE_CONTINUOUS		// Continuous capture
} PTHSENSOR_OPMODE;

//
// PTH sensor data
//
// 2 decimals fix point data
// value 1234 means 12.34
//
typedef struct __PthSensor_Data {
	uint32_t Pressure;		// Barometric pressure in Pa
	int16_t  Temperature;	// Temperature in degree C
	uint16_t Humidity;		// Relative humidity in %
} PTHSENSOR_DATA;

//
// PTH sensor configuration
//
typedef struct __PthSensor_Config {
	uint32_t		DevAddr;	// Either I2C dev address or CS index select if SPI is used
	PTHSENSOR_OPMODE OpMode;	// Operating mode
	uint32_t		Freq;		// Sampling frequency in Hz if continuous mode is used
} PTHSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

class PTHSensor : public Device {
public:
	virtual bool Init(const PTHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf) = 0;

	/**
	 * @brief	Read PTH data
	 * 			Read PTH data from device if available. If not
	 * 			return previous data.
	 *
	 * @param PthData : PTH data to return
	 *
	 * @return	true - new data
	 * 			false - old data
	 */
	virtual bool ReadPTH(PTHSENSOR_DATA &PthData) = 0;

	/**
	 * @brief Set operating mode
	 *
	 * @param OpMode : Operating mode
	 * 					- PTHSENSOR_OPMODE_SLEEP
	 * 					- PTHSENSOR_OPMODE_SINGLE
	 * 					- PTHSENSOR_OPMODE_CONTINUOUS
	 * @param Freq : Sampling frequency in Hz for continuous mode
	 *
	 * @return true- if success
	 */
	virtual bool SetMode(PTHSENSOR_OPMODE OpMode, uint32_t Freq) = 0;

	/**
	 * @brief	Start sampling data
	 *
	 * @return	true - success
	 */
	virtual bool StartSampling() = 0;

	float ReadTemperature() {
		PTHSENSOR_DATA pthdata;
		ReadPTH(pthdata);
		return (float)pthdata.Temperature / 100.0;
	}

	float ReadPressure() {
		PTHSENSOR_DATA pthdata;
		ReadPTH(pthdata);
		return (float)pthdata.Pressure / 100.0;
	}

	float ReadHumidity() {
		PTHSENSOR_DATA pthdata;
		ReadPTH(pthdata);
		return (float)pthdata.Humidity / 100.0;
	}
protected:
	PTHSENSOR_OPMODE vOpMode;
	uint32_t vSampFreq;			// Sampling frequency in Hz
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __PTH_SENSOR_H__
