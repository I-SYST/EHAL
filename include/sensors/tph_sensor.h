/*--------------------------------------------------------------------------
File   : tph_sensor.h

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : Generic TPH environment sensor abstraction
			- Temperature, Pressure, Humidity

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
#ifndef __TPH_SENSOR_H__
#define __TPH_SENSOR_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "sensor.h"


#pragma pack(push, 1)

//
// PTH sensor data
//
// 2 decimals fix point data
// value 1234 means 12.34
//
typedef struct __TPHSensor_Data {
	uint32_t Timestamp;		// Time stamp count in msec
	uint32_t Pressure;		// Barometric pressure in Pa no decimal
	int16_t  Temperature;	// Temperature in degree C, 2 decimals fixed point
	uint16_t Humidity;		// Relative humidity in %, 2 decimals fixed point
} TPHSENSOR_DATA;

#pragma pack(pop)

#pragma pack(push, 4)
//
// PTH sensor configuration
//
typedef struct __TPHSensor_Config {
	uint32_t		DevAddr;	// Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;		// Operating mode
	uint32_t		Freq;		// Sampling frequency in Hz if continuous mode is used
	int				TempOvrs;	// Oversampling measurement for temperature
	int				PresOvrs;	// Oversampling measurement for pressure
	int 			HumOvrs;	// Oversampling measurement for humidity
	uint32_t		FilterCoeff;// Filter coefficient select value (this value is device dependent)
} TPHSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

class TPHSensor : virtual public Sensor {
public:
	virtual bool Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf = NULL, Timer *pTimer = NULL) = 0;

	/**
	 * @brief	Read TPH data
	 * 			Read TPH data from device if available. If not
	 * 			return previous data.
	 *
	 * @param 	TphData : TPH data to return
	 *
	 * @return	true - new data
	 * 			false - old data
	 */
	virtual bool Read(TPHSENSOR_DATA &TphData) = 0;

	/**
	 * @brief	Start sampling data
	 *
	 * @return	true - success
	 */
	//virtual bool StartSampling() = 0;

	/**
	 * @brief	Read temperature
	 *
	 * @return	Temperature in degree C
	 */
	virtual float ReadTemperature() = 0;

	/**
	 * @brief	Read barometric pressure
	 *
	 * @return	Barometric pressure in Pascal
	 */
	virtual float ReadPressure() = 0;

	/**
	 * @brief	Read relative humidity
	 *
	 * @return	Relative humidity in %
	 */
	virtual float ReadHumidity() = 0;

protected:
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __TPH_SENSOR_H__
