/**-------------------------------------------------------------------------
@file	gas_sensor.h

@brief	Generic environment sensor abstraction, gas

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2017

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
#ifndef __GAS_SENSOR_H__
#define __GAS_SENSOR_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "coredev/iopincfg.h"
#include "sensors/sensor.h"

/** @addtogroup Sensors
  * @{
  */

#define GASSENSOR_MEASUREMENT_POINT_MAX			10	//!< Max number of gas measurement point

#pragma pack(push, 1)

/// @brief	Gas sensor data
typedef struct __GasSensor_Data {
	uint64_t Timestamp;			//!< Time stamp in usec
	uint32_t GasRes[GASSENSOR_MEASUREMENT_POINT_MAX];	//!< Gas resistance value
	int		 MeasIdx;			//!< Latest measure point index
	float	 AirQualIdx;		//!< Air Quality Index
} GASSENSOR_DATA;

/// @brief	Heating temperature setting point for the heating profile
typedef struct __GasSensor_Heater {
	int16_t	Temp;				//!< Heater temperature in Celsius in 2 decimal fix point (3145 = 31.45 Degree)
	int16_t Dur;				//!< heating duration in msec
} GASSENSOR_HEAT;

#pragma pack(pop)

#pragma pack(push, 4)

/// @brief	Gas sensor configuration data
typedef struct __GasSensor_Config {
	uint32_t		DevAddr;	//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;		//!< Operating mode
	uint32_t		Freq;		//!< Sampling frequency in Hz if continuous mode is used
	int				NbHeatPoint;//!< Number of heating point in profile
	const GASSENSOR_HEAT *pHeatProfile;	//!< Pointer to array of heating temperature profile
} GASSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

/// Gas sensor base class, implementation must derive this class
class GasSensor : public Sensor {
public:

	/**
	 * @brief	Initialize sensor (require implementation).
	 *
	 * @param 	CfgData : Reference to configuration data
	 * @param	pIntrf 	: Pointer to interface to the sensor.
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 * @param	pTimer	: Pointer to timer for retrieval of time stamp (optional)
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const GASSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) = 0;

	/**
	 * @brief	Read gas sensor data (require implementation).
	 *
	 * Read gas data from device if available. If not return previous data
	 *
	 * @param 	GasData : Gas sensor data
	 *
	 * @return
	 * 			- true	: If new data is returned
	 * 			- false	: If old data is returned
	 */
	virtual bool Read(GASSENSOR_DATA &GasData) = 0;

	/**
	 * @brief	Set gas heating profile (require implementation).
	 *
	 * @param	Count : Number of heating temperature settings
	 * @param	pProfile : Pointer to array of temperature/duration settings
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	virtual bool SetHeatingProfile(int Count, const GASSENSOR_HEAT * const pProfile) = 0;

protected:

	GASSENSOR_DATA vGasData;	//!< Last measured data
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __GAS_SENSOR_H__
