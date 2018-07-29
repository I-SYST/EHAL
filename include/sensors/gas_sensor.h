/**-------------------------------------------------------------------------
@file	gas_sensor.h

@brief	Generic environment sensor abstraction, gas

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2017

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
class GasSensor : virtual public Sensor {
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
