/**-------------------------------------------------------------------------
@file	temp_sensor.h

@brief	Generic temperature sensor abstraction.

@author	Hoang Nguyen Hoan
@date	Feb. 12, 2017

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
#ifndef __TEMP_SENSOR_H__
#define __TEMP_SENSOR_H__

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

#pragma pack(push, 1)


/// @brief	Temperature sensor data
///
/// Structure defining temperature sensordata
typedef struct __TemperatureSensor_Data {
	uint64_t Timestamp;		//!< Time stamp count in usec
	int32_t  Temperature;	//!< Temperature in degree C, 2 decimals fixed point
} TEMPSENSOR_DATA;

#pragma pack(pop)

class TempSensor;

typedef void (*TEMPSENSOR_EVTCB)(TempSensor * const pSensor, TEMPSENSOR_DATA *pData);

#pragma pack(push, 4)

/// @brief	Temperature sensor configuration
///
typedef struct __TempSensor_Config {
	uint32_t		DevAddr;		//!< Either I2C dev address or CS index select if SPI is used
	SENSOR_OPMODE 	OpMode;			//!< Operating mode
	uint32_t		Freq;			//!< Sampling frequency in mHz (milliHerz) if continuous mode is used
	int				TempOvrs;		//!< Oversampling measurement for temperature
	uint32_t		FilterCoeff;	//!< Filter coefficient select value (this value is device dependent)
	TEMPSENSOR_EVTCB EvtHandler;	//!< Event handler
} TEMPSENSOR_CFG;

#pragma pack(pop)

#ifdef __cplusplus

/// Temperature sensor base class.  Sensor implementation must derive form this class
class TempSensor : public Sensor {
public:

	/**
	 * @brief	Initialize sensor (require implementation).
	 *
	 * @param 	CfgData : Reference to configuration data
	 * @param	pIntrf 	: Pointer to interface to the sensor.
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 * @param	pTimer	: Pointer to timer for retrieval of time stamp
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	virtual bool Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL) = 0;

	/**
	 * @brief	Read temperature data.
	 *
	 * Read temperature value from the last UpdateData
	 *
	 * @param 	Buff : Reference buffer to be filled with measured data
	 *
	 * @return	None
	 */
	virtual void Read(TEMPSENSOR_DATA &Data) { Data = vData; }

	/**
	 * @brief	Read temperature (require implementation).
	 *
	 * @return	Temperature in degree C
	 */
	virtual float ReadTemperature() { return (float)vData.Temperature / 100.0; }

protected:

	TEMPSENSOR_DATA 	vData;			//!< Last measured data
	TEMPSENSOR_EVTCB	vEvtHandler;	//!< Event handler
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __TEMP_SENSOR_H__
