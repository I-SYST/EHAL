/**-------------------------------------------------------------------------
@file	tph_bme280.h

@brief	TphSensor implementation of Bosch #BME280 Temperature, Pressure, Humidity sensor.

Key features

- Package : 2.5 mm x 2.5 mm x 0.93 mm metal lid LGA
- Digital interface : I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 MHz)
- Supply voltage : VDD main supply voltage range: 1.71 V to 3.6 V, VDDIO interface voltage range: 1.2 V to 3.6 V
- Current consumption : 1.8 μA @ 1 Hz humidity and temperature. 2.8 μA @ 1 Hz pressure and temperature.
3.6 μA @ 1 Hz humidity, pressure and temperature. 0.1 μA in sleep mode
- Operating range : -40...+85 °C, 0...100 % rel. humidity, 300...1100 hPa
- Humidity sensor and pressure sensor can be independently enabled / disabled
- Register and performance compatible to Bosch Sensortec BMP280 digital pressure sensor

Key parameters for humidity sensor1
- Response time : 1 s
- Accuracy tolerance : ±3 % relative humidity
- Hysteresis : ±1% relative humidity

Key parameters for pressure sensor

- RMS Noise 0.2 Pa, equiv. to 1.7 cm
- Offset temperature coefficient ±1.5 Pa/K, equiv. to ±12.6 cm at 1 °C temperature change

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
#ifndef __TPH_BME280_H__
#define __TPH_BME280_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "coredev/iopincfg.h"
#include "sensors/temp_sensor.h"
#include "sensors/press_sensor.h"
#include "sensors/humi_sensor.h"


/** @addtogroup Sensors
  * @{
  */

// Device address depending on SDO wiring
#define BME280_I2C_DEV_ADDR0			0x76	//!< Device address when SDO to GND
#define BME280_I2C_DEV_ADDR1			0x77	//!< Device address when SDO to VCC

#define BME280_REG_HUM_LSB				0xFE
#define BME280_REG_HUM_MSB				0xFD
#define BME280_REG_TEMP_XLSB			0xFC
#define BME280_REG_TEMP_LSB				0xFB
#define BME280_REG_TEMP_MSB				0xFA
#define BME280_REG_PRESS_XLSB			0xF9
#define BME280_REG_PRESS_LSB			0xF8
#define BME280_REG_PRESS_MSB			0xF7

#define BME280_REG_CONFIG				0xF5

#define BME280_REG_CONFIG_FILTER_BITPOS		2
#define BME280_REG_CONFIG_FILTER_MASK		(7 << BME280_REG_CONFIG_FILTER_BITPOS)

#define BME280_REG_CTRL_MEAS			0xF4

#define BME280_REG_CTRL_MEAS_OSRS_P_BITPOS	2
#define BME280_REG_CTRL_MEAS_OSRS_P_MASK	(7 << BME280_REG_CTRL_MEAS_OSRS_P_BITPOS)
#define BME280_REG_CTRL_MEAS_OSRS_T_BITPOS	5
#define BME280_REG_CTRL_MEAS_OSRS_T_MASK	(7 << BME280_REG_CTRL_MEAS_OSRS_T_BITPOS)

#define BME280_REG_STATUS				0xF3

#define BME280_REG_STATUS_MEASURING			(1<<3)
#define BME280_REG_STATUS_IM_UPDATE			(1<<0)

#define BME280_REG_CTRL_HUM				0xF2
#define BME280_REG_CALIB_26_41_START	0xE1
#define BME280_REG_RESET				0xE0
#define BME280_REG_ID					0xD0
#define BME280_REG_CALIB_00_25_START	0x88

#define BME280_ID						0x60

#define BME280_REG_CTRL_MEAS_MODE_MASK		3
#define BME280_REG_CTRL_MEAS_MODE_SLEEP		0
#define BME280_REG_CTRL_MEAS_MODE_FORCED	1
#define BME280_REG_CTRL_MEAS_MODE_NORMAL	3

#define BME280_REG_RESET_VAL			0xB6

#pragma pack(push, 1)
typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
} BME280_CALIB_DATA;
#pragma pack(pop)

#ifdef __cplusplus

/// @brief	Implementation class of Bosch #BME280 Temperature, Pressure, Humidity sensor.
///
/// Key features
///
/// - Package : 2.5 mm x 2.5 mm x 0.93 mm metal lid LGA
/// - Digital interface : I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 MHz)
/// - Supply voltage : VDD main supply voltage range: 1.71 V to 3.6 V, VDDIO interface voltage range: 1.2 V to 3.6 V
/// - Current consumption : 1.8 μA @ 1 Hz humidity and temperature. 2.8 μA @ 1 Hz pressure and temperature.
/// 3.6 μA @ 1 Hz humidity, pressure and temperature. 0.1 μA in sleep mode
/// - Operating range : -40...+85 °C, 0...100 % rel. humidity, 300...1100 hPa
/// - Humidity sensor and pressure sensor can be independently enabled / disabled
/// - Register and performance compatible to Bosch Sensortec BMP280 digital pressure sensor
///
/// Key parameters for humidity sensor1
/// - Response time : 1 s
/// - Accuracy tolerance : ±3 % relative humidity
/// - Hysteresis : ±1% relative humidity
///
/// Key parameters for pressure sensor
///
/// - RMS Noise 0.2 Pa, equiv. to 1.7 cm
/// - Offset temperature coefficient ±1.5 Pa/K, equiv. to ±12.6 cm at 1 °C temperature change
class TphBme280 : public HumiSensor, public PressSensor, public TempSensor { //TphSensor {
public:
	TphBme280() : vCalibTFine(0), vbSpi(false) {}
	virtual ~TphBme280() {}

	/**
	 * @brief	Initialize sensor.
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
	bool Init(const HUMISENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	bool Init(const PRESSSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	bool Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

	/**
	 * @brief	Set current sensor state
	 *
	 * @param 	State
	 *				- SENSOR_STATE_SLEEP	// Sleep state low power
	 *				- SENSOR_STATE_IDLE		// Idle state powered on
	 *				- SENSOR_STATE_SAMPLING	// Sampling in progress
	 *
	 * @return	Actual state. In the case where the new state could
	 * 			not be set, it returns the actual state of the sensor.
	 */
	SENSOR_STATE State(SENSOR_STATE State);

	/**
	 * @brief Set operating mode
	 *
	 * @param OpMode : Operating mode
	 * 					- TPHSENSOR_OPMODE_SINGLE
	 * 					- TPHSENSOR_OPMODE_CONTINUOUS
	 * @param Freq : Sampling frequency in mHz for continuous mode
	 *
	 * @return true- if success
	 */
	virtual bool Mode(SENSOR_OPMODE OpMode, uint32_t Freq);

	/**
	 * @brief	Start sampling data
	 *
	 * @return	true - success
	 */
	virtual bool StartSampling();
	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * @return	None
	 */
	virtual void Disable();

	/**
	 * @brief	Reset device to it initial state
	 *
	 * @return	None
	 */
	virtual void Reset();

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
	void Read(HUMISENSOR_DATA &Data) { HumiSensor::Read(Data); }
	void Read(PRESSSENSOR_DATA &Data) { PressSensor::Read(Data); }
	void Read(TEMPSENSOR_DATA &Data) { TempSensor::Read(Data); }
/*
	float ReadTemperature() {
		TPHSENSOR_DATA tphdata;
		Read(tphdata);
		return (float)tphdata.Temperature / 100.0;
	}

	float ReadPressure() {
		TPHSENSOR_DATA tphdata;
		Read(tphdata);
		return (float)tphdata.Pressure;
	}

	float ReadHumidity() {
		TPHSENSOR_DATA tphdata;
		Read(tphdata);
		return (float)tphdata.Humidity / 100.0;
	}
*/

private:

	bool Init(uint32_t DevAddr, DeviceIntrf *pIntrf, Timer *pTimer);

	uint32_t CompenPress(int32_t RawPress);
	int32_t CompenTemp(int32_t RawTemp);
	uint32_t CompenHum(int32_t RawHum);
	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	bool UpdateData();

	int32_t vCalibTFine;	// For internal calibration use only
	BME280_CALIB_DATA vCalibData;
	uint8_t vCtrlReg;
	bool vbSpi;
	bool vbInitialized;
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __TPH_BME280_H__
