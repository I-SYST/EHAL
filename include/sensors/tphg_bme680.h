/**-------------------------------------------------------------------------
@file	tphg_bme680.h

@brief	Implementation of Bosch #BME680 low power gas, pressure, temperature & humidity sensor.

The Air Quality Index is undocumented.  It requires the library
Bosch Sensortec Environmental Cluster (BSEC) Software. Download from
https://www.bosch-sensortec.com/bst/products/all_products/bsec and put in
external folder as indicated on the folder tree.

The BSEC library must be initialized in the main application prior to initializing
this driver by calling the function

bsec_library_return_t res = bsec_init();

Specs:

The BME680 is a digital 4-in-1 sensor with gas, humidity, pressure and temperature
measurement based on proven sensing principles. The sensor module is housed in an
extremely compact metal-lid LGA package with a footprint of only 3.0 × 3.0 mm2 with
a maximum height of 1.00 mm (0.93 ± 0.07 mm). Its small dimensions and its low power
consumption enable the integration in battery-powered or frequency-coupled devices,
such as handsets or wearables.

Key features

- Package : 3.0 mm x 3.0 mm x 0.93 mm metal lid LGA
- Digital interface : I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 MHz)
- Supply voltage : VDD main supply voltage range: 1.71 V to 3.6 V. VDDIO interface voltage range: 1.2 V to 3.6 V
- Current consumption : 2.1 μA at 1 Hz humidity and temperature. 3.1 μA at 1 Hz pressure and temperature
3.7 μA at 1 Hz humidity, pressure and temperature. 0.09‒12 mA for p/h/T/gas depending on operation mode. 0.15 μA in sleep mode
- Operating range : -40‒+85 °C, 0‒100% r.H., 300‒1100 hPa
- Individual humidity, pressure and gas sensors can be independently enabled/disabled

Key parameters for gas sensor

- Response time (𝜏33−63%) : < 1 s (for new sensors)
- Power consumption : < 0.1 mA in ultra-low power mode
- Output data processing : direct indoor air quality (IAQ) index output

Key parameters for humidity sensor

- Response time (𝜏0−63%) : ~8 s
- Accuracy tolerance : ±3% r.H.
- Hysteresis : ±1.5% r.H.

Key parameters for pressure sensor

- RMS Noise 0.12 Pa, equiv. to 1.7 cm
- Offset temperature coefficient ±1.3 Pa/K, equiv. to ±10.9 cm at 1 °C temperature change

@author	Hoang Nguyen Hoan
@date	Oct. 15, 2017

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
#ifndef __TPHG_BME680_H__
#define __TPHG_BME680_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "tph_sensor.h"
#include "gas_sensor.h"

/** @addtogroup Sensors
  * @{
  */

// Device address depending on SDO wiring
#define BME680_I2C_DEV_ADDR0			0x76		// Device address when SDO to GND
#define BME680_I2C_DEV_ADDR1			0x77		// Device address when SDO to VCC

#define BME680_ID						0x61

#define BME680_REG_STATUS				0x73

#define BME680_REG_STATUS_SPI_MEM_PG		(1<<4)		// SPI page select
#define BME680_REG_STATUS_SPI_MEM_PG_BITPOS	(4)			// SPI page select shift position

#define BME680_REG_RESET				0xE0

#define BME680_REG_RESET_VAL				0xB6

#define BME680_REG_ID					0xD0

#define BME680_REG_ID_VAL					BME680_ID

#define BME680_REG_CONFIG				0x75

#define BME680_REG_CONFIG_SPI_EN			(1<<0)		// SPI enable
#define BME680_REG_CONFIG_FILTER_MASK		(7<<2)
#define BME680_REG_CONFIG_FILTER_BITPOS		(2)

#define BME680_REG_CTRL_MEAS			0x74

#define BME680_REG_CTRL_MEAS_MODE_MASK		(3<<0)
#define BME680_REG_CTRL_MEAS_MODE_SLEEP		(0<<0)
#define BME680_REG_CTRL_MEAS_MODE_FORCED	(1<<0)
#define BME680_REG_CTRL_MEAS_MODE_NORMAL	(1<<0)
#define BME680_REG_CTRL_MEAS_OSRS_P_MASK	(7<<2)
#define BME680_REG_CTRL_MEAS_OSRS_P_BITPOS	(2)
#define BME680_REG_CTRL_MEAS_OSRS_T_MASK	(7<<5)
#define BME680_REG_CTRL_MEAS_OSRS_T_BITPOS	(5)


#define BME680_REG_CTRL_HUM				0x72

#define BME680_REG_CTRL_HUMOSRS_H_MASK		(7<<0)
#define BME680_REG_CTRL_HUM_SPI_INT_EN		(1<<6)		// SPI Interrupt enable

#define BME680_REG_CTRL_GAS1			0x71

#define BME680_REG_CTRL_GAS1_NB_CONV_MASK	(0xF<<0)
#define BME680_REG_CTRL_GAS1_RUN_GAS		(1<<4)

#define BME680_REG_CTRL_GAS0			0x70

#define BME680_REG_CTRL_GAS0_HEAT_OFF		(1<<3)

#define BME680_REG_GAS_WAIT_X_START		0x64
#define BME680_REG_GAS_WAIT_X_END		0x6D

#define BME680_REG_RES_HEAT_X_START		0x5A
#define BME680_REG_RES_HEAT_X_END		0x63

#define BME680_REG_IDAC_HEAT_X_START	0x50
#define BME680_REG_IDAC_HEAT_X_END		0x59

#define BME680_REG_GAS_R_LSB			0x2B

#define BME680_REG_GAS_R_LSB_GAS_RANGE_R	(0xF<<0)
#define BME680_REG_GAS_R_LSB_HEAT_STAB_R	(1<<4)
#define BME680_REG_GAS_R_LSB_GAS_VALID_R	(1<<5)
#define BME680_REG_GAS_R_LSB_GAS_R_0_1		(3<<6)

#define BME680_REG_GAS_R_MSB			0x2A

#define BME680_REG_GAS_R_MSB_GAS_R_2_9		(0xFF<<0)

#define BME680_REG_HUM_LSB				0x26
#define BME680_REG_HUM_MSB				0x25

#define BME680_REG_TEMP_XLSB			0x24
#define BME680_REG_TEMP_LSB				0x23
#define BME680_REG_TEMP_MSB				0x22

#define BME680_REG_PRESS_XLSB			0x21
#define BME680_REG_PRESS_LSB			0x20
#define BME680_REG_PRESS_MSB			0x1F

#define BME680_REG_MEAS_STATUS_0			0x1D

#define BME680_REG_MEAS_STATUS_0_GAS_MEAS_IDX_0	(0xF<<0)
#define BME680_REG_MEAS_STATUS_0_MEASURING		(1<<5)
#define BME680_REG_MEAS_STATUS_0_GAS_MEASURING	(1<<6)
#define BME680_REG_MEAS_STATUS_0_NEW_DATA		(1<<7)
#define BME680_REG_MEAS_STATUS_0_BUSY			(BME680_REG_MEAS_STATUS_0_NEW_DATA | \
												 BME680_REG_MEAS_STATUS_0_MEASURING | \
												 BME680_REG_MEAS_STATUS_0_GAS_MEASURING)

#define BME680_REG_CALIB_00_23_START	0x8A
#define BME680_REG_CALIB_24_40_START	0xE1

#define BME680_REG_RES_HEAT_VAL			0x00

#define BME680_REG_RES_HEAT_RANGE		0x02

#define	BME680_REG_RES_HEAT_RANGE_MASK		0x30

#define BME680_REG_RANGE_SW_ERR			0x04

#define BME680_REG_RANGE_SW_ERR_MASK		0xF0

#define BME680_GAS_HEAT_PROFILE_MAX		10	// Max number of heating temperature set

#define BME680_REG_SPI_ADDR_MASK		0x7F
#define BME680_REG_I2C_ADDR_MASK		0xFF


#pragma pack(push, 1)
typedef struct {
	int16_t par_T2;
	int8_t par_T3;
	uint8_t pad0;
	uint16_t par_P1;
	int16_t par_P2;
	int8_t par_P3;
	uint8_t pad1;
	int16_t par_P4;
	int16_t par_P5;
	int8_t par_P7;
	int8_t par_P6;
	uint16_t pad2;
	int16_t par_P8;
	int16_t par_P9;
	uint8_t par_P10;
	uint16_t par_H2;
	uint16_t par_H1;
	int8_t par_H3;
	int8_t par_H4;
	int8_t par_H5;
	uint8_t par_H6;
	int8_t par_H7;
	uint16_t par_T1;
	int16_t par_GH2;
	int8_t par_GH1;
	int8_t par_GH3;
	uint8_t res_heat_range;
	int8_t res_heat_val;
	int8_t range_sw_err;
} BME680_CALIB_DATA;
#pragma pack(pop)

#ifdef __cplusplus

/// @brief	Implementation of Bosch #BME680 low power gas, pressure, temperature & humidity sensor.
///
/// Key features
///
/// - Package : 3.0 mm x 3.0 mm x 0.93 mm metal lid LGA
/// - Digital interface : I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 MHz)
/// - Supply voltage : VDD main supply voltage range: 1.71 V to 3.6 V. VDDIO interface voltage range: 1.2 V to 3.6 V
/// - Current consumption : 2.1 μA at 1 Hz humidity and temperature. 3.1 μA at 1 Hz pressure and temperature
/// 3.7 μA at 1 Hz humidity, pressure and temperature. 0.09‒12 mA for p/h/T/gas depending on operation mode. 0.15 μA in sleep mode
/// - Operating range : -40‒+85 °C, 0‒100% r.H., 300‒1100 hPa
/// - Individual humidity, pressure and gas sensors can be independently enabled/disabled
///
/// Key parameters for gas sensor
///
/// - Response time (𝜏33−63%) : < 1 s (for new sensors)
/// - Power consumption : < 0.1 mA in ultra-low power mode
/// - Output data processing : direct indoor air quality (IAQ) index output
///
/// Key parameters for humidity sensor
///
/// - Response time (𝜏0−63%) : ~8 s
/// - Accuracy tolerance : ±3% r.H.
/// - Hysteresis : ±1.5% r.H.
///
/// Key parameters for pressure sensor
///
/// - RMS Noise 0.12 Pa, equiv. to 1.7 cm
/// - Offset temperature coefficient ±1.3 Pa/K, equiv. to ±10.9 cm at 1 °C temperature change
class TphgBme680 : public TphSensor, public GasSensor {
public:
	TphgBme680();
	virtual ~TphgBme680() {}

	/**
	 * @brief	Main initialization TPH sensor.
	 *
	 * @note	This initialization must be called first.
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
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	bool Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer);

	/**
	 * @brief	Secondary initialization gas sensor.
	 *
	 * @note	Must call TPH initialization first before calling this function.
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
	bool Init(const GASSENSOR_CFG &CfgData, DeviceIntrf *pIntrf = NULL, Timer *pTimer = NULL);

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
	virtual SENSOR_STATE State(SENSOR_STATE State);

	/**
	 * @brief	Set gas heating profile
	 *
	 * @param	Count : Number of heating temperature settings
	 * 			pProfile : Pointer to array of temperature/duration settings
	 *
	 * @return	true - success
	 */
	virtual bool SetHeatingProfile(int Count, const GASSENSOR_HEAT *pProfile);

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
	 * @brief	Read gas sensor data
	 * 			Read gas data from device if available. If not
	 * 			return previous data
	 *
	 * @param 	GasData : Gas sensor data
	 *
	 * @return	true - new data
	 * 			false - old data
	 */
	bool Read(GASSENSOR_DATA &GasData);

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
	bool Read(TPHSENSOR_DATA &TphData);

	/**
	 * @brief	Read temperature
	 *
	 * @return	Temperature in degree C
	 */
	float ReadTemperature() {
		TPHSENSOR_DATA tphdata;
		Read(tphdata);
		return (float)tphdata.Temperature / 100.0;
	}

	/**
	 * @brief	Read barometric pressure
	 *
	 * @return	Barometric pressure in Pascal
	 */
	float ReadPressure() {
		TPHSENSOR_DATA tphdata;
		Read(tphdata);
		return (float)tphdata.Pressure / 100.0;
	}

	/**
	 * @brief	Read relative humidity
	 *
	 * @return	Relative humidity in %
	 */
	float ReadHumidity() {
		TPHSENSOR_DATA tphdata;
		Read(tphdata);
		return (float)tphdata.Humidity / 100.0;
	}

	uint32_t vGasResInt;

private:

	BME680_CALIB_DATA vCalibData;

	uint32_t CalcPressure(int32_t RawPress);
	int32_t CalcTemperature(int32_t RawTemp);
	uint32_t CalcHumidity(int32_t RawHum);
	//float fCalcGas(uint16_t RawGas, uint8_t Range);
	uint32_t CalcGas(uint16_t RawGas, uint8_t Range);
	uint8_t CalcHeaterResistance(uint16_t Temp);
	bool UpdateData();
	void SelectRegPage(uint8_t Reg);
	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

	int32_t vCalibTFine;	// For internal calibration use only
	uint8_t vCtrlReg;
	uint8_t vCtrlGas1Reg;
	bool vbSpi;				// Set to true if SPI interfacing
	int vRegPage;			// Current register page, SPI interface only
	bool vbMeasGas;			// Do gas measurement
	bool vbGasData;
	int vNbHeatPoint;		// Number of heating points
	GASSENSOR_HEAT vHeatPoints[BME680_GAS_HEAT_PROFILE_MAX];
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __TPHG_BME680_H__
