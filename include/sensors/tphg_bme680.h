/*--------------------------------------------------------------------------
File   : tphg_bme680.h

Author : Hoang Nguyen Hoan          			Oct. 15, 2017

Desc   : BME680 environment sensor implementation
			- Temperature, Pressure, Humidity, Gas

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

// Device address depending on SDO wiring
#define BME680_I2C_DEV_ADDR0			0x76		// SDO to GND
#define BME680_I2C_DEV_ADDR1			0x77		// SDO to VCC

#define BME680_ID						0x61

#define BME680_REG_STATUS				0x73

#define BME680_REG_STATUS_SPI_MEM_PG		(1<<4)		// SPI page select

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
#define BME680_REG_CTRL_MEAS_OSRS_P_MASK	(7<<2)
#define BME680_REG_CTRL_MEAS_OSRS_P_BITPOS	(2)
#define BME680_REG_CTRL_MEAS_OSRS_T_MASK	(7<<5)
#define BME680_REG_CTRL_MEAS_OSRS_T_BITPOS	(5)


#define BME680_REG_CTRL_HUM				0x72

#define BME680_REG_CTRL_HUMOSRS_H_MASK		(7<<0)
#define BME680_REG_CTRL_HUM_SPI_INT_EN		(1<<6)		// SPI Interrupt enable

#define BME680_REG_CTRL_GAS1			0x71

#define BME680_REG_CTRL_GAS1_NB_CONV		(0xF<<0)
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

#define BME680_REG_CALIB_00_23_START	0x8A
#define BME680_REG_CALIB_24_40_START	0xE1

#define BME680_REG_RES_HEAT_VAL			0x00

#define BME680_REG_RES_HEAT_RANGE		0x02

#define	BME680_REG_RES_HEAT_RANGE_MASK		0x30

#define BME680_REG_RANGE_SW_ERR			0x04

#define BME680_REG_RANGE_SW_ERR_MASK		0xF0

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

class TphgBme680 : public TPHSensor, public GasSensor {
public:
	TphgBme680() : vRegWrMask(0xFF), vMeasGas(false) {}
	virtual ~TphgBme680() {}
	virtual bool Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer);
	virtual bool Init(const GASSENSOR_CFG &CfgData, DeviceIntrf *pIntrf = NULL, Timer *pTimer = NULL);
//	virtual bool Init(const void *pCfgData, DeviceIntrf *pIntrf, Timer *pTimer);

	/**
	 * @brief Set operating mode
	 *
	 * @param OpMode : Operating mode
	 * 					- TPHSENSOR_OPMODE_SLEEP
	 * 					- TPHSENSOR_OPMODE_SINGLE
	 * 					- TPHSENSOR_OPMODE_CONTINUOUS
	 * @param Freq : Sampling frequency in Hz for continuous mode
	 *
	 * @return true- if success
	 */
	virtual bool SetMode(SENSOR_OPMODE OpMode, uint32_t Freq);

	/**
	 * @brief	Start sampling data
	 *
	 * @return	true - success
	 */
	virtual bool StartSampling();
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	bool ReadTPH(TPHSENSOR_DATA &PthData);
	float ReadTemperature() {
		TPHSENSOR_DATA tphdata;
		ReadTPH(tphdata);
		return (float)tphdata.Temperature / 100.0;
	}

	float ReadPressure() {
		TPHSENSOR_DATA tphdata;
		ReadTPH(tphdata);
		return (float)tphdata.Pressure / 100.0;
	}

	float ReadHumidity() {
		TPHSENSOR_DATA tphdata;
		ReadTPH(tphdata);
		return (float)tphdata.Humidity / 100.0;
	}

	bool ReadGas(GASSENSOR_DATA &TphData);

private:

	BME680_CALIB_DATA vCalibData;

	uint32_t CalcPressure(int32_t RawPress);
	int32_t CalcTemperature(int32_t RawTemp);
	uint32_t CalcHumidity(int32_t RawHum);
	uint32_t CalcGas(uint16_t RawGas, uint8_t Range);
	uint8_t CalcHeaterResistance(uint16_t Temp);

	int32_t vCurTemp;
	uint32_t vCurBarPres;
	uint32_t vCurRelHum;
	uint32_t vCurGas;

	int32_t vCalibTFine;	// For internal calibration use only
	uint8_t vCtrlReg;
	uint8_t vRegWrMask;

	bool vMeasGas;			// Do gas measurement
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __TPHG_BME680_H__
