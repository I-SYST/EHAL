/*--------------------------------------------------------------------------
File   : pth_bme280.cpp

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : BME280 environment sensor implementation
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
#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "idelay.h"
#include "device_intrf.h"
#include "iopincfg.h"
#include "sensors/tph_bme280.h"

//static BME280_CALIB_DATA s_Bme280CalibData;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t TphBme280::CompenTemp(int32_t adc_T)
{
	int32_t var1, var2, t;

	var1 = ((((adc_T >> 3) - ((int32_t)vCalibData.dig_T1 << 1))) * ((int32_t)vCalibData.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)vCalibData.dig_T1)) * ((adc_T >> 4) -
		    ((int32_t)vCalibData.dig_T1))) >> 12) * ((int32_t)vCalibData.dig_T3)) >> 14;
	vCalibTFine = var1 + var2;
	t = (vCalibTFine * 5 + 128) >> 8;

	return t;
}

// @return value in Pa
uint32_t TphBme280::CompenPress(int32_t adc_P)
{
	int64_t var1, var2;
	uint32_t p;

	var1 = (vCalibTFine >> 1) - 64000;
	var2 = (var1 * var1 * (int64_t)vCalibData.dig_P6) >> 15;
	var2 = (var2 + var1 * (int64_t)vCalibData.dig_P5) << 1;
	var2 = (var2 >> 2) + ((int64_t)vCalibData.dig_P4 << 16);
	var1 = ((((int64_t)vCalibData.dig_P3 * var1 * var1) >> 19) +
			(int64_t)vCalibData.dig_P2 * var1) >> 19;
	var1 = (int32_t)vCalibData.dig_P1 + ((var1 * (uint32_t)vCalibData.dig_P1) >> 15);

	if (var1 <= 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - adc_P;
	p = (p - (var2 >> 12LL)) * 6250 / var1;
	var1 = (((int64_t)vCalibData.dig_P9 * (int64_t)p * (int64_t)p) >> 31LL);
	var2 = (((int64_t)p * (int64_t)vCalibData.dig_P8) >> 15LL);
	p = p + ((var1 + var2 + ((int32_t)vCalibData.dig_P7)) >> 4);

	return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t TphBme280::CompenHum(int32_t adc_H)
{
	int32_t var1;

	var1 = (vCalibTFine - 76800);
	var1 = (((( (adc_H << 14) - ((int32_t)vCalibData.dig_H4 << 20) -
				 ((int32_t)vCalibData.dig_H5 * var1)) + 16384) >> 15) *
			(((((((var1 * (int32_t)vCalibData.dig_H6) >> 10) * (((var1 *
				 (int32_t)vCalibData.dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
				(int32_t)vCalibData.dig_H2 + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * (int32_t)vCalibData.dig_H1) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (uint32_t)((var1 * 100LL)>> 22LL);

}

bool TphBme280::Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr = BME280_REG_ID;
	uint8_t d;
	bool found = false;

	vCalibTFine = 0;

	if (pIntrf != NULL)
	{
		Interface(pIntrf);
		DeviceAddess(CfgData.DevAddr);
	}

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	if (CfgData.DevAddr == BME280_I2C_DEV_ADDR0 || CfgData.DevAddr == BME280_I2C_DEV_ADDR1)
	{
		// I2C mode
		//vRegWrMask = 0xFF;
		vbSpi = false;
	}
	else
	{
		//vRegWrMask = 0x7F;
		vbSpi = true;
	}

	Read((uint8_t*)&regaddr, 1, &d, 1);

	if (d == BME280_ID)
	{
		found  = true;

		DeviceID(d);
		Valid(true);

		Reset();

		usDelay(30000);


		// Load calibration data

		regaddr = BME280_REG_CALIB_00_25_START;
		Read(&regaddr, 1, (uint8_t*)&vCalibData, 26);

		uint8_t *p =  (uint8_t*)&vCalibData;

		p[24] = p[25];

		uint8_t cd[8];

		regaddr = BME280_REG_CALIB_26_41_START;
		Read(&regaddr, 1, cd, 8);

		p[25] = cd[0];
		p[26] = cd[1];
		p[27] = cd[2];

		vCalibData.dig_H4 = ((int16_t)cd[3] << 4) | (cd[4] & 0xF);
		vCalibData.dig_H5 = ((int16_t)cd[5] << 4) | (cd[4] >> 4);
		vCalibData.dig_H6 = cd[6];

		// Setup oversampling & filters.
		d = 0;
		if (CfgData.HumOvrs > 0)
		{
			d = (CfgData.HumOvrs & 7);
		}

		regaddr = BME280_REG_CTRL_HUM;
		Write((uint8_t*)&regaddr, 1, &d, 1);

		regaddr = BME280_REG_CONFIG;
		Read((uint8_t*)&regaddr, 1, &d, 1);

		d |= (CfgData.FilterCoeff << BME280_REG_CONFIG_FILTER_BITPOS) & BME280_REG_CONFIG_FILTER_MASK;
		Write((uint8_t*)&regaddr, 1, &d, 1);

		regaddr = BME280_REG_CTRL_MEAS;
		Read(&regaddr, 1, &vCtrlReg, 1);

		vCtrlReg |= (CfgData.PresOvrs << BME280_REG_CTRL_MEAS_OSRS_P_BITPOS) & BME280_REG_CTRL_MEAS_OSRS_P_MASK;
		vCtrlReg |= (CfgData.TempOvrs << BME280_REG_CTRL_MEAS_OSRS_T_BITPOS) & BME280_REG_CTRL_MEAS_OSRS_T_MASK;
		Write((uint8_t*)&regaddr, 1, &vCtrlReg, 1);

		Mode(CfgData.OpMode, CfgData.Freq);

		//State(SENSOR_STATE_SLEEP);

		usDelay(10000);
	}

	return found;
}

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
SENSOR_STATE TphBme280::State(SENSOR_STATE State) {

	if (State == SENSOR_STATE_SLEEP)
	{
		uint8_t regaddr = BME280_REG_CTRL_MEAS;
		vCtrlReg &= ~BME280_REG_CTRL_MEAS_MODE_MASK;
		Write(&regaddr, 1, &vCtrlReg, 1);
	}

	return Sensor::State(State);
}

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
bool TphBme280::Mode(SENSOR_OPMODE OpMode, uint32_t Freq)
{
	uint8_t regaddr;
	uint8_t d = 0;

	vOpMode = OpMode;

	Sensor::SamplingFrequency(Freq);

	// read current ctrl_meas register
	regaddr = BME280_REG_CTRL_MEAS;
	Read(&regaddr, 1, &vCtrlReg, 1);

	vCtrlReg &= ~BME280_REG_CTRL_MEAS_MODE_MASK;

	if (vOpMode == SENSOR_OPMODE_CONTINUOUS)
	{
		uint32_t period = 1000 / Freq;
		if (period >= 1000)
		{
			d = (5 << 5) | (4 << 2);
		}
		else if (period >= 500)
		{
			d = (4 << 5) | (4 << 2);
		}
		else if (period >= 250)
		{
			d = (3 << 5) | (3 << 2);
		}
		else if (period >= 125)
		{
			d = (2 << 5) | (3 << 2);
		}
		else if (period >= 62)
		{
			d = (1 << 5) | (3 << 2);
		}
		else if (period >= 20)
		{
			d = (7 << 5) | (2 << 2);
		}
		else if (period >= 10)
		{
			d = (6 << 5) | (1 << 2);
		}
		else
		{
			d = 0;
		}

		regaddr = BME280_REG_CONFIG;
		Write(&regaddr, 1, &d, 1);
	}

	//StartSampling();

	return true;
}

/**
 * @brief	Start sampling data
 *
 * @return	true - success
 * 			false - Busy
 */
bool TphBme280::StartSampling()
{
	uint8_t regaddr = BME280_REG_STATUS;
	uint8_t d;

	d = Read8(&regaddr, 1);

	if (d == 0xff)
	{
		State(SENSOR_STATE_IDLE);
		d = Read8(&regaddr, 1);
	}

	if (d & (BME280_REG_STATUS_MEASURING | BME280_REG_STATUS_IM_UPDATE))
		return false;

	regaddr = BME280_REG_CTRL_MEAS;
	d = vCtrlReg | BME280_REG_CTRL_MEAS_MODE_NORMAL;

	Write(&regaddr, 1, &d, 1);

	vbSampling = true;

	if (vpTimer)
	{
		vSampleTime = vpTimer->uSecond();
	}
	else
	{
		vSampleTime += vSampPeriod;
	}

	return true;
}

bool TphBme280::Enable()
{
	//SetMode(SENSOR_OPMODE_CONTINUOUS, vSampFreq);

	return true;
}

void TphBme280::Disable()
{
	State(SENSOR_STATE_SLEEP);
}

void TphBme280::Reset()
{
	uint8_t addr = BME280_REG_RESET;
	uint8_t d = BME280_REG_RESET_VAL;

	Write(&addr, 1, &d, 1);
}

bool TphBme280::UpdateData()
{
	uint8_t addr = BME280_REG_STATUS;
	uint8_t status = 0;
	bool retval = false;
	int timeout = 20;

	Read(&addr, 1, &status, 1);

	if ((status & (BME280_REG_STATUS_MEASURING | BME280_REG_STATUS_IM_UPDATE))== 0)
	{
		uint8_t d[8];
		addr = BME280_REG_PRESS_MSB;

		if (Read(&addr, 1, d, 8) == 8)
		{
			int32_t p = (((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | ((uint32_t)d[2] >> 4));
			int32_t t = (((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | ((uint32_t)d[5] >> 4));
			int32_t h = (((uint32_t)d[6] << 8) | d[7]);

			vTphData.Temperature = CompenTemp(t);
			vTphData.Pressure = CompenPress(p);
			vTphData.Humidity = CompenHum(h);
			vTphData.Timestamp = vSampleTime;

			vSampleCnt++;

			vbSampling = false;

			retval = true;
		}
	}

	return retval;
}

bool TphBme280::Read(TPHSENSOR_DATA &TphData)
{
	bool retval = UpdateData();

	memcpy(&TphData, &vTphData, sizeof(TPHSENSOR_DATA));

	return retval;
}

int TphBme280::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vbSpi == true)
	{
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int TphBme280::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vbSpi == true)
	{
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}
