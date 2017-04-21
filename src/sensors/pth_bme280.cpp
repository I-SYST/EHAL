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
#include "serialintrf.h"
#include "iopincfg.h"
#include "sensors/pth_bme280.h"

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

static BME280_CALIB_DATA s_Bme280CalibData;

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t BME280::CompenTemp(int32_t adc_T)
{
	int32_t var1, var2, t;

	var1 = ((((adc_T >> 3) - ((int32_t)s_Bme280CalibData.dig_T1 << 1))) * ((int32_t)s_Bme280CalibData.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)s_Bme280CalibData.dig_T1)) * ((adc_T >> 4) -
		    ((int32_t)s_Bme280CalibData.dig_T1))) >> 12) * ((int32_t)s_Bme280CalibData.dig_T3)) >> 14;
	vCalibTFine = var1 + var2;
	t = (vCalibTFine * 5 + 128) >> 8;

	return t;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280::CompenPress(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)vCalibTFine) - 128000;
	var2 = var1 * var1 * (int64_t)s_Bme280CalibData.dig_P6;
	var2 = var2 + ((var1*(int64_t)s_Bme280CalibData.dig_P5) << 17);
	var2 = var2 + (((int64_t)s_Bme280CalibData.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)s_Bme280CalibData.dig_P3) >> 8) + ((var1 * (int64_t)s_Bme280CalibData.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_Bme280CalibData.dig_P1) >> 33;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - adc_P;
	p = (((p<<31) - var2) * 3125) / var1;
	var1 = (((int64_t)s_Bme280CalibData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)s_Bme280CalibData.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)s_Bme280CalibData.dig_P7) << 4);

	return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t BME280::CompenHum(int32_t adc_H)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (vCalibTFine - 76800);
	v_x1_u32r = (((( (adc_H << 14) - (((int32_t)s_Bme280CalibData.dig_H4) << 20) -
				 (((int32_t)s_Bme280CalibData.dig_H5) * v_x1_u32r)) +
				 16384) >> 15) * (((((((v_x1_u32r * ((int32_t)s_Bme280CalibData.dig_H6)) >> 10) * (((v_x1_u32r *
				 ((int32_t)s_Bme280CalibData.dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
				((int32_t)s_Bme280CalibData.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)s_Bme280CalibData.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

	return (uint32_t)(v_x1_u32r>>12);
}

bool BME280::Init(void *pCfgData, DeviceIntrf *pIntrf)
{
	uint8_t addr = BME280_REG_ID;
	uint8_t d;
	bool found = false;

	SetInterface(pIntrf);
	SetDeviceAddess(BME280_DEV_ADDR0);

	//Reset();

	Read((uint8_t*)&addr, 1, &d, 1);

	if (d == BME280_ID)
	{
		found = true;
	}
	else
	{
		SetDeviceAddess(BME280_DEV_ADDR1);
		Read((uint8_t*)&addr, 1, &d, 1);
		if (d == BME280_ID)
		{
			found = true;
		}
	}

	if (found == true)
	{
		usDelay(1000);

		addr = BME280_REG_CALIB_00_25_START;
		Read(&addr, 1, (uint8_t*)&s_Bme280CalibData, 26);

		uint8_t *p =  (uint8_t*)&s_Bme280CalibData;

		p[24] = p[25];

		uint8_t cd[8];

		addr = BME280_REG_CALIB_26_41_START;
		Read(&addr, 1, cd, 8);

		p[25] = cd[0];
		p[26] = cd[1];
		p[27] = cd[2];

		s_Bme280CalibData.dig_H4 = ((int16_t)cd[3] << 4) | (cd[4] & 0xF);
		s_Bme280CalibData.dig_H5 = ((int16_t)cd[5] << 4) | (cd[4] >> 4);
		s_Bme280CalibData.dig_H6 = cd[6];

		addr = BME280_REG_CTRL_HUM;
		d = 1;	// oversampling x 1
		Write(&addr, 1, &d, 1);

		addr = BME280_REG_CTRL_MEAS;
		d = (1 << 5) | (1 << 2) | 0;	// oversampling x1, forced
		Write(&addr, 1, &d, 1);
	}

	return found;
}

bool BME280::Enable()
{

}

void BME280::Disable()
{

}

void BME280::Reset()
{
	uint8_t addr = BME280_REG_RESET;
	uint8_t d = 0xB6;

	Write(&addr, 1, &d, 1);
}

bool BME280::ReadPTH(PTHSENSOR_DATA &PthData)
{
	uint8_t addr = BME280_REG_STATUS;
	uint8_t status = 0;
	bool retval = false;
	int timeout = 10000;

	do {
		Read(&addr, 1, &status, 1);
	} while ((status ) != 0 && timeout-- > 0);

	if ((status & 9) == 0)
	{
		uint8_t d[8];
		addr = BME280_REG_PRESS_MSB;

		if (Read(&addr, 1, d, 8) == 8)
		{
			int32_t p = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | ((int32_t)d[2] >> 4);
			int32_t t = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | ((int32_t)d[5] >> 4);
			int32_t h = ((int32_t)d[6] << 8) | d[7];

			vCurTemp = CompenTemp(t);
			vCurBarPres = CompenPress(p);
			vCurRelHum = CompenHum(h);

			retval = true;
		}
	}

	PthData.Humidity = (int16_t)(((float)vCurRelHum / 1024.0) * 100.0);
	PthData.Pressure = (int16_t)(((float)vCurBarPres / 256.0) * 100.0);
	PthData.Temperature = vCurTemp;

	return retval;
}
