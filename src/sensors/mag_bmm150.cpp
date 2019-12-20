/**-------------------------------------------------------------------------
@file	mag_bmm150.cpp

@brief	Implementation of BOSCH BMM150 mag sensor

@author	Hoang Nguyen Hoan
@date	Aug. 23, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include <stdint.h>

#include "idelay.h"
#include "sensors/mag_bmm150.h"


/**
 * @brief	Initialize mag sensor.
 *
 * @param 	Cfg		: Configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool MagBmm150::Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	MagSensor::Type(SENSOR_TYPE_MAG);

	//vData.Range = Range(((1<<15) - 1) >> 1);
	//vData.Scale = 2500;

	uint8_t regaddr = BMM150_CTRL1_REG;
	uint8_t d = BMM150_CTRL1_POWER_ON | BMM150_CTRL1_SOFT_RESET | BMM150_CTRL1_SOFT_RESET2;

	// use this way to allow hook up on the secondary interface of a combo device
	MagBmm150::Write(&regaddr, 1, &d, 1);

	msDelay(10);

	regaddr = BMM150_CHIP_ID_REG;
	MagBmm150::Read(&regaddr, 1, &d, 1);

	if (d != BMM150_CHIP_ID)
	{
		return false;
	}

	// There is no setting to change precision
	// fix it to lowest value.
	// X, Y : 13 bits, Z : 15 bits, RHALL : 14 bits
	vPrecision = MAGSENSOR_PRECISION_LOW;
	vSensitivity[0] = BMI150_FLUX_DENSITY_XY / BMI150_ADC_RANGE_XY;
	vSensitivity[1] = vSensitivity[0];
	vSensitivity[2] = BMI150_FLUX_DENSITY_Z / BMI150_ADC_RANGE_Z;

	vRange = BMI150_ADC_RANGE_XY;
	vData.Sensitivity[0] = vSensitivity[0];
	vData.Sensitivity[1] = vSensitivity[1];
	vData.Sensitivity[2] = vSensitivity[2];

	ClearCalibration();

	SamplingFrequency(Cfg.Freq);

	Enable();

	return true;
}

uint32_t MagBmm150::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMM150_CTRL2_REG;
	uint8_t d;
	uint32_t f = 0;

	MagBmm150::Read(&regaddr, 1, &d, 1);

	d &= ~BMM150_CTRL2_ODR_MASK;

	if (Freq < 6000)
	{
		f = 2000;
		d |= BMM150_CTRL2_ODR_2HZ;
	}
	else if (Freq < 8000)
	{
		f = 6000;
		d |= BMM150_CTRL2_ODR_6HZ;
	}
	else if (Freq < 10000)
	{
		f = 8000;
		d |= BMM150_CTRL2_ODR_8HZ;
	}
	else if (Freq < 15000)
	{
		f = 10000;
		d |= BMM150_CTRL2_ODR_10HZ;
	}
	else if (Freq < 20000)
	{
		f = 15000;
		d |= BMM150_CTRL2_ODR_15HZ;
	}
	else if (Freq < 25000)
	{
		f = 20000;
		d |= BMM150_CTRL2_ODR_20HZ;
	}
	else if (Freq < 30000)
	{
		f = 25000;
		d |= BMM150_CTRL2_ODR_25HZ;
	}
	else
	{
		f = 30000;
		d |= BMM150_CTRL2_ODR_30HZ;
	}

	MagBmm150::Write(&regaddr, 1, &d, 1);

	return Sensor::SamplingFrequency(f);
}

bool MagBmm150::Enable()
{
	uint8_t regaddr = BMM150_CTRL2_REG;
	uint8_t d;

	MagBmm150::Read(&regaddr, 1, &d, 1);
	d &= ~(BMM150_CTRL2_OPMODE_MASK | BMM150_CTRL2_SELFTEST);
	d |= BMM150_CTRL2_OPMODE_NORMAL;
	MagBmm150::Write(&regaddr, 1, &d, 1);

	msDelay(10);

	regaddr = BMM150_CTRL3_REG;
	MagBmm150::Read(&regaddr, 1, &d, 1);

	d &= ~(BMM150_CTRL3_CHAN_X_DIS | BMM150_CTRL3_CHAN_Y_DIS | BMM150_CTRL3_CHAN_Y_DIS);
	MagBmm150::Write(&regaddr, 1, &d, 1);

	return true;
}

void MagBmm150::Disable()
{
	uint8_t regaddr = BMM150_CTRL3_REG;
	uint8_t d;

	MagBmm150::Read(&regaddr, 1, &d, 1);

	d |= (BMM150_CTRL3_CHAN_X_DIS | BMM150_CTRL3_CHAN_Y_DIS | BMM150_CTRL3_CHAN_Y_DIS);
	MagBmm150::Write(&regaddr, 1, &d, 1);

	regaddr = BMM150_CTRL2_REG;
	MagBmm150::Read(&regaddr, 1, &d, 1);
	d &= ~BMM150_CTRL2_OPMODE_MASK;
	d |= BMM150_CTRL2_OPMODE_SLEEP;
	MagBmm150::Write(&regaddr, 1, &d, 1);
}

void MagBmm150::Reset()
{
	uint8_t regaddr = BMM150_CTRL1_REG;
	uint8_t d;

	MagBmm150::Read(&regaddr, 1, &d, 1);

	d |= BMM150_CTRL1_SOFT_RESET;
	MagBmm150::Write(&regaddr, 1, &d, 1);

	msDelay(1);

	//d |= BMM150_CTRL1_POWER_ON;
	//MagBmm150::Write(&regaddr, 1, &d, 1);
}

bool MagBmm150::UpdateData()
{
	uint8_t regaddr = BMM150_INT_STATUS_REG;
	uint8_t d = 0;
	bool res = false;

	MagBmm150::Read(&regaddr, 1, &d, 1);
	if (d)
	{
		int16_t buff[4];
		regaddr = BMM150_DATA_X_LSB_REG;

		if (vpTimer)
		{
			vData.Timestamp = vpTimer->uSecond();
		}
		MagBmm150::Read(&regaddr, 1, (uint8_t*)buff, 8);

		vData.X = buff[0] >> 3;
		vData.Y = buff[1] >> 3;
		vData.Z = buff[2] >> 1;

		res = true;
	}

	return res;
}
