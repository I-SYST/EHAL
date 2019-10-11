/**-------------------------------------------------------------------------
@file	mag_ak09916.cpp

@brief	Implementation of Asahi Kasei AK09916 mag sensor


@author	Hoang Nguyen Hoan
@date	Sep. 8, 2019

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

#include "sensors/mag_ak09916.h"

bool MagAk09916::Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t regaddr = AK09916_WIA1_REG;
	uint16_t d;

	Read(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, (uint8_t*)&d, 2);

	if (d != AK09916_COMPANY_DEVICE_ID)
	{
		return false;
	}

	regaddr = AK09916_CTRL3_REG;
	d = AK09916_CTRL3_SRST;

	Write(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, (uint8_t*)&d, 1);

	SamplingFrequency(Cfg.Freq);

	Range(AK09916_ADC_RANGE);

	vSensitivity[0] = AK09916_FLUX_DENSITY / AK09916_ADC_RANGE;
	vSensitivity[1] = vSensitivity[0];
	vSensitivity[2] = vSensitivity[0];

	vData.Sensitivity[0] = vSensitivity[0];
	vData.Sensitivity[1] = vSensitivity[1];
	vData.Sensitivity[2] = vSensitivity[2];

	ClearCalibration();

	return true;
}

uint32_t MagAk09916::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = AK09916_CTRL2_REG;
	uint8_t d = 0;

	Write(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, &d, 1);

	if (Freq < 20000)
	{
		d |= AK09916_CTRL2_MODE_CONTINUOUS_10HZ;
		Freq = 10000;
	}
	else if (Freq < 50000)
	{
		d |= AK09916_CTRL2_MODE_CONTINUOUS_20HZ;
		Freq = 20000;
	}
	else if (Freq < 100000)
	{
		d |= AK09916_CTRL2_MODE_CONTINUOUS_50HZ;
		Freq = 50000;
	}
	else
	{
		d |= AK09916_CTRL2_MODE_CONTINUOUS_100HZ;
		Freq = 100000;
	}
	d = 4;
	printf("wd=%d\r\n", d);
	Write(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, &d, 1);

	d = 0;
	Read(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, &d, 1);
printf("d=%d\r\n", d);
	return MagSensor::SamplingFrequency(Freq);
}

/**
 * @brief	Power off the device completely.
 *
 * If supported, this will put the device in complete power down.
 * Full re-intialization is required to re-enable the device.
 */
void MagAk09916::PowerOff()
{
	uint8_t regaddr = AK09916_CTRL2_REG;
	uint8_t d = 0;

	Write(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, &d, 1);
}

bool MagAk09916::Enable()
{
	SamplingFrequency(Sensor::vSampFreq);

	return true;
}

void MagAk09916::Disable()
{
	uint8_t regaddr = AK09916_CTRL2_REG;
	uint8_t d = 0;

	Write(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, &d, 1);
}

void MagAk09916::Reset()
{
	uint8_t regaddr = AK09916_CTRL3_REG;
	uint8_t d = AK09916_CTRL3_SRST;

	Write(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, (uint8_t*)&d, 1);
}

bool MagAk09916::UpdateData()
{
	uint8_t regaddr = AK09916_ST1_REG;
	uint8_t d;

	Read(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, &d, 1);

	if (d & AK09916_ST1_DRDY)
	{
		regaddr = AK09916_HXL_REG;
		Read(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, (uint8_t*)vData.Val, 6);

		if (vpTimer)
		{
			vData.Timestamp = vpTimer->nSecond() * 1000ULL;
		}

		regaddr = AK09916_ST2_REG;
		Read(AK09916_I2C_7BITS_DEVADDR, &regaddr, 1, &d, 1);

		return true;
	}

	return false;
}
