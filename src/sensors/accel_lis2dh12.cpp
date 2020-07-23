/**-------------------------------------------------------------------------
@file	accel_lis22dh12.cpp

@brief	Implementation of ST LIS2DH12 accel. sensor

Note : More details about the registers programming are described in
       application AN5005. Those important details are not in the datasheet

@author	Hoang Nguyen Hoan
@date	Jan. 17, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#include "sensors/accel_lis2dh12.h"
#include "idelay.h"

bool AccelLis2dh12::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	// Read chip id
	uint8_t regaddr = LIS2DH12_WHO_AM_I_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d != LIS2DH12_WHO_AM_I_ID)
	{
		return false;
	}

	DeviceID(d);
	Valid(true);

	return true;
}

/**
 * @brief	Initialize accelerometer sensor.
 *
 * NOTE: This sensor must be the first to be initialized.
 *
 * @param 	Cfg		: Accelerometer configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool AccelLis2dh12::Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbValid == false)
	{
		if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		{
			return false;
		}
	}

	if (Cfg.IntHandler)
	{
		vIntHandler = Cfg.IntHandler;
	}

	// High res 12bits default
	uint8_t regaddr = LIS2DH12_CTRL_REG4;
	uint8_t d = Read8(&regaddr, 1) | LIS2DH12_CTRL_REG4_HR;
	Write8(&regaddr, 1, d);

	AccelSensor::Range(2047);
	AccelSensor::vData.Range = 2047;

	Scale(Cfg.Scale);
	uint32_t f = SamplingFrequency(Cfg.Freq);

	if (Cfg.bInter == true)
	{

		regaddr = LIS2DH12_CTRL_REG3;
		Write8(&regaddr, 1, LIS2DH12_CTRL_REG3_I1_OVERRUN | LIS2DH12_CTRL_REG3_I1_WTM);// |
				//LIS2DH12_CTRL_REG3_I1_ZYXDA | LIS2DH12_CTRL_REG3_I1_IA1);

		regaddr = LIS2DH12_CTRL_REG5;
		Write8(&regaddr, 1, LIS2DH12_CTRL_REG5_LIR_INT1 | LIS2DH12_CTRL_REG5_D4D_INT1);

		regaddr = LIS2DH12_INT1_CFG;
		Write8(&regaddr, 1, 0xf);

		regaddr = LIS2DH12_CTRL_REG6;
		Write8(&regaddr, 1, LIS2DH12_CTRL_REG6_INT_POLARITY_LOW);

		regaddr = LIS2DH12_INT1_DURATION;
		Write8(&regaddr, 1, 1000 / f);

		regaddr = LIS2DH12_INT1_THS;
		Write8(&regaddr, 1, 1);

		vbIntEn = true;
	}
	else
	{
		regaddr = LIS2DH12_INT1_CFG;
		Write8(&regaddr, 1, 0);
	}

	msDelay(1);

	// Flush FIFO
 	regaddr = LIS2DH12_FIFO_SRC_REG;
	d = Read8(&regaddr, 1) & LIS2DH12_FIFO_SRC_REG_FSS_MASK;

	uint8_t b[196];

	regaddr = LIS2DH12_OUT_X_L | 0x40;
	if (d > 0)
	{
		Device::Read(&regaddr, 1, b, d * 6);
	}

	Enable();

	return true;
}

bool AccelLis2dh12::Init(const TEMPSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbValid == false)
	{
		if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		{
			return false;
		}
	}

	uint8_t regaddr = LIS2DH12_TEMP_CFG_REG;
	Write8(&regaddr, 1, LIS2DH12_TEMP_CFG_EN);

	regaddr = LIS2DH12_CTRL_REG4;
	uint8_t d = Read8(&regaddr, 1) | LIS2DH12_CTRL_REG4_BDU;
	Write8(&regaddr, 1, d);

	regaddr = LIS2DH12_CTRL_REG1;
	d = Read8(&regaddr, 1);


	if (d & LIS2DH12_CTRL_REG1_LPEN)
	{
		TempSensor::Range(127);
	}
	else
	{
		TempSensor::Range(511);
	}

	return true;
}


uint16_t AccelLis2dh12::Scale(uint16_t Value)
{
	uint8_t regaddr = LIS2DH12_CTRL_REG4;
	uint8_t d = Read8(&regaddr, 1) & ~LIS2DH12_CTRL_REG4_FS_MASK;

	if (Value < 3)
	{
		Value = 2;
	}
	else if (Value < 6)
	{
		Value = 4;
		d |= LIS2DH12_CTRL_REG4_FS_4G;
	}
	else if (Value < 10)
	{
		Value = 8;
		d |= LIS2DH12_CTRL_REG4_FS_8G;
	}
	else
	{
		Value = 16;
		d |= LIS2DH12_CTRL_REG4_FS_16G;
	}

	Write8(&regaddr, 1, d);

	AccelSensor::vData.Scale = Value;

	return AccelSensor::Scale(Value);
}

/**
 * @brief	Set sampling frequency.
 *
 * The sampling frequency is relevant only in continuous mode.
 *
 * @return	Frequency in mHz (milliHerz)
 */
uint32_t AccelLis2dh12::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = LIS2DH12_CTRL_REG4;
	uint8_t d = Read8(&regaddr, 1) & ~(LIS2DH12_CTRL_REG4_HR);
	uint32_t f = 0;
	uint32_t range = 2047;
	uint32_t trange = 511;

	regaddr = LIS2DH12_CTRL_REG1;
	d = Read8(&regaddr, 1) & ~(LIS2DH12_CTRL_REG1_ODR_MASK | LIS2DH12_CTRL_REG1_LPEN);

	if (Freq == 0 || Freq >= 2000000)
	{
		// High freq.
		// 5.376 KHz, LP mode only
		f = 5376000;
		d |= LIS2DH12_CTRL_REG1_ODR_HR_LP | LIS2DH12_CTRL_REG1_LPEN;
		range = 127;
		trange = 127;
	}
	else if (Freq < 5000)
	{
		// 1 Hz
		f = 1;
		d |= LIS2DH12_CTRL_REG1_ODR_1HZ;
	}
	else if (Freq < 17500)
	{
		// 10 Hz
		f = 10;
		d |= LIS2DH12_CTRL_REG1_ODR_10HZ;
	}
	else if (Freq < 37500)
	{
		// 25 Hz
		f = 25;
		d |= LIS2DH12_CTRL_REG1_ODR_25HZ;
	}
	else if (Freq < 75000)
	{
		// 50 Hz
		f = 50;
		d |= LIS2DH12_CTRL_REG1_ODR_50HZ;
	}
	else if (Freq < 150000)
	{
		// 100 Hz
		f = 100;
		d |= LIS2DH12_CTRL_REG1_ODR_100HZ;
	}
	else if (Freq < 300000)
	{
		// 200 Hz
		f = 200;
		d |= LIS2DH12_CTRL_REG1_ODR_200HZ;
	}
	else if (Freq < 600000)
	{
		// 400 Hz
		f = 400;
		d |= LIS2DH12_CTRL_REG1_ODR_400HZ;
	}
	else if (Freq < 1400000)
	{
		// 1.344 KHz, HR/Normal
		f = 1344000;
		d |= LIS2DH12_CTRL_REG1_ODR_HR_LP;
	}
	else if (Freq < 2000000)
	{
		// 1.62 KHz, LP mode only
		f = 1620000;
		d |= LIS2DH12_CTRL_REG1_ODR_1620HZ | LIS2DH12_CTRL_REG1_LPEN;
		range = 127;
		trange = 127;
	}
	else
	{
		// 5.376 KHz, LP mode only
		f = 5376000;
		d |= LIS2DH12_CTRL_REG1_ODR_HR_LP | LIS2DH12_CTRL_REG1_LPEN;
		range = 127;
		trange = 127;
	}

	Write8(&regaddr, 1, d);

	if ((d & LIS2DH12_CTRL_REG1_LPEN) == 0)
	{
		regaddr = LIS2DH12_CTRL_REG4;
		d = Read8(&regaddr, 1) | LIS2DH12_CTRL_REG4_HR;
		Write8(&regaddr, 1, d);
	}

	// Read this register for changes to take effect
	regaddr = LIS2DH12_REFERENCE;
	d = Read8(&regaddr, 1);

	AccelSensor::Range(range);
	AccelSensor::vData.Range = range;
	TempSensor::Range(trange);

	return  AccelSensor::SamplingFrequency(f);
}

/**
 * @brief	Set and enable filter cutoff frequency
 *
 * Optional implementation can override this to implement filtering supported by the device
 *
 * @param	Freq : Filter frequency in mHz
 *
 * @return	Actual frequency in mHz
 */
uint32_t AccelLis2dh12::FilterFreq(uint32_t Freq)
{
	uint8_t regaddr = LIS2DH12_CTRL_REG2;
	uint8_t d = Read8(&regaddr, 1) & ~(LIS2DH12_CTRL_REG2_HPM_MASK | LIS2DH12_CTRL_REG2_FDS);

	if (Freq != 0)
	{
		d |= LIS2DH12_CTRL_REG2_FDS;
	}

	Write8(&regaddr, 1, d);

	regaddr = LIS2DH12_REFERENCE;
	d = Read8(&regaddr, 1);

	return AccelSensor::FilterFreq(Freq);
}

bool AccelLis2dh12::Enable()
{
	uint8_t regaddr = LIS2DH12_CTRL_REG1;
	uint8_t d = Read8(&regaddr, 1);

	Write8(&regaddr, 1, d | LIS2DH12_CTRL_REG1_XEN |
			LIS2DH12_CTRL_REG1_YEN | LIS2DH12_CTRL_REG1_ZEN);

	if (vbIntEn)
	{
		regaddr = LIS2DH12_CTRL_REG5;
		d = Read8(&regaddr, 1) | LIS2DH12_CTRL_REG5_FIFO_EN;
		Write8(&regaddr, 1, d);

		regaddr = LIS2DH12_FIFO_CTRL_REG;
		d = Read8(&regaddr, 1) & ~(LIS2DH12_FIFO_CTRL_REG_FM_MASK | LIS2DH12_FIFO_CTRL_REG_FTH_MASK);

		Write8(&regaddr, 1, d);

		d |= LIS2DH12_FIFO_CTRL_REG_FM_STREAM | 1;
		Write8(&regaddr, 1, d);

	}
	else
	{
		regaddr = LIS2DH12_FIFO_CTRL_REG;
		d = Read8(&regaddr, 1) & ~(LIS2DH12_FIFO_CTRL_REG_FM_MASK | LIS2DH12_FIFO_CTRL_REG_FTH_MASK);

		Write8(&regaddr, 1, d);
	}

	return true;
}

void AccelLis2dh12::Disable()
{
	uint8_t regaddr = LIS2DH12_CTRL_REG1;
	uint8_t d = Read8(&regaddr, 1) & ~(LIS2DH12_CTRL_REG1_XEN |
			LIS2DH12_CTRL_REG1_YEN | LIS2DH12_CTRL_REG1_ZEN);

	Write8(&regaddr, 1, d);

	regaddr = LIS2DH12_FIFO_CTRL_REG;
	d = Read8(&regaddr, 1) & ~(LIS2DH12_FIFO_CTRL_REG_FM_MASK | LIS2DH12_FIFO_CTRL_REG_FTH_MASK);
	Write8(&regaddr, 1, d);

	regaddr = LIS2DH12_CTRL_REG5;
	d = Read8(&regaddr, 1) & ~LIS2DH12_CTRL_REG5_FIFO_EN;
	Write8(&regaddr, 1, d);
}

void AccelLis2dh12::Reset()
{
	uint8_t regaddr = LIS2DH12_CTRL_REG1;
	uint8_t d = 0;

	Write8(&regaddr, 1, 0);

	regaddr = LIS2DH12_CTRL_REG2;
	Write8(&regaddr, 1, 0);

	regaddr = LIS2DH12_CTRL_REG3;
	Write8(&regaddr, 1, 0);

	regaddr = LIS2DH12_FIFO_CTRL_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LIS2DH12_CTRL_REG5;
	Write8(&regaddr, 1, LIS2DH12_CTRL_REG5_BOOT);
	msDelay(1);
	Write8(&regaddr, 1, 0);

	regaddr = LIS2DH12_REFERENCE;
	d = Read8(&regaddr,1);

	regaddr = LIS2DH12_INT1_SRC;
	d = Read8(&regaddr, 1);

}

void AccelLis2dh12::PowerOff()
{
	uint8_t regaddr = LIS2DH12_CTRL_REG1;

	Write8(&regaddr, 1, 0);
}

void AccelLis2dh12::IntHandler()
{
	uint8_t regaddr = LIS2DH12_STATUS_REG;
	uint8_t status = Read8(&regaddr, 1);

	regaddr = LIS2DH12_FIFO_SRC_REG;
	uint8_t fstatus = Read8(&regaddr, 1);

	regaddr = LIS2DH12_INT1_SRC;
	uint8_t isrc1 = Read8(&regaddr, 1);

	regaddr = LIS2DH12_INT2_SRC;
	uint8_t isrc2 = Read8(&regaddr, 1);

	if ((fstatus & LIS2DH12_FIFO_SRC_REG_WTM) || isrc1 || isrc2)
	{
		if (UpdateData() == true)
		{
			if (vIntHandler)
			{
				ACCELSENSOR_DATA data;

				AccelSensor::Read(data);

				vIntHandler(&data);
			}
		}
	}

	if (fstatus & LIS2DH12_FIFO_SRC_REG_OVRN_FIFO)
	{
		// Overrun, clear fifo
		regaddr = LIS2DH12_FIFO_CTRL_REG;
		uint8_t d = Read8(&regaddr, 1) & ~LIS2DH12_FIFO_CTRL_REG_FM_MASK;
		Write8(&regaddr, 1, d);
		d |= LIS2DH12_FIFO_CTRL_REG_FM_FIFO;
		Write8(&regaddr, 1, d);
	}
}

bool AccelLis2dh12::UpdateData()
{
	uint8_t regaddr = LIS2DH12_STATUS_REG;
	uint8_t d = Read8(&regaddr, 1);
	uint8_t fstatus;
	uint32_t ts = 0;

	if (vpTimer)
	{
		ts = vpTimer->uSecond();
	}

	regaddr = LIS2DH12_FIFO_SRC_REG;
	fstatus = Read8(&regaddr, 1);

	bool avail = false;

	if ((fstatus & LIS2DH12_FIFO_SRC_REG_FSS_MASK) & vbIntEn)
	{
		avail = true;
	}
	else if (d & 0xf)
	{
		avail = true;
	}

	if (avail == true)
	{
		// New data avail
		if (vpTimer)
		{
			AccelSensor::vData.Timestamp = ts;
		}
		for (int i = 0; i < (fstatus & 0x1f); i++)
		{
			regaddr = LIS2DH12_OUT_X_L | 0x40;
			Device::Read(&regaddr, 1, (uint8_t*)AccelSensor::vData.Val, 6);
		}
	}

	regaddr = LIS2DH12_STATUS_REG_AUX;
	d = Read8(&regaddr, 1);
	if (d & LIS2DH12_STATUS_REG_AUX_TDA)
	{
		if (vpTimer)
		{
			TempSensor::vData.Timestamp = ts;
		}

		regaddr = LIS2DH12_OUT_TEMP_L | 0x40;
		uint16_t t = 0;
		Device::Read(&regaddr, 1, (uint8_t*)&t, 2);

		int32_t r = TempSensor::Range();
		if (r <= 127)
		{
			TempSensor::vData.Temperature = t * 100 / 256 + 2500;
		}
		else
		{
			TempSensor::vData.Temperature = t * 100 / (64 * r) + 2500;
		}

		avail = true;
	}

	return avail;
}

