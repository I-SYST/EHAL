/**-------------------------------------------------------------------------
@file	accel_h3lis331dl.cpp

@brief	Implementation of ST H3LIS331DL accel. sensor

@author	Hoang Nguyen Hoan
@date	Oct. 2, 2019

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

#include "coredev/spi.h"
#include "sensors/accel_h3lis331dl.h"

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
bool AccelH3lis331dl::Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;

	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	Reset();

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI || (vDevAddr != H3LIS331DL_I2C_DEVADDR && vDevAddr != H3LIS331DL_I2C_DEVADDR1))
	{
		regaddr = H3LIS331DL_CTRL_REG4_REG;
		if (((SPI*)vpIntrf)->Mode() == SPIMODE_3WIRE)
		{
			d = Read8(&regaddr, 1) | H3LIS331DL_CTRL_REG4_SIM_3WIRE;
			Write8(&regaddr, 1, d);
		}
	}

	// Read chip id
	regaddr = H3LIS331DL_WHO_AM_I_REG;
	d = Read8(&regaddr, 1);

	if (d != H3LIS331DL_WHO_AM_I_ID)
	{
		return false;
	}

	DeviceID(d);
	Valid(true);

	Range(32767);

	regaddr = H3LIS331DL_CTRL_REG4_REG;
	d = Read8(&regaddr, 1) & ~H3LIS331DL_CTRL_REG4_BDU_SINGLE;

	if (Cfg.OpMode == SENSOR_OPMODE_SINGLE)
	{
		Write8(&regaddr, 1, d | H3LIS331DL_CTRL_REG4_BDU_SINGLE);
	}

	vOpMode = Cfg.OpMode;

	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);
	FilterFreq(Cfg.FltrFreq);

	if (Cfg.bInter)
	{
		regaddr = H3LIS331DL_CTRL_REG3_A_REG;
		d = Read8(&regaddr, 1) & ~(H3LIS331DL_CTRL_REG3_IHL_LOW | H3LIS331DL_CTRL_REG3_PP_OD);

		if (Cfg.IntPol == DEVINTR_POL_LOW)
		{
			d |= H3LIS331DL_CTRL_REG3_IHL_LOW;
		}

		Write8(&regaddr, 1, d);

		regaddr = H3LIS331DL_INT1_CFG_REG;
		Write8(&regaddr, 1, 0x3F);

		regaddr = H3LIS331DL_INT2_CFG_REG;
		Write8(&regaddr, 1, 0x3F);
	}

	regaddr = H3LIS331DL_CTRL_REG1_REG;
	d = Read8(&regaddr, 1) | H3LIS331DL_CTRL_REG1_XEN | H3LIS331DL_CTRL_REG1_YEN | H3LIS331DL_CTRL_REG1_ZEN;
	Write8(&regaddr, 1, d);

	return true;
}

uint16_t AccelH3lis331dl::Scale(uint16_t Value)
{
	uint8_t regaddr = H3LIS331DL_CTRL_REG4_REG;
	uint8_t d = Read8(&regaddr, 1) & ~H3LIS331DL_CTRL_REG4_FS_MASK;
	uint16_t retval = 0;

	if (Value < 150)
	{
		d |= H3LIS331DL_CTRL_REG4_FS_100G;
		retval = 100;
	}
	else if (Value < 250)
	{
		d |= H3LIS331DL_CTRL_REG4_FS_200G;
		retval = 200;
	}
	else
	{
		d |= H3LIS331DL_CTRL_REG4_FS_400G;
		retval = 400;
	}

	Write8(&regaddr, 1, d);

	return retval;
}

uint32_t AccelH3lis331dl::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = H3LIS331DL_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(H3LIS331DL_CTRL_REG1_DR_MASK | H3LIS331DL_CTRL_REG1_PM_MASK);
	uint8_t wd = H3LIS331DL_CTRL_REG5_TURNON_DISABLE;

	if (Freq < 750)
	{
		wd = H3LIS331DL_CTRL_REG5_TURNON_LP;
		d |= H3LIS331DL_CTRL_REG1_PM_LP05HZ;
		Freq = 500;
	}
	else if (Freq < 1500)
	{
		wd = H3LIS331DL_CTRL_REG5_TURNON_LP;
		d |= H3LIS331DL_CTRL_REG1_PM_LP1HZ;
		Freq = 1000;
	}
	else if (Freq < 3500)
	{
		wd = H3LIS331DL_CTRL_REG5_TURNON_LP;
		d |= H3LIS331DL_CTRL_REG1_PM_LP2HZ;
		Freq = 2000;
	}
	else if (Freq < 7500)
	{
		wd = H3LIS331DL_CTRL_REG5_TURNON_LP;
		d |= H3LIS331DL_CTRL_REG1_PM_LP5HZ;
		Freq = 5000;
	}
	else if (Freq < 25000)
	{
		wd = H3LIS331DL_CTRL_REG5_TURNON_LP;
		d |= H3LIS331DL_CTRL_REG1_PM_LP10HZ;
		Freq = 10000;
	}
	else if (Freq < 70000)
	{
		d |= H3LIS331DL_CTRL_REG1_PM_NORMAL | H3LIS331DL_CTRL_REG1_DR_ODR50HZ;
		Freq = 50000;
	}
	else if (Freq < 200000)
	{
		d |= H3LIS331DL_CTRL_REG1_PM_NORMAL | H3LIS331DL_CTRL_REG1_DR_ODR100HZ;
		Freq = 100000;
	}
	else if (Freq < 650000)
	{
		d |= H3LIS331DL_CTRL_REG1_PM_NORMAL | H3LIS331DL_CTRL_REG1_DR_ODR400HZ;
		Freq = 400000;
	}
	else
	{
		d |= H3LIS331DL_CTRL_REG1_PM_NORMAL | H3LIS331DL_CTRL_REG1_DR_ODR1KHZ;
		Freq = 1000000;
	}

	Write8(&regaddr, 1, d);

	regaddr = H3LIS331DL_CTRL_REG5_REG;
	Write8(&regaddr, 1, wd);

	return Sensor::SamplingFrequency(Freq);
}

uint32_t AccelH3lis331dl::FilterFreq(uint32_t Freq)
{
	uint8_t regaddr = H3LIS331DL_CTRL_REG2_REG;
	uint8_t d = Read8(&regaddr, 1) & ~H3LIS331DL_CTRL_REG2_HPCF_MASK;

	switch (Sensor::SamplingFrequency())
	{
		case 50000:
			if (Freq < 200)
			{
				d |= 3;
				Freq = 125;
			}
			else if (Freq < 350)
			{
				d |= 2;
				Freq = 250;
			}
			else if (Freq < 750)
			{
				d |= 1;
				Freq = 500;
			}
			else
			{
				Freq = 1000;
			}
			break;
		case 100000:
			if (Freq < 300)
			{
				d |= 3;
				Freq = 250;
			}
			else if (Freq < 750)
			{
				d |= 2;
				Freq = 500;
			}
			else if (Freq < 1500)
			{
				d |= 1;
				Freq = 1000;
			}
			else
			{
				Freq = 2000;
			}
			break;
		case 400000:
			if (Freq < 1500)
			{
				d |= 3;
				Freq = 1000;
			}
			else if (Freq < 3000)
			{
				d |= 2;
				Freq = 2000;
			}
			else if (Freq < 6000)
			{
				d |= 1;
				Freq = 4000;
			}
			else
			{
				Freq = 8000;
			}
			break;
		case 1000000:
			if (Freq < 3500)
			{
				d |= 3;
				Freq = 2500;
			}
			else if (Freq < 7500)
			{
				d |= 2;
				Freq = 5000;
			}
			else if (Freq < 15000)
			{
				d |= 1;
				Freq = 10000;
			}
			else
			{
				Freq = 20000;
			}
			break;
	}

	Write8(&regaddr, 1, d);

	return AccelSensor::FilterFreq(Freq);
}

bool AccelH3lis331dl::Enable()
{
	uint8_t regaddr = H3LIS331DL_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) | H3LIS331DL_CTRL_REG1_XEN |
				H3LIS331DL_CTRL_REG1_YEN | H3LIS331DL_CTRL_REG1_ZEN;

	Write8(&regaddr, 1, d);

	return true;
}

void AccelH3lis331dl::Disable()
{
	uint8_t regaddr = H3LIS331DL_CTRL_REG1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(H3LIS331DL_CTRL_REG1_XEN |
				H3LIS331DL_CTRL_REG1_YEN | H3LIS331DL_CTRL_REG1_ZEN);

	Write8(&regaddr, 1, d);
}

void AccelH3lis331dl::Reset()
{
	uint8_t regaddr = H3LIS331DL_HP_FILTER_RESET_REG;
	uint8_t d = Read8(&regaddr, 1);	// Dummy read to reset high pass filter data

	regaddr = H3LIS331DL_CTRL_REG2_REG;
	d = Read8(&regaddr, 1) | H3LIS331DL_CTRL_REG2_BOOT;
	Write8(&regaddr, 1, d);
}

bool AccelH3lis331dl::UpdateData()
{
	uint8_t regaddr = H3LIS331DL_STATUS_REG_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d & 0xf)
	{
		if (vpTimer)
		{
			vData.Timestamp = vpTimer->uSecond();
		}
		regaddr = H3LIS331DL_OUT_X_L_REG;
		Read(&regaddr, 1, (uint8_t*)vData.Val, 6);

		//regaddr = H3LIS331DL_OUT_X_H_REG;
		//vData.X = Read8(&regaddr, 1);

		return true;
	}

	return false;
}

void AccelH3lis331dl::IntHandler()
{
	uint8_t regaddr = H3LIS331DL_INT1_SRC_REG;
	uint8_t d1 = Read8(&regaddr, 1);
	uint8_t d2;

	regaddr = H3LIS331DL_INT2_SRC_REG;
	d2 = Read8(&regaddr, 1);

	UpdateData();
}

/**
 * @brief	Read device's register/memory block.
 *
 * This default implementation sets bit 7 of the Cmd/Addr byte for SPI read access as most
 * devices work this way on SPI interface. Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior reading data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pBuff		: Data buffer container
 * @param	BuffLen		: Data buffer size
 *
 * @return	Actual number of bytes read
 */
int AccelH3lis331dl::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// Most sensor that supports SPI have this for reading registers
		// overload this function if different
		*pCmdAddr |= 0x80;
		if (BuffLen > 1)
		{
			*pCmdAddr |= 0x40;	// Multi bytes read with address auto increments
		}
	}
	else
	{
		if (BuffLen > 1)
		{
			*pCmdAddr |= 0x80;	// Multi bytes read with address auto increments
		}
	}

	return vpIntrf->Read(vDevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

/**
 * @brief	Write to device's register/memory block
 *
 * This default implementation clears bit 7 of the Cmd/Addr byte for SPI write access as most
 * devices work this way on SPI interface.  Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior writing data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pData		: Data buffer to be written to the device
 * @param	DataLen		: Size of data
 *
 * @return	Actual number of bytes written
 */
int AccelH3lis331dl::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x3F;
		if (DataLen > 1)
		{
			*pCmdAddr |= 0x40;	// Multi bytes write with address auto increments
		}
	}
	else
	{
		*pCmdAddr &= 0x7F;
		if (DataLen > 1)
		{
			*pCmdAddr |= 0x80;	// Multi bytes write with address auto increments
		}
	}

	return vpIntrf->Write(vDevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
}

