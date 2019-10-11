/**-------------------------------------------------------------------------
@file	gyro_l3gd20h.cpp

@brief	Implementation of ST L3GD20H gyro sensor

@author	Hoang Nguyen Hoan
@date	July 28, 2019

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
#include "sensors/gyro_l3gd20h.h"


/**
 * @brief	Initialize gyro sensor.
 *
 * @param 	Cfg		: Configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool GyroL3gd20h::Init(const GYROSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbInitialized)
		return true;;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;
	uint8_t userctrl = 0;///*MPU9250_AG_USER_CTRL_FIFO_EN | MPU9250_AG_USER_CTRL_DMP_EN |*/ MPU9250_AG_USER_CTRL_I2C_MST_EN;
	uint8_t mst = 0;

	if (pTimer != NULL)
	{
		GyroSensor::vpTimer = pTimer;
	}

	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);

	Reset();

	msDelay(10);

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI ||
		(Cfg.DevAddr != L3GD20H_I2C_7BITS_DEVADDR0 && Cfg.DevAddr != L3GD20H_I2C_7BITS_DEVADDR0))
	{
		regaddr = L3GD20H_LOW_ODR_REG;
		d = Read8(&regaddr, 1);
		Write8(&regaddr, 1, d | L3GD20H_LOW_ODR_I2C_DIS);
		msDelay(1);
	}

	// Read chip id
	regaddr = L3GD20H_WHO_AM_I_A_REG;
	d = Read8(&regaddr, 1);

	if (d != L3GD20H_WHO_AM_I_A_ID && d != L3GD20_WHO_AM_I_A_ID)
	{
		return false;
	}

	DeviceID(d);
	Valid(true);

	return true;
}

bool GyroL3gd20h::Init(const TEMPSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	return true;
}

bool GyroL3gd20h::Enable()
{
	uint8_t regaddr = L3GD20H_CTRL1_REG;
	uint8_t d = Read8(&regaddr, 1);

	d |= (L3GD20H_CTRL1_XEN | L3GD20H_CTRL1_YEN | L3GD20H_CTRL1_ZEN | L3GD20H_CTRL1_PD);

	Write8(&regaddr, 1, d);

	return true;
}

void GyroL3gd20h::Disable()
{
	uint8_t regaddr = L3GD20H_CTRL1_REG;
	uint8_t d = Read8(&regaddr, 1);

	d &= ~ (L3GD20H_CTRL1_XEN | L3GD20H_CTRL1_YEN | L3GD20H_CTRL1_ZEN | L3GD20H_CTRL1_PD);

	Write8(&regaddr, 1, d);
}

void GyroL3gd20h::Reset()
{
	uint8_t regaddr = L3GD20H_LOW_ODR_REG;
	Write8(&regaddr, 1, L3GD20H_LOW_ODR_SW_RES);

	msDelay(1);

	uint8_t d = Write8(&regaddr, 1, 0);
}
