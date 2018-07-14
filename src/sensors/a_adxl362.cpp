/**-------------------------------------------------------------------------
@file	a_adxl362.cpp

@brief	Implementation of Analog ADXL362 accelerometer

@author	Hoang Nguyen Hoan
@date	July 13, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#include "sensors/a_adxl362.h"

bool AccelAdxl362::Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbInitialized)
		return true;;

	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);
	DeviceAddess(Cfg.DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	// Read chip id
	uint16_t id;
	uint8_t cmd[3] = { ADXL362_CMD_READ, ADXL362_DEVID_AD_REG, };

	id = Read16(cmd, 2);

	if (id != ADXL362_DEVID)
	{
		return false;
	}

	Reset();

	uint32_t freq = 0;

	cmd[0] = ADXL362_CMD_WRITE;
	cmd[1] = ADXL362_FILTER_CTL_REG;
	cmd[2] = ADXL362_FILTER_CTL_HALF_BW;

	if (Cfg.Freq < 25)
	{
		freq = 12.5;
		cmd[2] |= ADXL362_FILTER_CTL_ODR_12_5HZ;
	}
	else if (Cfg.Freq < 50)
	{
		freq = 25;
		cmd[2] |= ADXL362_FILTER_CTL_ODR_25HZ;
	}
	else if (Cfg.Freq < 100)
	{
		freq = 50;
		cmd[2] |= ADXL362_FILTER_CTL_ODR_50HZ;
	}
	else if (Cfg.Freq < 200)
	{
		freq = 100;
		cmd[2] |= ADXL362_FILTER_CTL_ODR_100HZ;
	}
	else if (Cfg.Freq < 400)
	{
		freq = 200;
		cmd[2] |= ADXL362_FILTER_CTL_ODR_200HZ;
	}
	else
	{
		freq = 400;
		cmd[2] |= ADXL362_FILTER_CTL_ODR_400HZ;
	}

	Write(cmd, 3, NULL, 0);

	Mode(Cfg.OpMode, Cfg.Freq);

	Scale(Cfg.Scale);

	if (Cfg.bInter)
	{

	}

	vbInitialized = true;

	return true;
}

bool AccelAdxl362::Enable()
{
	uint8_t cmd[3] = {ADXL362_CMD_WRITE, ADXL362_POWER_CTL_REG,
					  ADXL362_POWER_CTL_MEASURE_START | ADXL362_POWER_CTL_LOW_NOISE_NORMAL};

	Write(cmd, 3, NULL, 0);

	return true;
}

void AccelAdxl362::Disable()
{
	uint8_t cmd[3] = {ADXL362_CMD_WRITE, ADXL362_POWER_CTL_REG, ADXL362_POWER_CTL_MEASURE_STANDBY};

	Write(cmd, 3, NULL, 0);
}

void AccelAdxl362::Reset()
{
	uint8_t cmd[3] = { ADXL362_CMD_WRITE, ADXL362_SOFT_RESET_REG, ADXL362_SOFT_RESET };

	Write(cmd, 3, NULL, 0);
}

bool AccelAdxl362::StartSampling()
{
	uint8_t cmd[3] = {ADXL362_CMD_WRITE, ADXL362_POWER_CTL_REG,
					  ADXL362_POWER_CTL_MEASURE_START | ADXL362_POWER_CTL_LOW_NOISE_LOW_NOISE};

	Write(cmd, 3, NULL, 0);

	return true;
}

uint16_t AccelAdxl362::Scale(uint16_t Value)
{
	int retval = Value;
	uint8_t cmd[3] = { ADXL362_CMD_READ, ADXL362_FILTER_CTL_REG, };
	uint8_t regval = Read8(cmd, 2) & ~ADXL362_FILTER_CTL_RANGE_MASK;

	cmd[0] = ADXL362_CMD_WRITE;

	if (Value <= 2)
	{
		Value = 2;
		regval |= ADXL362_FILTER_CTL_RANGE_2G;
	}
	else if (Value <= 4)
	{
		Value = 4;
		regval |= ADXL362_FILTER_CTL_RANGE_4G;
	}
	else
	{
		Value = 8;
		regval |= ADXL362_FILTER_CTL_RANGE_8G;
	}

	cmd[2] = regval;

	Write(cmd, 3, NULL, 0);

	return AccelSensor::Scale(Value);
}

bool AccelAdxl362::Read(ACCELSENSOR_DATA *pData)
{
	if (pData == NULL)
	{
		return false;
	}

	*pData = vData;

	return true;
}

bool AccelAdxl362::UpdateData()
{
	uint8_t cmd[2] = { ADXL362_CMD_READ, ADXL362_STATUS_REG };
	uint8_t status = Read8(cmd, 2);
	bool retval = false;

	if (status & ADXL362_STATUS_DATA_READY)
	{
		if (vpTimer)
		{
			vData.Timestamp = vpTimer->uSecond();
		}
		else
		{
			vData.Timestamp++;
		}

		cmd[1] = ADXL362_XDATA_L_REG;
		vData.x = Read16(cmd, 2);

		cmd[1] = ADXL362_YDATA_L_REG;
		vData.y = Read16(cmd, 2);

		cmd[1] = ADXL362_ZDATA_L_REG;
		vData.z = Read16(cmd, 2);

		retval = true;
	}
	else if (status & ADXL362_STATUS_FIFO_READY)
	{

	}

	return retval;
}

bool AccelAdxl362::WakeOnEvent(bool bEnable, int Threshold)
{
	return false;
}
