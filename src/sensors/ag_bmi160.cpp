/**-------------------------------------------------------------------------
@file	ag_bmi160.cpp

@brief	Implementation of BOSCH BMI160 sensor Accel, Gyro


@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

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
#include <math.h>
#include <stdlib.h>

#include "istddef.h"
#include "coredev/spi.h"
#include "sensors/ag_bmi160.h"
#include "idelay.h"

bool AccelBmi160::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init((uint32_t)CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	vData.Range = Range(0x7FFF);
	Scale(CfgData.Scale);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	AccelBmi160::Enable();

	if (CfgData.bInter)
	{
		uint8_t regaddr = BMI160_INT_EN_0;
		uint8_t d = 0;//BMI160_INT_EN_0_INT_ANYMO_X_EN | BMI160_INT_EN_0_INT_ANYMO_Y_EN | BMI160_INT_EN_0_INT_ANYMO_Z_EN;

		Write8(&regaddr, 1, d);

		regaddr = BMI160_LATCH;
		Write8(&regaddr, 1, BMI160_LATCH_INT_LATCH_MASK);

		regaddr = BMI160_INT_EN_1;
		Write8(&regaddr, 1, /*BMI160_INT_EN_1_INT_HIGHG_X_EN | BMI160_INT_EN_1_INT_HIGHG_Y_EN |
				BMI160_INT_EN_1_INT_HIGHG_Z_EN |*/ BMI160_INT_EN_1_INT_DRDY_EN);

		regaddr = BMI160_INT_OUT_CTRL;
		d = BMI160_INT_OUT_CTRL_INT1_OUTPUT_EN | BMI160_INT_OUT_CTRL_INT2_OUTPUT_EN |
			BMI160_INT_OUT_CTRL_INT1_EDGE_CTRL | BMI160_INT_OUT_CTRL_INT2_EDGE_CTRL;

		if (CfgData.IntPol == DEVINTR_POL_HIGH)
		{
			d |= BMI160_INT_OUT_CTRL_INT1_LVL | BMI160_INT_OUT_CTRL_INT2_LVL;
		}

		Write8(&regaddr, 1, d);

		//regaddr = BMI160_INT_MAP_0;
		//Write8(&regaddr, 1, BMI160_INT_MAP_0_INT1_HIGHG | BMI160_INT_MAP_0_INT1_ANYMOTION);

		regaddr = BMI160_INT_MAP_1;
		Write8(&regaddr, 1, BMI160_INT_MAP_1_INT1_DRDY);
	}

	return true;
}

uint8_t AccelBmi160::Scale(uint8_t Value)
{
	uint8_t regaddr = BMI160_ACC_RANGE;

	if (Value < 4)
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_2G);
		Value = 2;
	}
	else if (Value < 8)
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_4G);
		Value = 4;
	}
	else
	{
		Write8(&regaddr, 1, BMI160_ACC_RANGE_ACC_RANGE_8G);
		Value = 8;
	}

	vData.Scale = Value;

	return AccelSensor::Scale(Value);
}

uint32_t AccelBmi160::FilterFreq(uint32_t Freq)
{
	uint8_t t = AccelSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI160_ACC_CONF;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_ACC_CONF_ACC_BWP_MASK;

	if ( t < 4)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_OSR2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_OSR4;
		t = 2;
	}
	else if (t < 16)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG8;
		t = 3;
	}
	else if (t < 32)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG16;
		t = 4;
	}
	else if (t < 64)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG32;
		t = 5;
	}
	else if (t < 128)
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG64;
		t = 6;
	}
	else
	{
		d |= BMI160_ACC_CONF_ACC_BWP_AVG128;
		t = 7;
	}

	Write8(&regaddr, 1, d);

	return AccelSensor::FilterFreq(AccelSensor::SamplingFrequency() >> t);
}

uint32_t AccelBmi160::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI160_ACC_CONF;
	uint32_t accconf = Read8(&regaddr, 1) & ~BMI160_ACC_CONF_ACC_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 100000)
	{
		for (int i = 1; i < 8; i++)
		{
			uint32_t t = 100000 >> (8 - i);
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				accconf &= ~BMI160_ACC_CONF_ACC_ODR_MASK;
				accconf |= i;
				f = t;
				dif = x;
			}
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			uint32_t t = 100000 << i;
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				accconf &= ~BMI160_ACC_CONF_ACC_ODR_MASK;
				accconf |= i | 0x8;
				f = t;
				dif = x;
			}
		}
	}

	if (f < 12500)
	{
		// under sampling
		accconf |= BMI160_ACC_CONF_ACC_US;
	}
	else
	{
		accconf &= ~BMI160_ACC_CONF_ACC_US;
	}

	Write8(&regaddr, 1, accconf);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

bool AccelBmi160::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = BMI160_FIFO_CONFIG_1;
	d = Read8(&regaddr, 1) | BMI160_FIFO_CONFIG_1_FIFO_ACC_EN;

	Write8(&regaddr, 1, d);

	msDelay(1); // Require delay, do not remove

	regaddr = BMI160_CMD;

	if (Sensor::SamplingFrequency() > 12500)
	{
		Write8(&regaddr, 1, BMI160_CMD_ACC_SET_PMU_MODE_NORMAL);
	}
	else
	{
		Write8(&regaddr, 1, BMI160_CMD_ACC_SET_PMU_MODE_LOWPOWER);
	}

	msDelay(20); // Require delay, do not remove

	regaddr = BMI160_ERR_REG;
	d = Read8(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	return true;
}

void AccelBmi160::Disable()
{
	uint8_t regaddr = BMI160_CMD;

	Write8(&regaddr, 1, BMI160_CMD_ACC_SET_PMU_MODE_SUSPEND);

	msDelay(10);

	regaddr = BMI160_ERR_REG;
	Read8(&regaddr, 1);

	regaddr = BMI160_FIFO_CONFIG_1;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_FIFO_CONFIG_1_FIFO_ACC_EN;

	Write8(&regaddr, 1, d);
}

bool GyroBmi160::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	GyroSensor::Type(SENSOR_TYPE_GYRO);

	vData.Range = Range(0x7FFF);

	Sensitivity(CfgData.Sensitivity);
	SamplingFrequency(CfgData.Freq);
	FilterFreq(CfgData.FltrFreq);

	GyroBmi160::Enable();

	return true;
}

uint32_t GyroBmi160::FilterFreq(uint32_t Freq)
{
	uint8_t t = GyroSensor::SamplingFrequency() / Freq;
	uint8_t regaddr = BMI160_GYR_CONF;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_GYR_CONF_GYR_BWP_MASK;

	if ( t < 4)
	{
		d |= BMI160_GYR_CONF_GYR_BWP_OSR2;
		t = 1;
	}
	else if (t < 8)
	{
		d |= BMI160_GYR_CONF_GYR_BWP_OSR4;
		t = 2;
	}
	else
	{
		d |= BMI160_GYR_CONF_GYR_BWP_NORMAL;
		t = 3;
	}

	Write8(&regaddr, 1, d);

	return GyroSensor::FilterFreq(GyroSensor::SamplingFrequency() >> t);
}

uint32_t GyroBmi160::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI160_GYR_CONF;
	uint32_t odrval = Read8(&regaddr, 1) & ~BMI160_GYR_CONF_GYR_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 100000)
	{
		for (int i = 6; i < 8; i++)
		{
			uint32_t t = 100000 >> (8 - i);
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				odrval &= ~BMI160_GYR_CONF_GYR_ODR_MASK;
				odrval |= i;
				f = t;
				dif = x;
			}
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			uint32_t t = 100000 << i;
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				odrval &= ~BMI160_GYR_CONF_GYR_ODR_MASK;
				odrval |= i | 0x8;
				f = t;
				dif = x;
			}
		}
	}

	Write8(&regaddr, 1, odrval);

	msDelay(1);

	return Sensor::SamplingFrequency(f);
}

uint32_t GyroBmi160::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = BMI160_GYR_RANGE;
	uint32_t range = 0;

	if (Value < 250)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_125);
		range = 125;
	}
	else if (Value < 500)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_250);
		range = 250;
	}
	else if (Value < 1000)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_500);
		range = 500;
	}
	else if (Value < 2000)
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_1000);
		range = 1000;
	}
	else
	{
		Write8(&regaddr, 1, BMI160_GYR_RANGE_GYR_RANGE_2000);
		range = 2000;
	}

	vData.Sensitivity = range;

	return GyroSensor::Sensitivity(range);
}

bool GyroBmi160::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = BMI160_FIFO_CONFIG_1;
	d = Read8(&regaddr, 1) | BMI160_FIFO_CONFIG_1_FIFO_GYR_EN;

	Write8(&regaddr, 1, d);

	msDelay(5); // Require delay, do not remove

	regaddr = BMI160_CMD;

	if (Sensor::SamplingFrequency() < 25000)
	{
		Write8(&regaddr, 1, BMI160_CMD_GYRO_SET_PMU_MODE_FASTSTARTUP);
	}
	else
	{
		Write8(&regaddr, 1, BMI160_CMD_GYRO_SET_PMU_MODE_NORMAL);
	}

	msDelay(40); // Require delay, do not remove

	regaddr = BMI160_ERR_REG;
	d = Read8(&regaddr, 1);

	if (d != 0)
	{
		return false;
	}

	return true;
}

void GyroBmi160::Disable()
{
	uint8_t regaddr = BMI160_CMD;

	Write8(&regaddr, 1, BMI160_CMD_GYRO_SET_PMU_MODE_SUSPEND);

	msDelay(10);

	regaddr = BMI160_ERR_REG;
	Read8(&regaddr, 1);

	regaddr = BMI160_FIFO_CONFIG_1;
	uint8_t d = Read8(&regaddr, 1) & ~BMI160_FIFO_CONFIG_1_FIFO_GYR_EN;

	Write8(&regaddr, 1, d);
}

bool MagBmi160::Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint8_t regaddr = BMI160_CMD;
	uint8_t d;

	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	vDevAddr = Cfg.DevAddr;

	Write8(&regaddr, 1, BMI160_CMD_MAG_SET_PMU_MODE_NORMAL);

	msDelay(1);

	regaddr = BMI160_PMU_STATUS;
	d = Read8(&regaddr, 1);

	// Set IF : prim autoconf, secondary : Mag
	regaddr = BMI160_IF_CONF;
	d = Read8(&regaddr, 1) & ~BMI160_IF_CONF_IF_MODE_MASK;
	d |= BMI160_IF_CONF_IF_MODE_AUTO_MAG;
	Write8(&regaddr, 1, d);

	regaddr = BMI160_MAG_IF_0;
	Write8(&regaddr, 1, Cfg.DevAddr << 1);

	regaddr = BMI160_MAG_IF_1;
	Write8(&regaddr, 1, BMI160_MAG_IF_1_MAG_RD_BURST_8 | BMI160_MAG_IF_1_MAG_MANUAL_EN);

	return MagBmm150::Init(Cfg, pIntrf, pTimer);
}

uint32_t MagBmi160::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr = BMI160_MAG_CONF;
	uint32_t odrval = Read8(&regaddr, 1) & ~BMI160_MAG_CONF_MAG_ODR_MASK;
	uint32_t f = 0;
	uint32_t dif = 100000;

	if (Freq < 50000)
	{
		for (int i = 0; i < 7; i++)
		{
			uint32_t t = 50000 >> (7 - i);
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				odrval &= ~BMI160_MAG_CONF_MAG_ODR_MASK;
				odrval |= i;
				f = t;
				dif = x;
			}
		}
	}
	else
	{
		for (int i = 0; i < 5; i++)
		{
			uint32_t t = 50000 << i;
			uint32_t x = labs(Freq - t);
			if (x < dif)
			{
				odrval &= ~BMI160_MAG_CONF_MAG_ODR_MASK;
				odrval |= (i | 0x8) - 1;
				f = t;
				dif = x;
			}
		}
	}

	Write8(&regaddr, 1, odrval);

	msDelay(2);

	return MagBmm150::SamplingFrequency(f);
}

bool MagBmi160::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = BMI160_CMD;
	Write8(&regaddr, 1, BMI160_CMD_MAG_SET_PMU_MODE_NORMAL);

	msDelay(10);

	if (MagBmm150::Enable() == false)
	{
		return false;
	}

	msDelay(10);

	regaddr = BMI160_FIFO_CONFIG_1;
	d = Read8(&regaddr, 1) | BMI160_FIFO_CONFIG_1_FIFO_MAG_EN;

	Write8(&regaddr, 1, d);

	msDelay(2); // Require delay, do not remove

	// Must disable manual mode for BMI160 fifo update
	regaddr = BMI160_MAG_IF_1;
	Write8(&regaddr, 1, BMI160_MAG_IF_1_MAG_RD_BURST_8);

	// NOTE : Must set this register for the BMI160 to send the command to secondary IF
	// to read Mag data.  Mag data will not get updated in FIFO if not set.
	regaddr = BMI160_MAG_IF_2;
	Write8(&regaddr, 1, BMM150_DATA_X_LSB_REG);

	return true;
}

int MagBmi160::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	uint8_t regaddr = BMI160_MAG_IF_4;
	uint8_t auxaddr = pCmdAddr[0];
	int cnt = 0;

	while (BuffLen > 0)
	{
		regaddr = BMI160_MAG_IF_2;
		Write8(&regaddr, 1, auxaddr);
		msDelay(10);

		int len = min(BuffLen, 8);

		// Note : data is read from starting from MAG_X register, not from MAG_IF_4
		regaddr = BMI160_DATA_MAG_X_LSB;
		len = Device::Read(&regaddr, 1, pBuff, len);

		BuffLen -= len;
		pBuff += len;
		cnt += len;
		auxaddr += len;
	}

	return cnt;
}

int MagBmi160::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	uint8_t regaddr = BMI160_MAG_IF_4;
	uint8_t auxaddr = pCmdAddr[0];
	int cnt = 0;

	for (int i = 0; i < DataLen; i++, auxaddr++)
	{
		regaddr = BMI160_MAG_IF_4;
		Write8(&regaddr, 1, pData[i]);
		//msDelay(10);
		regaddr = BMI160_MAG_IF_3;
		Write8(&regaddr, 1, auxaddr);
		msDelay(10);
		cnt++;
	}

	return cnt;
}

bool AgBmi160::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
		return true;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;

	Interface(pIntrf);
	Device::DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	// Read chip id
	regaddr = BMI160_CHIP_ID_REG;
	d = Read8(&regaddr, 1);

	if (d != BMI160_CHIP_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	msDelay(50);

	regaddr = BMI160_CMD;
	Write8(&regaddr, 1, BMI160_CMD_FIFO_FLUSH);

	msDelay(10);

	regaddr = BMI160_FIFO_CONFIG_0;
	Write8(&regaddr, 1, 7);

	msDelay(2);

	regaddr = BMI160_FIFO_CONFIG_1;
	d = Read8(&regaddr, 1) & ~(BMI160_FIFO_CONFIG_1_FIFO_TIME_EN | BMI160_FIFO_CONFIG_1_FIFO_HEADER_EN);

	Write8(&regaddr, 1, d | BMI160_FIFO_CONFIG_1_FIFO_TIME_EN | BMI160_FIFO_CONFIG_1_FIFO_HEADER_EN);

	msDelay(2);

	//regaddr = BMI160_FIFO_DOWNS;
	//Write8(&regaddr, 1, (2<<BMI160_FIFO_DOWNS_GYR_FIFO_DOWNS_BITPOS) | BMI160_FIFO_DOWNS_GYR_FIFO_FILT_DATA |
	//					(2<<BMI160_FIFO_DOWNS_ACC_FIFO_DOWNS_BITPOS) | BMI160_FIFO_DOWNS_ACC_FIFO_FILT_DATA);

	return true;
}

bool AgBmi160::Enable()
{
	AccelBmi160::Enable();
	GyroBmi160::Enable();
	MagBmi160::Enable();

	return true;
}

void AgBmi160::Disable()
{
	AccelBmi160::Disable();
	GyroBmi160::Disable();
}

void AgBmi160::Reset()
{
	uint8_t regaddr = BMI160_CMD;

	Write8(&regaddr, 1, BMI160_CMD_SOFT_RESET);

	msDelay(1);

	//MagBmi160::Reset();

//	msDelay(5);

	// Read err register to clear
	regaddr = BMI160_ERR_REG;
	Read8(&regaddr, 1);
}

bool AgBmi160::UpdateData()
{
	uint8_t regaddr = BMI160_FIFO_LENGTH_0;
	int len = 0;

	Device::Read(&regaddr, 1, (uint8_t*)&len, 2);
	if (len <= 0)
	{
		return false;
	}

	//printf("len %d\r\n", len);

	uint8_t buff[BMI160_FIFO_MAX_SIZE];
	uint64_t t = 0;

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	//regaddr = BMI160_DATA_MAG_X_LSB;
	//Device::Read(&regaddr, 1, (uint8_t*)MagSensor::vData.Val, 6);
	//printf("mx %d %d %d\r\n", MagSensor::vData.X, MagSensor::vData.Y, MagSensor::vData.Z);

	len += 4; // read time stamp

	regaddr = BMI160_FIFO_DATA;
	len = Device::Read(&regaddr, 1, buff, len);

	uint8_t *p = buff;
	uint8_t dflag = 0;
	bool res = false;

	while (len > 0)
	{
		BMI160_HEADER *hdr = (BMI160_HEADER *)p;

		len--;

		if (*p == 0x80 || len < 1)
		{
			break;
		}

		p++;

		if (hdr->Type == BMI160_FRAME_TYPE_DATA)
		{
			//printf("Data frame %x %d\r\n", *p, len);
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_MAG)
			{
				if (len >= 8)
				{
					dflag |= (1<<2);
					memcpy(MagSensor::vData.Val, p, 6);

					MagSensor::vData.Timestamp = t;
					MagSensor::vData.Val[0] >>= 3;
					MagSensor::vData.Val[1] >>= 3;
					MagSensor::vData.Val[2] >>= 1;
				}
				p += 8;
				len -= 8;
			}
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_GYRO)
			{
				if (len >= 6)
				{
					dflag |= (1<<1);
					memcpy(GyroBmi160::vData.Val, p, 6);
					GyroBmi160::vData.Timestamp = t;
					GyroBmi160::vData.Sensitivity = GyroSensor::Sensitivity();
				}
				p += 6;
				len -= 6;
			}
			if (hdr->Parm & BMI160_FRAME_DATA_PARM_ACCEL)
			{
				if (len >= 6)
				{
					dflag |= (1<<0);
					memcpy(AccelBmi160::vData.Val, p, 6);
					AccelBmi160::vData.Timestamp = t;
					AccelBmi160::vData.Scale = AccelSensor::Scale();
				}
				p += 6;
				len -= 6;
			}
		}
		else if (hdr->Type == BMI160_FRAME_TYPE_CONTROL)
		{
			switch (hdr->Parm)
			{
				case BMI160_FRAME_CONTROL_PARM_SKIP:
					AccelBmi160::vDropCnt += *p;
					GyroBmi160::vDropCnt = AccelBmi160::vDropCnt;
					len--;
					p++;
					break;
				case BMI160_FRAME_CONTROL_PARM_TIME:
					if (len >= 3)
					{
						dflag |= (1<<3);
						uint64_t t = 0;

						memcpy(&t, p, 3);
						t &= 0xFFFFFF;
						t *= BMI160_TIME_RESOLUTION_USEC;

						if (vpTimer == nullptr)
						{
							if (dflag & 1)
							{
								AccelBmi160::vData.Timestamp = t;
							}
							if (dflag & 2)
							{
								GyroBmi160::vData.Timestamp = t;
							}
							if (dflag & 4)
							{
								MagBmi160::vData.Timestamp = t;
							}
						}
					}
					len -= 3;
					p += 3;
					break;
				case BMI160_FRAME_CONTROL_PARM_INPUT:
					//Read8(&regaddr, 1);
					//printf("Input\r\n");
					p++;
					len--;
					break;
//				default:
					//printf("Control??\r\n");
			}
		}
		else
		{
			//printf("Unknown\r\n");
		}
	}

	if (dflag == 0xf && vEvtHandler)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}

	return res;
}

void AgBmi160::IntHandler()
{
	bool res = false;

	uint8_t regaddr = BMI160_STATUS;
	uint8_t d[8];

	// Read all status
	Read(&regaddr, 1, d, 5);

	if (d[2] & BMI160_INT_STATUS_1_DRDY_INT)
	{
		UpdateData();
	}
}

