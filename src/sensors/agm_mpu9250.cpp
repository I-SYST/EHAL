/*--------------------------------------------------------------------------
File   : agm_mpu9250.cpp

Author : Hoang Nguyen Hoan          						Nov. 18, 2017

Desc   : Implementation of TDK MPU-9250 accel, gyro, mag sensor


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

#include "idelay.h"
#include "i2c.h"
#include "spi.h"
#include "sensors/agm_mpu9250.h"

bool AgmMpu9250::InitDefault(uint32_t DevAddr, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (vbInitialized)
		return true;;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;

	Interface(pIntrf);
	DeviceAddess(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	if (DevAddr == MPU9250_I2C_DEV_ADDR0 || DevAddr == MPU9250_I2C_DEV_ADDR1)
	{
		// I2C mode
		vbSpi = false;
	}
	else
	{
		vbSpi = true;
	}

	// Read chip id
	regaddr = MPU9250_AG_WHO_AM_I;
	d = Read8((uint8_t*)&regaddr, 1);

	if (d != MPU9250_AG_WHO_AM_I_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	usDelay(200000);

	//regaddr = MPU9250_AG_CONFIG;
	//Write8(&regaddr, 1, MPU9250_AG_CONFIG_FIFO_MODE_BLOCKING);

	vbInitialized = true;

	//regaddr = MPU9250_AG_PWR_MGMT_1;
	//Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_CYCLE);

	return true;
}

bool AgmMpu9250::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d;

	if (InitDefault(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	regaddr = MPU9250_AG_LP_ACCEL_ODR;

	if (CfgData.Freq < 2)
	{
		Write8(&regaddr, 1, 2);
		vSampFreq = 1;
	}
	else if (CfgData.Freq < 4)
	{
		Write8(&regaddr, 1, 3);
		vSampFreq = 2;
	}
	else if (CfgData.Freq < 8)
	{
		Write8(&regaddr, 1, 4);
		vSampFreq = 4;
	}
	else if (CfgData.Freq < 16)
	{
		Write8(&regaddr, 1, 5);
		vSampFreq = 8;
	}
	else if (CfgData.Freq < 31)
	{
		Write8(&regaddr, 1, 6);
		vSampFreq = 16;
	}
	else if (CfgData.Freq < 62)
	{
		Write8(&regaddr, 1, 7);
		vSampFreq = 31;
	}
	else if (CfgData.Freq < 125)
	{
		Write8(&regaddr, 1, 8);
		vSampFreq = 62;
	}
	else if (CfgData.Freq < 250)
	{
		Write8(&regaddr, 1, 9);
		vSampFreq = 125;
	}
	else if (CfgData.Freq < 500)
	{
		Write8(&regaddr, 1, 10);
		vSampFreq = 250;
	}
	else
	{
		Write8(&regaddr, 1, 11);
		vSampFreq = 500;
	}

	Valid(true);

	return true;
}

bool AgmMpu9250::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d;

	if (InitDefault(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	return true;
}

bool AgmMpu9250::Init(const MAGSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d;

	if (InitDefault(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	return true;
}

bool AgmMpu9250::Enable()
{
	return true;
}

void AgmMpu9250::Disable()
{

}

void AgmMpu9250::Reset()
{
	uint8_t regaddr = MPU9250_AG_PWR_MGMT_1;

	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_H_RESET);
}

uint32_t AgmMpu9250::SamplingFrequency(uint32_t FreqHz)
{
	vSampFreq = FreqHz;

	return vSampFreq;
}

bool AgmMpu9250::StartSampling()
{
	return true;
}

bool AgmMpu9250::WakeOnMotion(bool bEnable, uint8_t Threshold)
{
    uint8_t regaddr;

	if (bEnable == true)
	{
	    regaddr = MPU9250_AG_PWR_MGMT_1;
	    Write8(&regaddr, 1, 0);

	    regaddr = MPU9250_AG_PWR_MGMT_2;
		Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_2_DIS_XA | MPU9250_AG_PWR_MGMT_2_DIS_YA |
				MPU9250_AG_PWR_MGMT_2_DIS_XG | MPU9250_AG_PWR_MGMT_2_DIS_YG);// | MPU9250_AG_PWR_MGMT_2_DIS_ZG);

		regaddr = MPU9250_AG_ACCEL_CONFIG2;
	    Write8(&regaddr, 1, (3<<MPU9250_AG_ACCEL_CONFIG2_ACCEL_FCHOICE_B_BITPOS) | (1<<MPU9250_AG_ACCEL_CONFIG2_A_DLPF_CFG_BITPOS));

	    regaddr = MPU9250_AG_INT_ENABLE;
	    Write8(&regaddr, 1, MPU9250_AG_INT_ENABLE_WOM_EN);

	    regaddr = MPU9250_AG_MOT_DETECT_CTRL;
	    Write8(&regaddr, 1, MPU9250_AG_MOT_DETECT_CTRL_ACCEL_INTEL_MODE | MPU9250_AG_MOT_DETECT_CTRL_ACCEL_INTEL_EN);

	    regaddr = MPU9250_AG_WOM_THR;
	    Write8(&regaddr, 1, Threshold);

		regaddr = MPU9250_AG_PWR_MGMT_1;
		Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_CYCLE);

	}
	else
	{
	    regaddr = MPU9250_AG_INT_ENABLE;
	    Write8(&regaddr, 1, 0);

	    regaddr = MPU9250_AG_PWR_MGMT_1;
		Write8(&regaddr, 1, 0);
	}

	return true;
}

uint8_t AgmMpu9250::Scale(uint8_t Value)
{
	uint8_t regaddr = MPU9250_AG_ACCEL_CONFIG;
	if (Value < 4)
	{
		Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_2G);
		AccelSensor::Scale(2);
	}
	else if (Value < 8)
	{
		Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_4G);
		AccelSensor::Scale(4);
	}
	else if (Value < 16)
	{
		Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_8G);
		AccelSensor::Scale(8);
	}
	else
	{
		Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_16G);
		AccelSensor::Scale(16);
	}

	return AccelSensor::Scale();
}

bool AgmMpu9250::UpdateData()
{
	return true;
}

bool AgmMpu9250::Read(ACCELSENSOR_DATA *pData)
{
	uint8_t regaddr = MPU9250_AG_ACCEL_XOUT_H;
	int8_t d[6];

	Read(&regaddr, 1, (uint8_t*)d, 6);

	pData->x = (((int16_t)d[0] & 0xFF) << 8) | d[1];
	pData->y = (((int16_t)d[2] & 0xFF) << 8) | d[3];
	pData->x = (((int16_t)d[4] & 0xFF) << 8) | d[5];

	if (vpTimer)
	{
		pData->Timestamp = vpTimer->uSecond();
	}

	return true;
}

bool AgmMpu9250::Read(GYROSENSOR_DATA*)
{
	return true;
}
bool AgmMpu9250::Read(MAGSENSOR_DATA*)
{
	return true;
}

int AgmMpu9250::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vbSpi == true)
	{
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int AgmMpu9250::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vbSpi == true)
	{
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

