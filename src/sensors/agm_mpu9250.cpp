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

	regaddr = MPU9250_AG_CONFIG;
	Write8(&regaddr, 1, MPU9250_AG_CONFIG_FIFO_MODE_BLOCKING);

	vbInitialized = true;

	return true;
}

bool AgmMpu9250::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d;

	if (InitDefault(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

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

void AgmMpu9250::WakeOnMotion(bool bEnable)
{
	if (bEnable == true)
	{

	}
	else
	{

	}
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

bool AgmMpu9250::StartSampling()
{
	return true;
}

bool AgmMpu9250::Read(ACCELSENSOR_DATA *pData)
{
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

