/**-------------------------------------------------------------------------
@file	am_lsm303c.cpp

@brief	Implementation of ST LSM303C accel, mag sensor

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

#include "coredev/spi.h"
#include "sensors/am_lsm303c.h"

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
bool AccLsm303c::Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	return true;
}

bool AccLsm303c::Init(const TEMPSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	return true;
}

bool AccLsm303c::Enable()
{
	return true;
}
void AccLsm303c::Disable()
{

}
void AccLsm303c::Reset()
{

}

// Default base initialization. Does detection and set default config for all sensor.
// All sensor init must call this first prio to initializing itself
bool AccLsm303c::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbInitialized)
		return true;;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;
	//uint8_t userctrl = 0;///*MPU9250_AG_USER_CTRL_FIFO_EN | MPU9250_AG_USER_CTRL_DMP_EN |*/ MPU9250_AG_USER_CTRL_I2C_MST_EN;
	//uint8_t mst = 0;

	Interface(pIntrf);
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		AccelSensor::vpTimer = pTimer;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI || DevAddr != LSM303C_ACCEL_I2C_DEVADDR)
	{
		regaddr = LSM303C_CTRL_REG4_A_REG;
		Write8(&regaddr, 1, LSM303C_CTRL_REG4_A_SIM_SPI_RW | LSM303C_CTRL_REG4_A_I2C_DISABLE);
	}

	// Read chip id
	regaddr = LSM303C_WHO_AM_I_A_REG;
	d = Read8(&regaddr, 1);
printf("Chip Id %x\r\n", d);

	if (d != LSM303C_WHO_AM_I_A_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);

	return true;
}


/**
 * @brief	Initialize magnetometer sensor.
 *
 * NOTE : Accelerometer must be initialized first prior to this one.
 *
 * @param 	Cfg		: Accelerometer configuration data
 * @param 	pIntrf	: Pointer to communication interface
 * @param 	pTimer	: Pointer to Timer use for time stamp
 *
 * @return	true - Success
 */
bool MagLsm303c::Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	return true;
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
int AccLsm303c::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_3WIRE);
	}

	int retval = Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_NORMAL);
	}

	return retval;
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
int AccLsm303c::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_3WIRE);
	}

	int retval = Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		((SPI*)vpIntrf)->Phy(SPIPHY_NORMAL);
	}

	return retval;
}

