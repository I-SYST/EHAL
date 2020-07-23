/**-------------------------------------------------------------------------
@file	am_lsm303agr.cpp

@brief	Implementation of ST LSM303AGR accel, mag sensor

This device is a combination of 2 independent entities.  Therefore the
implementation consists of 2 independent objects.


@author	Hoang Nguyen Hoan
@date	Sept. 18, 2019

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
#include "coredev/spi.h"
#include "sensors/am_lsm303agr.h"

bool AccelLsm303agr::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	uint8_t regaddr;
	uint8_t d;

	Interface(pIntrf);
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI || DevAddr != LSM303AGR_ACCEL_I2C_DEVADDR)
	{
		regaddr = LSM303AGR_CTRL_REG4_A_REG;
		Write8(&regaddr, 1, LSM303AGR_CTRL_REG4_A_SPI_3WIRE_ENABLE);
	}

	// Read chip id
	regaddr = LSM303AGR_WHO_AM_I_A_REG;
	d = Read8(&regaddr, 1);
	d = Read8(&regaddr, 1);
	d = Read8(&regaddr, 1);

	if (d != LSM303AGR_WHO_AM_I_A_ID)
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
bool AccelLsm303agr::Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbValid == false)
	{
		if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		{
			return false;
		}
	}

	uint8_t regaddr;
	uint8_t d;
	uint8_t userctrl = 0;
	uint8_t mst = 0;

	// High res 12bits default
	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	d = Read8(&regaddr, 1) | LSM303AGR_CTRL_REG4_A_HR;
	Write8(&regaddr, 1, d);

	vRShift = 4;
	AccelSensor::vData.Range = AccelSensor::Range((1<<11)-1);

	Scale(Cfg.Scale);
	uint32_t f = SamplingFrequency(Cfg.Freq);

	vbIntEn = Cfg.bInter;

	if (Cfg.bInter)
	{
		vIntHandler = Cfg.IntHandler;

		regaddr = LSM303AGR_CTRL_REG3_A_REG;
		Write8(&regaddr, 1, LSM303AGR_CTRL_REG3_A_I1_OVERRUN | LSM303AGR_CTRL_REG3_A_I1_WTM);

		regaddr = LSM303AGR_CTRL_REG5_A_REG;
		Write8(&regaddr, 1, LSM303AGR_CTRL_REG5_A_LIR_INT1 | LSM303AGR_CTRL_REG5_A_D4D_INT1);

		regaddr = LSM303AGR_INT1_CFG_A_REG;
		Write8(&regaddr, 1, 0x3F);

		regaddr = LSM303AGR_INT1_DURATION_A_REG;
		Write8(&regaddr, 1, 1000 / f);

		regaddr = LSM303AGR_INT1_THS_A_REG;
		Write8(&regaddr, 1, 1);
	}
	else
	{
		regaddr = LSM303AGR_INT1_CFG_A_REG;
		Write8(&regaddr, 1, 0);
	}

	msDelay(1);

	// Flush FIFO
	regaddr = LSM303AGR_FIFO_SRC_REG;
	d = Read8(&regaddr, 1) & LSM303AGR_FIFO_SRC_FSS_MASK;

	uint8_t b[32 * 6];

	regaddr = LSM303AGR_OUT_X_L_A_REG | 0x40;
	if (d > 0)
	{
		Read(&regaddr, 1, b, d * 6);
	}

	Enable();


	return true;
}

bool AccelLsm303agr::Init(const TEMPSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (vbValid == false)
	{
		if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		{
			return false;
		}
	}

	uint8_t regaddr = LSM303AGR_TEMP_CFG_REG_A_REG;
	Write8(&regaddr, 1, LSM303AGR_TEMP_CFG_REG_A_ENABLE);

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	uint8_t d = Read8(&regaddr, 1) | LSM303AGR_CTRL_REG4_A_BDU;
	Write8(&regaddr, 1, d);

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	d = Read8(&regaddr, 1);


	if (d & LSM303AGR_CTRL_REG1_A_LPEN)
	{
		TempSensor::Range(127);
	}
	else
	{
		TempSensor::Range(511);
	}

	return true;
}

uint32_t AccelLsm303agr::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr;
	uint32_t r = (1 << 11) - 1;
	uint32_t tr = 511;
	uint32_t f = 0;
	uint8_t ctrl = 0;
	uint8_t ctrl4 = 0;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	ctrl = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG1_A_ODR_MASK | LSM303AGR_CTRL_REG1_A_LPEN);

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	ctrl4 = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG4_A_HR);


	if (Freq < 2000)
	{
		f = 1000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 15000)
	{
		f = 10000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_10HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 35000)
	{
		f = 25000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_25HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 70000)
	{
		f = 50000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_50HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 150000)
	{
		f = 100000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_100HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 300000)
	{
		f = 200000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_200HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 700000)
	{
		f = 400000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_400HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 1400000)
	{
		f = 1344000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1344_5376HZ;
		ctrl4 |= LSM303AGR_CTRL_REG4_A_HR;
	}
	else if (Freq < 2500000)
	{
		f = 1620000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1620HZ | LSM303AGR_CTRL_REG1_A_LPEN;
		tr = r = 127;
		vRShift = 8;

	}
	else
	{
		f = 5376000;
		ctrl |= LSM303AGR_CTRL_REG1_A_ODR_1344_5376HZ | LSM303AGR_CTRL_REG1_A_LPEN;
		tr = r = 127;
		vRShift = 8;

	}

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	Write8(&regaddr, 1, ctrl4);

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	Write8(&regaddr, 1, ctrl);

	AccelSensor::Range(r);
	AccelSensor::vData.Range = r;
	TempSensor::Range(tr);

	return AccelSensor::SamplingFrequency(f);
}

uint8_t AccelLsm303agr::Scale(uint8_t Value)
{
	uint8_t regaddr;
	uint8_t d;
	uint32_t f = 0;
	uint8_t ctrl = 0;
	uint8_t g = 0;

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	ctrl = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG4_A_FS_MASK);

	if (Value < 3)
	{
		g = 2;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_2G;
	}
	else if (Value < 6)
	{
		g = 4;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_4G;
	}
	else if (Value < 12)
	{
		g = 8;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_8G;
	}
	else
	{
		g = 16;
		ctrl |= LSM303AGR_CTRL_REG4_A_FS_16G;
	}

	Write8(&regaddr, 1, ctrl);
	AccelSensor::vData.Scale = g;

	return AccelSensor::Scale(g);
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
uint32_t AccelLsm303agr::FilterFreq(uint32_t Freq)
{
	uint8_t regaddr = LSM303AGR_CTRL_REG2_A_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG2_A_HPM_MASK | LSM303AGR_CTRL_REG2_A_FDS);

	if (Freq != 0)
	{
		d |= LSM303AGR_CTRL_REG2_A_FDS;
	}

	Write8(&regaddr, 1, d);

	regaddr = LSM303AGR_REF_DATACAPTURE_A_REG;
	d = Read8(&regaddr, 1);

	return AccelSensor::FilterFreq(Freq);
}

bool AccelLsm303agr::Enable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d | LSM303AGR_CTRL_REG1_A_XEN | LSM303AGR_CTRL_REG1_A_YEN | LSM303AGR_CTRL_REG1_A_ZEN);

	regaddr = LSM303AGR_CTRL_REG5_A_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, d | LSM303AGR_CTRL_REG5_A_FIFO_EN);

	regaddr = LSM303AGR_FIFO_CTRL_REG;
	d = Read8(&regaddr, 1) & ~(LSM303AGR_FIFO_CTRL_FTH_MASK | LSM303AGR_FIFO_CTRL_FM_MASK);

	// stop fifo first
	Write8(&regaddr, 1, d);

	if (vbIntEn)
	{
		d |= LSM303AGR_FIFO_CTRL_FM_STREAM | 1;
		Write8(&regaddr, 1, d);
	}
	else
	{
		d |= LSM303AGR_FIFO_CTRL_FM_STREAM | 6;
		Write8(&regaddr, 1, d);
	}

	regaddr = LSM303AGR_CTRL_REG4_A_REG;
	d = Read8(&regaddr, 1) | LSM303AGR_CTRL_REG4_A_BDU;
	Write8(&regaddr, 1, d);

	return true;
}

void AccelLsm303agr::Disable()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	d = Read8(&regaddr, 1) & ~(LSM303AGR_CTRL_REG1_A_XEN | LSM303AGR_CTRL_REG1_A_YEN | LSM303AGR_CTRL_REG1_A_ZEN);
	Write8(&regaddr, 1, d);

	regaddr = LSM303AGR_FIFO_CTRL_REG;
	d = Read8(&regaddr, 1) & ~(LSM303AGR_FIFO_CTRL_FTH_MASK | LSM303AGR_FIFO_CTRL_FM_MASK);
	Write8(&regaddr, 1, d);

	regaddr = LSM303AGR_CTRL_REG5_A_REG;
	d = Read8(&regaddr, 1) & ~LSM303AGR_CTRL_REG5_A_FIFO_EN;
	Write8(&regaddr, 1, d);
}

void AccelLsm303agr::Reset()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LSM303AGR_CTRL_REG2_A_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LSM303AGR_CTRL_REG3_A_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LSM303AGR_FIFO_CTRL_REG;
	Write8(&regaddr, 1, 0);

	regaddr = LSM303AGR_CTRL_REG5_A_REG;
	Write8(&regaddr, 1, LSM303AGR_CTRL_REG5_A_BOOT);
	msDelay(1);
	Write8(&regaddr, 1, 0);

	regaddr = LSM303AGR_REF_DATACAPTURE_A_REG;
	d = Read8(&regaddr, 1);

	regaddr = LSM303AGR_INT1_SRC_A_REG;
	d = Read8(&regaddr, 1);
}

void AccelLsm303agr::PowerOff()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG1_A_REG;
	Write8(&regaddr, 1, 0);
}

bool AccelLsm303agr::UpdateData()
{
	uint8_t regaddr = LSM303AGR_STATUS_REG_A_REG;
	uint8_t d = Read8(&regaddr, 1);
	uint8_t fstatus;
	uint32_t ts = 0;

	if (vpTimer)
	{
		ts = vpTimer->uSecond();
	}

	regaddr = LSM303AGR_FIFO_SRC_REG;
	fstatus = Read8(&regaddr, 1);

	bool avail = false;

	if ((fstatus & LSM303AGR_FIFO_SRC_FSS_MASK) & vbIntEn)
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
		AccelSensor::vData.Timestamp = ts;

		for (int i = 0; i < (fstatus & 0x1f); i++)
		{
			regaddr = LSM303AGR_OUT_X_L_A_REG | 0x40;
			Read(&regaddr, 1, (uint8_t*)AccelSensor::vData.Val, 6);
		}

		// Right justify to get correct value
		for (int i = 0; i < 3; i++)
		{
			AccelSensor::vData.Val[i] >>= vRShift;
		}
	}

	regaddr = LSM303AGR_STATUS_REG_AUX_A_REG;
	d = Read8(&regaddr, 1);

	if (d & LSM303AGR_STATUS_REG_AUX_A_TDA)
	{
		// Temperature data avail
		TempSensor::vData.Timestamp = ts;

		regaddr = LSM303AGR_OUT_TEMP_L_A_REG | 0x40;
		uint16_t t = 0;
		Read(&regaddr, 1, (uint8_t*)&t, 2);

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
int AccelLsm303agr::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x3F;
		if (BuffLen > 1)
		{
			*pCmdAddr |= 0x40;
		}
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
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
int AccelLsm303agr::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x3F;
		if (DataLen > 1)
		{
			*pCmdAddr |= 0x40;
		}
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

void AccelLsm303agr::IntHandler()
{
	uint8_t regaddr = LSM303AGR_STATUS_REG_A_REG;
	uint8_t status = Read8(&regaddr, 1);

	regaddr = LSM303AGR_FIFO_SRC_REG;
	uint8_t fstatus = Read8(&regaddr, 1);

	regaddr = LSM303AGR_INT1_SRC_A_REG;
	uint8_t isrc1 = Read8(&regaddr, 1);

	regaddr = LSM303AGR_INT2_SRC_A_REG;
	uint8_t isrc2 = Read8(&regaddr, 1);

	if ((fstatus & LSM303AGR_FIFO_SRC_WTM) || isrc1 || isrc2)
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

	if (fstatus & LSM303AGR_FIFO_SRC_OVRN)
	{
		// Overrun, clear fifo
		regaddr = LSM303AGR_FIFO_CTRL_REG;
		uint8_t d = Read8(&regaddr, 1) & ~LSM303AGR_FIFO_CTRL_FM_MASK;
		Write8(&regaddr, 1, d);
		d |= LSM303AGR_FIFO_CTRL_FM_STREAM_TO_FIFO;
		Write8(&regaddr, 1, d);
	}
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
bool MagLsm303agr::Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	uint8_t regaddr;
	uint8_t d;

	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Reset();

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI || Cfg.DevAddr != LSM303AGR_MAG_I2C_DEVADDR)
	{
		regaddr = LSM303AGR_CFG_REG_C_M_REG;
		Write8(&regaddr, 1, LSM303AGR_CFG_REG_C_M_I2C_DIS);
	}

	// Read chip id
	regaddr = LSM303AGR_WHO_AM_I_M_REG;
	d = Read8(&regaddr, 1);

	if (d != LSM303AGR_WHO_AM_I_M_ID)
	{
		return false;
	}

	DeviceID(d);
	Valid(true);

	vSensitivity[0] = vSensitivity[1] = vSensitivity[2] = LSM303AGR_MAG_SENSITTIVITY;

	ClearCalibration();

	SamplingFrequency(Cfg.Freq);

	regaddr = LSM303AGR_CFG_REG_B_M_REG;
	Write8(&regaddr, 1,  LSM303AGR_CFG_REG_B_M_LPF | LSM303AGR_CFG_REG_B_M_OFF_CANC);

	regaddr = LSM303AGR_OFFSET_X_REG_L_M_REG;
	MagSensor::Read(&regaddr, 1, (uint8_t*)vOffset, 6);

	regaddr = LSM303AGR_INT_CTRL_REG_M_REG;
	vbIntEn = Cfg.bInter;

	if (vbIntEn)
	{
		Write8(&regaddr, 1, LSM303AGR_INT_CTRL_REG_M_IEN | LSM303AGR_INT_CTRL_REG_M_IEL |
							LSM303AGR_INT_CTRL_REG_M_IEA | LSM303AGR_INT_CTRL_REG_M_ZIEN |
							LSM303AGR_INT_CTRL_REG_M_YIEN | LSM303AGR_INT_CTRL_REG_M_XIEN);
		regaddr = LSM303AGR_INT_THS_L_REG_M_REG;
		Write16(&regaddr, 1, 1);
	}
	else
	{
		Write8(&regaddr, 1, 0);
	}

	Enable();

	return true;
}

uint32_t MagLsm303agr::SamplingFrequency(uint32_t Freq)
{
	uint8_t regaddr;
	uint8_t d;
	uint32_t f = 0;
	uint8_t cfg = 3;

	cfg = Read8(&regaddr, 1);

	if (Freq < 15)
	{
		f = 10000;
		cfg |= LSM303AGR_CTRL_REG_A_M_ODR_10HZ;
	}
	else if (Freq < 30)
	{
		f = 20000;
		cfg |= LSM303AGR_CTRL_REG_A_M_ODR_20HZ;
	}
	else if (Freq < 70)
	{
		f = 50000;
		cfg |= LSM303AGR_CTRL_REG_A_M_ODR_50HZ;
	}
	else if (Freq < 125)
	{
		f = 100000;
		cfg |= LSM303AGR_CTRL_REG_A_M_ODR_100HZ;
	}
	else
	{
		f = 150000;
		cfg |= LSM303AGR_CTRL_REG_A_M_ODR_100HZ | LSM303AGR_CTRL_REG_A_M_LP;

	}

	Write8(&regaddr, 1, cfg);

	return MagSensor::SamplingFrequency(f);
}

bool MagLsm303agr::Enable()
{
	uint8_t regaddr = LSM303AGR_CTRL_REG_A_M_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LSM303AGR_CTRL_REG_A_M_MD_MASK;

	Write8(&regaddr, 1, d);

	return true;
}

void MagLsm303agr::Disable()
{
	uint8_t regaddr = LSM303AGR_CTRL_REG_A_M_REG;
	uint8_t d = Read8(&regaddr, 1) & ~LSM303AGR_CTRL_REG_A_M_MD_MASK;

	Write8(&regaddr, 1, d | LSM303AGR_CTRL_REG_A_M_MD_IDLE);
}

void MagLsm303agr::Reset()
{
	uint8_t regaddr;
	uint8_t d;

	regaddr = LSM303AGR_CTRL_REG_A_M_REG;
	d = Read8(&regaddr, 1);
	Write8(&regaddr, 1, LSM303AGR_CTRL_REG_A_M_SOFT_RST | d);

	do {
		d = Read8(&regaddr, 1);
	} while (d & LSM303AGR_CTRL_REG_A_M_SOFT_RST);
}

bool MagLsm303agr::UpdateData()
{
	uint8_t regaddr = LSM303AGR_STATUS_REG_M_REG;
	uint8_t status = Read8(&regaddr, 1);

	if (status)
	{
		if (vpTimer)
		{
			vData.Timestamp = vpTimer->uSecond();
		}

		regaddr = LSM303AGR_OUTX_L_REG_M_REG;
		MagSensor::Read(&regaddr, 1, (uint8_t*)vData.Val, 6);

		vData.X += vOffset[0];
		vData.Y += vOffset[1];
		vData.Z += vOffset[2];
	}

	return true;
}

void MagLsm303agr::IntHandler()
{
	uint8_t regaddr = LSM303AGR_INT_SOURCE_REG_M_REG;
	uint8_t status = Read8(&regaddr, 1);

	if (status)
	{
		UpdateData();
	}
}

