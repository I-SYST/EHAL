/**-------------------------------------------------------------------------
@file	agm_icm20948.cpp

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

@author	Hoang Nguyen Hoan
@date	Nov. 5, 2018

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
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"

#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"

#include "idelay.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_icm20948.h"

static inv_icm20948_t icm_device;

static const uint8_t dmp3_image[] = {
#include "imu/icm20948_img.dmp3a.h"
};

bool AgmIcm20948::Init(uint32_t DevAddr, DeviceIntrf *pIntrf, Timer *pTimer)
{
	//if (vbInitialized)
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	uint16_t regaddr;
	uint8_t d;
	uint8_t userctrl = 0;//ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	uint8_t lpconfig = ICM20948_LP_CONFIG_ACCEL_CYCLE | ICM20948_LP_CONFIG_GYRO_CYCLE;

	Interface(pIntrf);
	DeviceAddess(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// in SPI mode, use i2c master mode to access Mag device (AK09916)
		userctrl |= ICM20948_USER_CTRL_I2C_IF_DIS | ICM20948_USER_CTRL_I2C_MST_EN;

		//lpconfig |= ICM20948_LP_CONFIG_I2C_MST_CYCLE;
	}

	vCurrBank = -1;

	// Read chip id
	regaddr = ICM20948_WHO_AM_I;
	d = Read8((uint8_t*)&regaddr, 2);

	if (d != ICM20948_WHO_AM_I_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	usDelay(500000);

	//inv_icm20948_initialize_lower_driver(&icm_device, SERIAL_INTERFACE_SPI, NULL, 0);//, dmp3_image_size);

	regaddr = ICM20948_USER_CTRL;
	Write8((uint8_t*)&regaddr, 2, userctrl);

	regaddr = ICM20948_PWR_MGMT_1;
	Write8((uint8_t*)&regaddr, 2, 1);

	regaddr = ICM20948_PWR_MGMT_2;
	Write8((uint8_t*)&regaddr, 2, 0x7f);

	// Init master I2C interface

	regaddr = ICM20948_FIFO_EN_1;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_EN_1_SLV_0_FIFO_EN);

	//regaddr = ICM20948_LP_CONFIG;
	//Write8((uint8_t*)&regaddr, 2, lpconfig);


	regaddr = ICM20948_I2C_MST_CTRL;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG;
	Write8((uint8_t*)&regaddr, 2, 0);

#if 0
	regaddr = ICM20948_AK09916_WIA1;
	uint8_t x[2];
	Read(AK09916_I2C_ADDR1, (uint8_t*)&regaddr, 1, x, 2);

	if (x[0] != ICM20948_AK09916_WIA1_ID)
	{
		return false;
	}
#endif

	vbInitialized  = true;

	return true;
}

bool AgmIcm20948::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint16_t regaddr;
	uint8_t d;

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	// ODR = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])

	// Find closes DIV value
	uint16_t div = 0;
	int diff = 1200;

	for (int i = 0; i < 0x1000; i++)
	{
		uint32_t f = 1125 / (1 + i);
		int df = CfgData.Freq > f ? CfgData.Freq - f : f - CfgData.Freq;

		if (df < diff)
		{
			diff = df;
			div = i;
			vSampFreq = f;
		}
	}

	regaddr = ICM20948_ACCEL_SMPLRT_DIV_1;
	Write8((uint8_t*)&regaddr, 2, div >> 8);

	regaddr = ICM20948_ACCEL_SMPLRT_DIV_2;
	Write8((uint8_t*)&regaddr, 2, div & 0xFF);

	Scale(CfgData.Scale);
	//LowPassFreq(vSampFreq / 2000);

	//regaddr = MPU9250_AG_INT_ENABLE;
	//Write8(&regaddr, 1, MPU9250_AG_INT_ENABLE_DMP_EN);

//	Reset();

	msDelay(100);

//	regaddr = MPU9250_AG_PWR_MGMT_1;
//	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_CYCLE);

	return true;
}

bool AgmIcm20948::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	uint16_t regaddr;
	uint8_t d = 0;
	uint8_t fchoice = 0;
	uint32_t f = CfgData.Freq >> 1;
#if 0
	if (f == 0)
	{
		fchoice = 1;
	}
	if (f < 10000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_5HZ;
	}
	else if (f < 20000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_10HZ;
	}
	else if (f < 30000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_20HZ;
	}
	else if (f < 60000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_41HZ;
	}
	else if (f < 150000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_92HZ;
	}
	else if (f < 220000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_184HZ;
	}
	else if (f < 40000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_250HZ;
	}
	else if (f < 400000)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_3600HZ;
	}
	else
	{
		// 8800Hz
		fchoice = 1;
	}

	regaddr = MPU9250_AG_CONFIG;
	Write8(&regaddr, 1, d);

	regaddr = MPU9250_AG_GYRO_CONFIG;
	Write8(&regaddr, 1, fchoice);

#endif
	Sensitivity(CfgData.Sensitivity);

	return true;
}

bool AgmIcm20948::Init(const MAGSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d[4];

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	msDelay(200);

	regaddr = ICM20948_AK09916_WIA1;
	Read(AK09916_I2C_ADDR1, &regaddr, 1, d, 2);

	if (d[0] != ICM20948_AK09916_WIA1_ID)
	{
		Read(AK09916_I2C_ADDR2, &regaddr, 1, d, 2);
		if (d[0] != ICM20948_AK09916_WIA1_ID)
			return false;
	}

	msDelay(1);
#if 0
	// Read ROM sensitivity adjustment values
	regaddr = MPU9250_MAG_CTRL1;
	d[0] = MPU9250_MAG_CTRL1_MODE_PWRDOWN;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 1);

	msDelay(1);

	d[0] = MPU9250_MAG_CTRL1_MODE_FUSEROM_ACCESS;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 1);

	msDelay(100);

	regaddr = MPU9250_MAG_ASAX;
	Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 3);

	vMagSenAdj[0] = (int16_t)d[0] - 128;
	vMagSenAdj[1] = (int16_t)d[1] - 128;
	vMagSenAdj[2] = (int16_t)d[2] - 128;

	// Transition out of reading ROM
	regaddr = MPU9250_MAG_CTRL1;
	d[0] = MPU9250_MAG_CTRL1_MODE_PWRDOWN;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 1);

	MagSensor::vPrecision = 14;
	vMagCtrl1Val = 0;
	MagSensor::vScale = 8190;

	if (CfgData.Precision >= 16)
	{
		MagSensor::vPrecision = 16;
		MagSensor::vScale = 32760;
		vMagCtrl1Val = MPU9250_MAG_CTRL1_BIT_16;
	}

	if (CfgData.OpMode == SENSOR_OPMODE_CONTINUOUS)
	{
		if (CfgData.Freq < 50000)
		{
			// Select 8Hz
			vMagCtrl1Val |= MPU9250_MAG_CTRL1_MODE_8HZ;
			MagSensor::Mode(CfgData.OpMode, 8000000);
		}
		else
		{
			// Select 100Hz
			vMagCtrl1Val |= MPU9250_MAG_CTRL1_MODE_100HZ;
			MagSensor::Mode(CfgData.OpMode, 100000000);
		}
	}
	else
	{
		vMagCtrl1Val |= MPU9250_MAG_CTRL1_MODE_SINGLE;
		MagSensor::Mode(CfgData.OpMode, 0);
	}

	msDelay(10);

	regaddr = MPU9250_MAG_CTRL1;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &vMagCtrl1Val, 1);
#endif
	return true;
}

bool AgmIcm20948::Enable()
{
#if 0
	uint8_t regaddr = MPU9250_AG_PWR_MGMT_1;

	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_CYCLE | MPU9250_AG_PWR_MGMT_1_GYRO_STANDBY |
			MPU9250_AG_PWR_MGMT_1_CLKSEL_INTERNAL);

	regaddr = MPU9250_AG_PWR_MGMT_2;

	// Enable Accel & Gyro
	Write8(&regaddr, 1,
			MPU9250_AG_PWR_MGMT_2_DIS_ZG |
			MPU9250_AG_PWR_MGMT_2_DIS_YG |
			MPU9250_AG_PWR_MGMT_2_DIS_XG);

	// Enable Mag
	//regaddr = MPU9250_MAG_CTRL1;
	//Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &vMagCtrl1Val, 1);
#endif
	return true;
}

void AgmIcm20948::Disable()
{
	uint16_t regaddr;
	uint8_t d;

	// Disable Accel & Gyro
	regaddr = ICM20948_PWR_MGMT_2;
	d = ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK | ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_USER_CTRL;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_LP_CONFIG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_PWR_MGMT_1;
	d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_SLEEP;// | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	Write8((uint8_t*)&regaddr, 2, d);


#if 0
	uint8_t regaddr = MPU9250_AG_PWR_MGMT_2;

Reset();
msDelay(2000);

	regaddr = MPU9250_AG_PWR_MGMT_1;
	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_PD_PTAT |
						MPU9250_AG_PWR_MGMT_1_GYRO_STANDBY);

	//return;

	regaddr = MPU9250_AG_USER_CTRL;
	Write8(&regaddr, 1, MPU9250_AG_USER_CTRL_I2C_MST_EN);

	// Disable Mag
	regaddr = MPU9250_MAG_CTRL1;
	uint8_t d = 0;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d, 1);

	// Disable Accel Gyro
	Write8(&regaddr, 1,
		 MPU9250_AG_PWR_MGMT_2_DIS_ZG | MPU9250_AG_PWR_MGMT_2_DIS_YG | MPU9250_AG_PWR_MGMT_2_DIS_XG |
		 MPU9250_AG_PWR_MGMT_2_DIS_ZA | MPU9250_AG_PWR_MGMT_2_DIS_YA | MPU9250_AG_PWR_MGMT_2_DIS_XA);

	regaddr = MPU9250_AG_PWR_MGMT_1;
	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_PD_PTAT |
						MPU9250_AG_PWR_MGMT_1_GYRO_STANDBY);
#endif
}

void AgmIcm20948::Reset()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_1;

	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_DEVICE_RESET);
}

bool AgmIcm20948::StartSampling()
{
	return true;
}

// Implement wake on motion
bool AgmIcm20948::WakeOnEvent(bool bEnable, int Threshold)
{
    uint16_t regaddr;

	if (bEnable == true)
	{
		Reset();

		msDelay(2000);
	}
	else
	{
//	    regaddr = MPU9250_AG_INT_ENABLE;
	    Write8((uint8_t*)&regaddr, 2, 0);

//	    regaddr = MPU9250_AG_PWR_MGMT_1;
		Write8((uint8_t*)&regaddr, 2, 0);
	}

	return true;
}

// Accel low pass frequency
uint32_t AgmIcm20948::LowPassFreq(uint32_t Freq)
{
	return AccelSensor::LowPassFreq();
}

// Accel scale
uint16_t AgmIcm20948::Scale(uint16_t Value)
{
	return AccelSensor::Scale();
}

// Gyro scale
uint32_t AgmIcm20948::Sensitivity(uint32_t Value)
{

	return GyroSensor::Sensitivity();
}

bool AgmIcm20948::UpdateData()
{
#if 0
	uint8_t regaddr = MPU9250_AG_FIFO_COUNT_H;//MPU9250_AG_ACCEL_XOUT_H;
	int8_t d[20];
	int32_t val;

	Read(&regaddr, 1, (uint8_t*)d, 2);
	val = ((d[0] & 0xF) << 8) | d[1];

	//printf("%d\r\n", val);

	if (val > 0)
	{
		int cnt = min(val, 18);
		regaddr = MPU9250_AG_FIFO_R_W;
	//	Read(&regaddr, 1, d, cnt);
	}

	vSampleCnt++;

	if (vpTimer)
	{
		vSampleTime = vpTimer->uSecond();
	}

	regaddr = MPU9250_AG_ACCEL_XOUT_H;
	Read(&regaddr, 1, (uint8_t*)d, 6);

	int32_t scale =  AccelSensor::Scale();
	val = (((((int32_t)d[0] << 8) | d[1]) * scale) << 8L) / 0x7FFF;
	AccelSensor::vData.X = val;
	val = (((((int32_t)d[2] << 8) | d[3]) * scale) << 8L) / 0x7FFF;
	AccelSensor::vData.Y = val;
	val = (((((int32_t)d[4] << 8) | d[5]) * scale) << 8L) / 0x7FFF;
	AccelSensor::vData.Z = val;
	AccelSensor::vData.Timestamp = vSampleTime;

	regaddr = MPU9250_AG_GYRO_XOUT_H;

	Read(&regaddr, 1, (uint8_t*)d, 6);

	val = ((((int16_t)d[0] << 8) | d[1]) << 8) / GyroSensor::vSensitivity;
	GyroSensor::vData.X = val;
	val = ((((int16_t)d[2] << 8) | d[3]) << 8) / GyroSensor::vSensitivity;
	GyroSensor::vData.Y = val;
	val = ((((int32_t)d[4] << 8) | d[5]) << 8L) / GyroSensor::vSensitivity;
	GyroSensor::vData.Z = val;
	GyroSensor::vData.Timestamp = vSampleTime;

	regaddr = MPU9250_MAG_ST1;
	Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, (uint8_t*)d, 8);

	if (d[14] & MPU9250_MAG_ST1_DRDY)
	{
		val = (((int16_t)d[0]) << 8L) | d[1];
		val += (val * vMagSenAdj[0]) >> 8L;
		MagSensor::vData.X = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vScale);

		val = (((int16_t)d[2]) << 8) | d[3];
		val += (val * vMagSenAdj[1]) >> 8L;
		MagSensor::vData.Y = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vScale);

		val = (((int16_t)d[4]) << 8) | d[5];
		val += (val * vMagSenAdj[2]) >> 8L;
		MagSensor::vData.Z = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vScale);

		MagSensor::vData.Timestamp = vSampleTime;
	}
#endif
	return true;
}

bool AgmIcm20948::Read(ACCELSENSOR_DATA &Data)
{
	Data = AccelSensor::vData;

	return true;
}

bool AgmIcm20948::Read(GYROSENSOR_DATA &Data)
{
	Data = GyroSensor::vData;

	return true;
}

bool AgmIcm20948::Read(MAGSENSOR_DATA &Data)
{
	Data = MagSensor::vData;

	return true;
}

int AgmIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(pCmdAddr[1]);
		CmdAddrLen--;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}


int AgmIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(pCmdAddr[1]);
		CmdAddrLen--;
	}

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

int AgmIcm20948::Read(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr;
		uint8_t userctrl;
		uint8_t lpconfig;

		regaddr = ICM20948_USER_CTRL;
		//Write8(&regaddr, 1, ICM20948_USER_CTRL_I2C_IF_DIS | ICM20948_USER_CTRL_I2C_MST_EN);
		userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;

		regaddr = ICM20948_LP_CONFIG;
		lpconfig = Read8((uint8_t*)&regaddr, 2);

		//Write8((uint8_t*)&regaddr, 2, 0);

#if 1
		uint8_t d[8];


//		regaddr = ICM20948_I2C_SLV0_CTRL;
//		Write8(&regaddr, 1, 0);

		regaddr = ICM20948_I2C_SLV0_ADDR;

		//d[0] = ICM20948_I2C_SLV0_ADDR;
		d[0] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD;
		d[1] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

			SelectBank(3);

			d[2] = ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (cnt & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK);

			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = ICM20948_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			//usDelay(500 + (cnt << 4));
			msDelay(100);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

			regaddr = ICM20948_EXT_SLV_SENS_DATA_00;
			cnt = Read((uint8_t*)&regaddr, 1, pBuff, cnt);
			if (cnt <= 0)
				break;

			pBuff += cnt;
			BuffLen -= cnt;
			retval += cnt;
		}
		regaddr = ICM20948_LP_CONFIG;

		//Write8((uint8_t*)&regaddr, 2, lpconfig);
#else
		regaddr = ICM20948_I2C_SLV0_ADDR;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);

		regaddr = ICM20948_I2C_SLV0_REG;
		Write8((uint8_t*)&regaddr, 2, *pCmdAddr);

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (1 & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK));

		regaddr = ICM20948_USER_CTRL;
		Write8((uint8_t*)&regaddr, 2, userctrl);

		msDelay(100);

		Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);
		regaddr = ICM20948_EXT_SLV_SENS_DATA_00;
		int cnt = Read((uint8_t*)&regaddr, 2, pBuff, 1);

		BuffLen -= cnt;

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, 0);


#endif
	}
	else
	{
		retval = vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	return retval;
}

int AgmIcm20948::Write(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr;
		uint8_t d[8];

		d[0] = ICM20948_I2C_SLV0_ADDR & 0xff;
		d[1] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_WR;
		d[2] = *pCmdAddr;
		d[3] = DataLen & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK;

		while (DataLen > 0)
		{
			regaddr = ICM20948_I2C_SLV0_DO;
			Write8((uint8_t*)&regaddr, 2, *pData);

			Write(d, 4, NULL, 0);

			d[2]++;
			pData++;
			DataLen--;
			retval++;
		}
	}
	else
	{
		retval = vpIntrf->Write(DevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	return retval;
}

bool AgmIcm20948::SelectBank(uint8_t BankNo)
{
	if (BankNo > 3 || vCurrBank == BankNo)
		return false;

	vCurrBank = BankNo;

	uint8_t regaddr = ICM20948_REG_BANK_SEL;

	return Write8(&regaddr, 1, (BankNo << ICM20948_REG_BANK_SEL_USER_BANK_BITPOS) & ICM20948_REG_BANK_SEL_USER_BANK_MASK);
}

void AgmIcm20948::IntHandler()
{
	uint16_t regaddr = 0;//MPU9250_AG_INT_STATUS;
	uint8_t d;

	d = Read8((uint8_t*)&regaddr, 2);
//	if (d & MPU9250_AG_INT_STATUS_RAW_DATA_RDY_INT)
	{
		UpdateData();
	}
}


