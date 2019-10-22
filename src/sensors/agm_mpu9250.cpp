/**-------------------------------------------------------------------------
@file	agm_mpu9250.cpp

@brief	Implementation of TDK MPU-9250 accel, gyro, mag sensor

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
#include "istddef.h"
#include "convutil.h"
#include "idelay.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_mpu9250.h"
#include "iopinctrl.h"

#define MPU9250_ACCEL_IDX		0
#define MPU9250_GYRO_IDX		1
#define MPU9250_MAG_IDX			2

bool AgmMpu9250::Init(uint32_t DevAddr, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (vbInitialized)
		return true;;

	if (pIntrf == NULL)
		return false;

	uint8_t regaddr;
	uint8_t d;
	uint8_t userctrl = 0;///*MPU9250_AG_USER_CTRL_FIFO_EN | MPU9250_AG_USER_CTRL_DMP_EN |*/ MPU9250_AG_USER_CTRL_I2C_MST_EN;
	uint8_t mst = 0;

	Interface(pIntrf);
	DeviceAddress(DevAddr);
	vbSensorEnabled[0] = vbSensorEnabled[1] = vbSensorEnabled[2] = false;

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	if (pIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// in SPI mode, use i2c master mode to access Mag device (AK8963C)
		userctrl |= MPU9250_AG_USER_CTRL_I2C_MST_EN | MPU9250_AG_USER_CTRL_I2C_IF_DIS;// | MPU9250_AG_USER_CTRL_FIFO_EN;
		mst = MPU9250_AG_I2C_MST_CTRL_WAIT_FOR_ES | 13;
	}

	int rty = 5;

	// The MPU-9250 has an issue that often unresponsive when rebooting while MAG is streaming
	// A few reset/retry would require.  Have not found workaround yet.
	do {
		regaddr = MPU9250_AG_PWR_MGMT_1;
		Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_H_RESET);

		msDelay(100);

		// Init master I2C interface
		regaddr = MPU9250_AG_I2C_MST_CTRL;
		Write8(&regaddr, 1, mst);

		regaddr = MPU9250_AG_USER_CTRL;
		Write8(&regaddr, 1, userctrl | MPU9250_AG_USER_CTRL_I2C_MST_RST);


		msDelay(10);

		regaddr = MPU9250_MAG_CTRL2;
		d = MPU9250_MAG_CTRL2_SRST;
		MagMpu9250::Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d, 1);

		msDelay(100);

		regaddr = MPU9250_MAG_WIA;
		MagMpu9250::Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d, 1);
		//printf("Mag %x\r\n", d);

		// Read chip id
		regaddr = MPU9250_AG_WHO_AM_I;
		d = Read8((uint8_t*)&regaddr, 1);
	} while (rty-- > 0 && d != MPU9250_AG_WHO_AM_I_ID);

	if (d != MPU9250_AG_WHO_AM_I_ID)
	{
		// At this point, it is better to power cycle
		return false;
	}

	//printf("rty = %d\r\n", rty);

	Reset();

	DeviceID(d);
	Valid(true);
	vbDmpEnabled = false;

	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	msDelay(100);

	// Disable all interrupt
	regaddr = MPU9250_AG_INT_ENABLE;
	Write8(&regaddr, 1, 0);

	//regaddr = MPU9250_AG_CONFIG;
	//Write8(&regaddr, 1, MPU9250_AG_CONFIG_FIFO_MODE_BLOCKING);

	vbInitialized = true;

	// Init master I2C interface
	regaddr = MPU9250_AG_USER_CTRL;
	Write8(&regaddr, 1, userctrl);

	regaddr = MPU9250_AG_I2C_MST_CTRL;
	Write8(&regaddr, 1, mst);

	// Enable FIFO

    // Undocumented register
	// shares 4kB of memory between the DMP and the FIFO. Since the
    // first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
	//regaddr = MPU9250_AG_ACCEL_CONFIG2;
	//Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG2_ACCEL_FCHOICE_B | MPU9250_AG_ACCEL_CONFIG2_FIFO_SIZE_1024);


	//regaddr = MPU9250_AG_FIFO_EN;
	//Write8(&regaddr, 1, MPU9250_AG_FIFO_EN_ACCEL |
	//		MPU9250_AG_FIFO_EN_GYRO_ZOUT | MPU9250_AG_FIFO_EN_GYRO_YOUT |
	//		MPU9250_AG_FIFO_EN_GYRO_XOUT);

	//regaddr = MPU9250_MAG_WIA;
	//Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d, 1);

	//printf("WIA %x\r\n", d);

	return true;
}

bool AccelMpu9250::Init(const ACCELSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d;

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	//
	// NOTE : ODR setting is valid only in low power mode by setting
	// MPU9250_AG_PWR_MGMT_1_CYCLE in register MPU9250_AG_PWR_MGMT_1
	// in this mode is only acce is activel, gyro & mag are disabled.
	//
	// Accel should be the first to be initialized, though low power
	// mode is set by default. This mode is disabled if gyro or mag
	// is activated
	//
	regaddr = MPU9250_AG_PWR_MGMT_1;
	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_CYCLE);

	msDelay(1);	// Min delays required for mode change to sync before config can be set

	regaddr = MPU9250_AG_LP_ACCEL_ODR;

	if (CfgData.Freq < 400)
	{
		Write8(&regaddr, 1, 0);
		AccelSensor::vSampFreq = 240;	// 0.24 Hz
	}
	else if (CfgData.Freq < 900)
	{
		Write8(&regaddr, 1, 1);
		AccelSensor::vSampFreq = 490;	// 0.49 Hz
	}
	else if (CfgData.Freq < 1500)
	{
		Write8(&regaddr, 1, 2);
		AccelSensor::vSampFreq = 980;	// 0.98 Hz
	}
	else if (CfgData.Freq < 2500)
	{
		Write8(&regaddr, 1, 3);
		AccelSensor::vSampFreq = 1950;	// 1.95 Hz
	}
	else if (CfgData.Freq < 3500)
	{
		Write8(&regaddr, 1, 4);
		AccelSensor::vSampFreq = 3910;	// 3.91 Hz
	}
	else if (CfgData.Freq < 10000)
	{
		Write8(&regaddr, 1, 5);
		AccelSensor::vSampFreq = 7810;	// 7.81 Hz
	}
	else if (CfgData.Freq < 20000)
	{
		Write8(&regaddr, 1, 6);
		AccelSensor::vSampFreq = 15630;	// 15.63 Hz
	}
	else if (CfgData.Freq < 50000)
	{
		Write8(&regaddr, 1, 7);
		AccelSensor::vSampFreq = 31250;	// 31.25 Hz
	}
	else if (CfgData.Freq < 100000)
	{
		Write8(&regaddr, 1, 8);
		AccelSensor::vSampFreq = 62500;	// 62.5 Hz
	}
	else if (CfgData.Freq < 200000)
	{
		Write8(&regaddr, 1, 9);
		AccelSensor::vSampFreq = 125000;	// 125 Hz
	}
	else if (CfgData.Freq < 500000)
	{
		Write8(&regaddr, 1, 10);
		AccelSensor::vSampFreq = 250000;	// 250 Hz
	}
	else
	{
		Write8(&regaddr, 1, 11);
		AccelSensor::vSampFreq = 500000;	// 500 Hz
	}

	AccelSensor::Range(MPU9250_AG_ADC_RANGE);
	Scale(CfgData.Scale);
	FilterFreq(CfgData.FltrFreq);

	//regaddr = MPU9250_AG_ACCEL_CONFIG2;
	//Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG2_ACCEL_FCHOICE_B);

	msDelay(100);

	d = 0;

	if (CfgData.IntPol == DEVINTR_POL_LOW)
	{
		regaddr = MPU9250_AG_INT_ENABLE;
		d |= MPU9250_AG_INT_PIN_CFG_ACTL | MPU9250_AG_INT_PIN_CFG_ACTL_FSYNC;
		Write8(&regaddr, 1, d);
	}

	regaddr = MPU9250_AG_INT_ENABLE;
	Write8(&regaddr, 1, MPU9250_AG_INT_ENABLE_RAW_RDY_EN);

	//vbSensorEnabled[MPU9250_ACCEL_IDX] = true;

	AccelSensor::Type(SENSOR_TYPE_ACCEL);

	return true;
}

bool GyroMpu9250::Init(const GYROSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	uint8_t regaddr;
	uint8_t d = 0;
	uint8_t fchoice = 0;
	uint32_t f = CfgData.FltrFreq;////max(CfgData.Freq, vSampFreq);
	GyroSensor::vSampFreq = CfgData.Freq;

	uint16_t smplrt = 1000000 / GyroSensor::vSampFreq;
	uint8_t intval;

	// Disable interrupt
	regaddr = MPU9250_AG_INT_ENABLE;
	intval = Read8(&regaddr, 1);
	Write8(&regaddr, 1, 0);

	// Read interrupt status to clear
	regaddr = MPU9250_AG_INT_STATUS;
	d = Read8(&regaddr, 1);

	// Enable full power
	regaddr = MPU9250_AG_PWR_MGMT_1;
	d = Read8(&regaddr, 1);
	d  &= ~MPU9250_AG_PWR_MGMT_1_CYCLE;
	Write8(&regaddr, 1, d);

	msDelay(1);	// Min delays required for mode change to sync before config can be set

	d = 0;

	if (f == 0)
	{
		fchoice = 1;
		smplrt = 32000000 / GyroSensor::vSampFreq;
	}
	else if (f < 6)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_5HZ;
	}
	else if (f < 16)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_10HZ;
	}
	else if (f < 30)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_20HZ;
	}
	else if (f < 60)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_41HZ;
	}
	else if (f < 150)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_92HZ;
	}
	else if (f < 220)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_184HZ;
	}
	else if (f < 300)
	{
		d = MPU9250_AG_CONFIG_DLPF_CFG_250HZ;
		smplrt = 8000000 / GyroSensor::vSampFreq;
	}
	else if (f < 5000)
	{
		// 3600 Hz
		d = MPU9250_AG_CONFIG_DLPF_CFG_3600HZ;
		smplrt = 8000000 / GyroSensor::vSampFreq;
	}
	else
	{
		// 8800Hz
		fchoice = 1;
		smplrt = 32000000 / GyroSensor::vSampFreq;
	}

	regaddr = MPU9250_AG_CONFIG;
	Write8(&regaddr, 1, d);// | MPU9250_AG_CONFIG_FIFO_MODE_BLOCKING);

	regaddr = MPU9250_AG_GYRO_CONFIG;
	Write8(&regaddr, 1, fchoice);

	regaddr = MPU9250_AG_SMPLRT_DIV;
	Write8(&regaddr, 1, smplrt - 1);

	Sensitivity(CfgData.Sensitivity);


	//vbSensorEnabled[MPU9250_GYRO_IDX] = true;
	GyroSensor::Type(SENSOR_TYPE_GYRO);

	//regaddr = MPU9250_AG_FIFO_EN;
	//d = Read8(&regaddr, 1);

	//printf("FIFO EN = %x\r\n", d);
	regaddr = MPU9250_AG_INT_ENABLE;
	intval |= MPU9250_AG_INT_ENABLE_RAW_RDY_EN;
	Write8(&regaddr, 1, intval);

	return true;
}

bool MagMpu9250::Init(const MAGSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	uint8_t regaddr;
	uint8_t d[4];
	uint8_t intval;

	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	// Disable interrupt
	regaddr = MPU9250_AG_INT_ENABLE;
	intval = Read8(&regaddr, 1);
	Write8(&regaddr, 1, 0);

	msDelay(5);

	// Read interrupt status to clear
	regaddr = MPU9250_AG_INT_STATUS;
	d[0] = Read8(&regaddr, 1);

	// Enable full power
	regaddr = MPU9250_AG_PWR_MGMT_1;
	d[0] = Read8(&regaddr, 1);
	d[0]  &= ~MPU9250_AG_PWR_MGMT_1_CYCLE;
	Write8(&regaddr, 1, d[0]);

	msDelay(10);	// Min delays required for mode change to sync before config can be set

	int rty = 10;

	do {
		regaddr = MPU9250_MAG_CTRL2;
		d[0] = MPU9250_MAG_CTRL2_SRST;
		Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 1);

		msDelay(100);

		regaddr = MPU9250_MAG_WIA;
		Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 2);
		//printf("Mag %x\r\n", d[0]);

	} while (rty-- > 0 && d[0] != MPU9250_MAG_WIA_DEVICE_ID);

	//printf("r = %d\r\n", rty);

	//printf("WIA %x %x\r\n", d[0], d[1]);

	if (d[0] != MPU9250_MAG_WIA_DEVICE_ID)
	{
		regaddr = MPU9250_AG_INT_ENABLE;
		Write8(&regaddr, 1, intval);

		return false;
	}

	msDelay(1);

	// Read ROM sensitivity adjustment values
	regaddr = MPU9250_MAG_CTRL1;
	d[0] = MPU9250_MAG_CTRL1_MODE_PWRDOWN;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 1);

	msDelay(1);

	d[0] = MPU9250_MAG_CTRL1_MODE_FUSEROM_ACCESS;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 1);

	msDelay(10);

	regaddr = MPU9250_MAG_ASAX;
	Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 3);

	vMagSenAdj[0] = (int16_t)d[0] - 128;
	vMagSenAdj[1] = (int16_t)d[1] - 128;
	vMagSenAdj[2] = (int16_t)d[2] - 128;

	printf("Adj %x %x %x\r\n", d[0], d[1], d[2]);

	//msDelay(10);

	// Transition out of reading ROM
	regaddr = MPU9250_MAG_CTRL1;
	d[0] = MPU9250_MAG_CTRL1_MODE_PWRDOWN;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, d, 1);

	//msDelay(10);

	//MagSensor::vPrecision = 14;
	vMagCtrl1Val = 0;
	MagSensor::vRange = MPU9250_MAG_ADC_LOW_RANGE;

	if (CfgData.Precision == MAGSENSOR_PRECISION_HIGH)
	{
		//MagSensor::vPrecision = 16;
		MagSensor::vRange = MPU9250_MAG_ADC_HIGH_RANGE;
		vMagCtrl1Val = MPU9250_MAG_CTRL1_BIT_16;
	}

	MagSensor::vSensitivity[0] = MPU9250_MAG_MAX_FLUX_DENSITY / MagSensor::vRange;
	MagSensor::vSensitivity[1] = MPU9250_MAG_MAX_FLUX_DENSITY / MagSensor::vRange;
	MagSensor::vSensitivity[2] = MPU9250_MAG_MAX_FLUX_DENSITY / MagSensor::vRange;

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

//	msDelay(10);

	regaddr = MPU9250_MAG_CTRL1;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &vMagCtrl1Val, 1);

	msDelay(1);

	//vbSensorEnabled[MPU9250_MAG_IDX] = true;

	regaddr = MPU9250_AG_INT_ENABLE;
	intval |= MPU9250_AG_INT_ENABLE_RAW_RDY_EN;
	Write8(&regaddr, 1, intval);

	ClearCalibration();

	return true;
}

MAGSENSOR_PRECISION MagMpu9250::Precision(MAGSENSOR_PRECISION Val)
{
	if (Val == vPrecision)
	{
		return vPrecision;
	}

	uint16_t sen[3];

	if (MAGSENSOR_PRECISION_HIGH)
	{
		sen[0] = MPU9250_MAG_MAX_FLUX_DENSITY / MPU9250_MAG_ADC_HIGH_RANGE;
		vMagCtrl1Val = MPU9250_MAG_CTRL1_BIT_16;
	}
	else
	{
		sen[0] = MPU9250_MAG_MAX_FLUX_DENSITY / MPU9250_MAG_ADC_LOW_RANGE;
		vMagCtrl1Val &= ~MPU9250_MAG_CTRL1_BIT_16;
	}

	uint8_t regaddr = MPU9250_MAG_CTRL1;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &vMagCtrl1Val, 1);

	sen[1] = sen[0];
	sen[2] = sen[0];

	Sensitivity(sen);

	vPrecision = Val;

	return vPrecision;
}

bool AgmMpu9250::Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(CfgData.DevAddr, pIntrf, pTimer) == false)
		return false;

	return true;
}

bool AgmMpu9250::Enable()
{
	uint8_t regaddr = MPU9250_AG_PWR_MGMT_1;
	uint8_t d;

	d = Read8(&regaddr, 1);
	d &= ~(MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_CLKSEL_MASK);

	Write8(&regaddr, 1, d |	MPU9250_AG_PWR_MGMT_1_CLKSEL_AUTO);

	regaddr = MPU9250_AG_PWR_MGMT_2;

	d = 0;

	// Enable Accel & Gyro
	if (vbSensorEnabled[MPU9250_ACCEL_IDX] == false)
	{
		d |= MPU9250_AG_PWR_MGMT_2_DIS_ZA |
			 MPU9250_AG_PWR_MGMT_2_DIS_YA |
			 MPU9250_AG_PWR_MGMT_2_DIS_XA;
	}
	if (vbSensorEnabled[MPU9250_GYRO_IDX] == false)
	{
		d |= MPU9250_AG_PWR_MGMT_2_DIS_ZG |
			 MPU9250_AG_PWR_MGMT_2_DIS_YG |
			 MPU9250_AG_PWR_MGMT_2_DIS_XG;
	}
	Write8(&regaddr, 1, d);

	if (vbSensorEnabled[MPU9250_MAG_IDX] == true)
	{
		printf("Mag Enabled\r\n");

		MagMpu9250::Enable();
		// Enable Mag
//		regaddr = MPU9250_MAG_CTRL1;
//		Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &vMagCtrl1Val, 1);
	}


	return true;
}

void AgmMpu9250::Disable()
{
	uint8_t regaddr = MPU9250_AG_PWR_MGMT_2;
	uint8_t d;
//Reset();
//msDelay(2000);
/*
	regaddr = MPU9250_AG_PWR_MGMT_1;
	d = Read8(&regaddr, 1);
	d |= MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_CLKSEL_STOP;
	Write8(&regaddr, 1, d);

	return;
*/
	regaddr = MPU9250_AG_USER_CTRL;
	Write8(&regaddr, 1, MPU9250_AG_USER_CTRL_I2C_MST_EN);

	// Disable Mag
	//regaddr = MPU9250_MAG_CTRL1;
	//uint8_t d = 0;
	//Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d, 1);
	MagMpu9250::Disable();

	// Disable Accel Gyro
	Write8(&regaddr, 1,
		 MPU9250_AG_PWR_MGMT_2_DIS_ZG | MPU9250_AG_PWR_MGMT_2_DIS_YG | MPU9250_AG_PWR_MGMT_2_DIS_XG |
		 MPU9250_AG_PWR_MGMT_2_DIS_ZA | MPU9250_AG_PWR_MGMT_2_DIS_YA | MPU9250_AG_PWR_MGMT_2_DIS_XA);

	regaddr = MPU9250_AG_PWR_MGMT_1;
	d = Read8(&regaddr, 1);
	d |= MPU9250_AG_PWR_MGMT_1_SLEEP;
	Write8(&regaddr, 1, d);//MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_PD_PTAT |
//	Write8(&regaddr, 1, MPU9250_AG_PWR_MGMT_1_SLEEP | MPU9250_AG_PWR_MGMT_1_PD_PTAT |
//						MPU9250_AG_PWR_MGMT_1_GYRO_STANDBY);

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

// Implement wake on motion
bool AgmMpu9250::WakeOnEvent(bool bEnable, int Threshold)
{
    uint8_t regaddr;

	if (bEnable == true)
	{
		Reset();

		msDelay(2000);

	    regaddr = MPU9250_AG_PWR_MGMT_1;
	    Write8(&regaddr, 1, 0);

	    regaddr = MPU9250_AG_PWR_MGMT_2;
		Write8(&regaddr, 1, /**MPU9250_AG_PWR_MGMT_2_DIS_XA | MPU9250_AG_PWR_MGMT_2_DIS_YA |*/
				MPU9250_AG_PWR_MGMT_2_DIS_XG | MPU9250_AG_PWR_MGMT_2_DIS_YG | MPU9250_AG_PWR_MGMT_2_DIS_ZG);

		regaddr = MPU9250_AG_ACCEL_CONFIG2;
	    Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG2_ACCEL_FCHOICE_B
	    	   | MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_5HZ);//
	    	   //| MPU9250_AG_ACCEL_CONFIG2_FIFO_SIZE_1024);

	    regaddr = MPU9250_AG_INT_ENABLE;
	    Write8(&regaddr, 1, MPU9250_AG_INT_ENABLE_WOM_EN);

	    regaddr = MPU9250_AG_MOT_DETECT_CTRL;
	    Write8(&regaddr, 1, MPU9250_AG_MOT_DETECT_CTRL_ACCEL_INTEL_MODE | MPU9250_AG_MOT_DETECT_CTRL_ACCEL_INTEL_EN);

	    regaddr = MPU9250_AG_WOM_THR;
	    Write8(&regaddr, 1, Threshold);

	    regaddr = MPU9250_AG_LP_ACCEL_ODR;
	    Write8(&regaddr, 1, 0);

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

// Accel low pass frequency
uint32_t AgmMpu9250::LowPassFreq(uint32_t Freq)
{
	uint8_t regaddr = MPU9250_AG_ACCEL_CONFIG2;
	uint32_t rate = 1000;
	uint8_t d = 0;

	if (Freq == 0)
	{
		rate = 4000;
		d = MPU9250_AG_ACCEL_CONFIG2_ACCEL_FCHOICE_B;
		AccelSensor::FilterFreq(1130);
	}
	else if (Freq < 10)
	{
		d = MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_5HZ;
		AccelSensor::FilterFreq(5);
	}
	else if (Freq < 20)
	{
		d = MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_10HZ;
		AccelSensor::FilterFreq(10);
	}
	else if (Freq < 40)
	{
		d = MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_20HZ;
		AccelSensor::FilterFreq(20);
	}
	else if (Freq < 50)
	{
		d = MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_41HZ;
		AccelSensor::FilterFreq(41);
	}
	else if (Freq < 100)
	{
		d = MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_92HZ;
		AccelSensor::FilterFreq(92);
	}
	else if (Freq < 200)
	{
		d = MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_184HZ;
		AccelSensor::FilterFreq(184);
	}
	else if (Freq < 500)
	{
		d = MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_460HZ;
		AccelSensor::FilterFreq(460);
	}
	else
	{
		rate = 4000;
		d = MPU9250_AG_ACCEL_CONFIG2_ACCEL_FCHOICE_B;
		AccelSensor::FilterFreq(1130);
	}

	Write8(&regaddr, 1, d);// | MPU9250_AG_ACCEL_CONFIG2_FIFO_SIZE_1024);

	regaddr = MPU9250_AG_SMPLRT_DIV;
	d = Read8(&regaddr, 0);
	d = rate / Freq - 1;
	Write8(&regaddr, 1, d);


	return AccelSensor::FilterFreq();
}

// Accel scale
uint16_t AccelMpu9250::Scale(uint16_t Value)
{
	uint8_t regaddr = MPU9250_AG_ACCEL_CONFIG;

	if (Value < 3)
	{
		Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_2G);
		AccelSensor::Scale(2);
	}
	else if (Value < 6)
	{
		Write8(&regaddr, 1, MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_4G);
		AccelSensor::Scale(4);
	}
	else if (Value < 12)
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

// Gyro scale
uint32_t GyroMpu9250::Sensitivity(uint32_t Value)
{
	uint8_t regaddr = MPU9250_AG_GYRO_CONFIG;
	uint8_t d = Read8(&regaddr, 1) & MPU9250_AG_GYRO_CONFIG_FCHOICE_MASK;

	if (Value < 500)
	{
		d |= MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_250DPS;
		GyroSensor::Sensitivity(250);
	}
	else if (Value < 1000)
	{
		d |= MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_500DPS;
		GyroSensor::Sensitivity(500);
	}
	else if (Value < 2000)
	{
		d |= MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_1000DPS;
		GyroSensor::Sensitivity(1000);
	}
	else
	{
		d |= MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_2000DPS;
		GyroSensor::Sensitivity(2000);
	}

	Write8(&regaddr, 1, d);

	return GyroSensor::Sensitivity();
}

bool AgmMpu9250::UpdateData()
{
	uint8_t regaddr = MPU9250_AG_FIFO_COUNT_H;//MPU9250_AG_ACCEL_XOUT_H;
	uint8_t d[32];
	uint16_t val;
	uint16_t cnt;
	uint64_t t = 0;

	//vSampleCnt++;

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	val = 0;

	if (vbSensorEnabled[MPU9250_ACCEL_IDX] == true)
		val += 8;
	if (vbSensorEnabled[MPU9250_GYRO_IDX] == true)
		val += 6;
	//if (vbSensorEnabled[MPU9250_MAG_IDX] == true)
	//	val += 8;

	if (vbDmpEnabled == true)
	{
		Read(&regaddr, 1, (uint8_t*)&cnt, 2);
		cnt = EndianCvt16(cnt);

		if (cnt >= val)
		{
			val = min(cnt, 32);
			regaddr = MPU9250_AG_FIFO_R_W;
			val = Read(&regaddr, 1, d , val);
		}
		else
		{
			val = 0;
		}
	}
	else
	{
		regaddr = MPU9250_AG_ACCEL_XOUT_H;
		Read(&regaddr, 1, (uint8_t*)d, val);
		if (vbSensorEnabled[MPU9250_MAG_IDX] == true)
		{
			regaddr = MPU9250_MAG_ST1;
			MagMpu9250::Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d[val], 8);
		}
	}

	int idx = 0;

	if (val > 0)
	{
		AccelSensor::vData.Scale =  AccelSensor::Scale();
		AccelSensor::vData.Range = 0x7FFF;
		AccelSensor::vData.X = ((int32_t)d[0] << 8) | d[1];
		AccelSensor::vData.Y = ((int32_t)d[2] << 8) | d[3];
		AccelSensor::vData.Z = ((int32_t)d[4] << 8) | d[5];
		AccelSensor::vData.Timestamp = t;
		AccelSensor::vSampleTime = t;
		AccelSensor::vSampleCnt++;

		TempSensor::vData.Temperature = (((int32_t)d[6] << 8) | d[7]) * 100;
		TempSensor::vData.Timestamp = t;

		idx += 8;

		if (val > 8)
		{
			GyroSensor::vData.X = ((int32_t)d[8] << 8) | d[9];
			GyroSensor::vData.Y = ((int32_t)d[10] << 8) | d[11];
			GyroSensor::vData.Z = ((int32_t)d[12] << 8) | d[13];
			GyroSensor::vData.Timestamp = t;
			GyroSensor::vSampleTime = t;
			GyroSensor::vSampleCnt++;
			idx += 6;
		}
	}

	if (vbSensorEnabled[MPU9250_MAG_IDX] == true)
	{
		//regaddr = MPU9250_MAG_ST1;
		//Read(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, (uint8_t*)d, 8);

		if (d[idx++] & MPU9250_MAG_ST1_DRDY)
		{
			val = (((int16_t)d[idx]) << 8L) | d[idx + 1];
			val += (val * vMagSenAdj[0]) >> 8L;
			MagSensor::vData.X = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vRange);

			idx += 2;
			val = (((int16_t)d[idx]) << 8) | d[idx + 1];
			val += (val * vMagSenAdj[1]) >> 8L;
			MagSensor::vData.Y = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vRange);

			idx += 2;
			val = (((int16_t)d[idx]) << 8) | d[idx + 1];
			val += (val * vMagSenAdj[2]) >> 8L;
			MagSensor::vData.Z = (int16_t)(val * (MPU9250_MAG_MAX_FLUX_DENSITY << 8) / MagSensor::vRange);

			MagSensor::vData.Timestamp = t;
			MagSensor::vSampleTime = t;
			MagSensor::vSampleCnt++;
		}
	}

	return true;
}

int AgmMpu9250::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}


int AgmMpu9250::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

int MagMpu9250::Read(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint8_t regaddr;
		uint8_t d[8];

		d[0] = MPU9250_AG_I2C_SLV0_ADDR;
		d[1] = DevAddr | MPU9250_AG_I2C_SLV0_ADDR_I2C_SLVO_RD;
		d[2] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(15, BuffLen);

			d[3] = MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_EN |cnt;

			Write(d, 4, NULL, 0);
			//Write8(d, 3, d[3]);

			// Delay require for transfer to complete
			msDelay(15 + cnt);

			regaddr = MPU9250_AG_EXT_SENS_DATA_00;

			cnt = Read(&regaddr, 1, pBuff, cnt);
			if (cnt <=0)
				break;

			pBuff += cnt;
			BuffLen -= cnt;
			retval += cnt;
		}
	}
	else
	{
		retval = vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	return retval;
}

int MagMpu9250::Write(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint8_t regaddr;
		uint8_t d[8];

		d[0] = MPU9250_AG_I2C_SLV0_ADDR;
		d[1] = DevAddr;
		d[2] = *pCmdAddr;
		d[3] = MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_EN | 1;

		while (DataLen > 0)
		{
			regaddr = MPU9250_AG_I2C_SLV0_DO;
			Write8(&regaddr, 1, *pData);

			//Write(d, 4, NULL, 0);
			Write8(d, 3, d[3]);

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

void AgmMpu9250::IntHandler()
{
	uint8_t regaddr = MPU9250_AG_INT_STATUS;
	uint8_t d;

	d = Read8(&regaddr, 1);
	//printf("int %x\r\n", d);
	if (d & MPU9250_AG_INT_STATUS_RAW_DATA_RDY_INT)
	{
		IOPinSet(0, 24);
		UpdateData();
		IOPinClear(0, 24);
	}
}

void AgmMpu9250::EnableFifo()
{
	uint8_t regaddr;// = MPU9250_AG_USER_CTRL;
	uint8_t d;
	uint8_t intval;

	// Save relevant registers
	regaddr = MPU9250_AG_INT_ENABLE;
	intval = Read8(&regaddr, 1);
	Write8(&regaddr, 1, 0);

	regaddr = MPU9250_AG_USER_CTRL;
	d = Read8(&regaddr, 1) | MPU9250_AG_USER_CTRL_FIFO_EN;
	Write8(&regaddr, 1, d);

	regaddr = MPU9250_AG_ACCEL_CONFIG2;
	d = Read8(&regaddr, 1) | MPU9250_AG_ACCEL_CONFIG2_FIFO_SIZE_1024;
	Write8(&regaddr, 1, d);

	regaddr = MPU9250_AG_FIFO_EN;
	d = Read8(&regaddr, 1);

	if (vbSensorEnabled[MPU9250_ACCEL_IDX])
	{
		d |= MPU9250_AG_FIFO_EN_ACCEL | MPU9250_AG_FIFO_EN_TEMP_OUT;
	}
	if (vbSensorEnabled[MPU9250_GYRO_IDX])
	{
		d |= MPU9250_AG_FIFO_EN_GYRO_ZOUT | MPU9250_AG_FIFO_EN_GYRO_YOUT |
			 MPU9250_AG_FIFO_EN_GYRO_XOUT;
	}
	Write8(&regaddr, 1, d);

	regaddr = MPU9250_AG_INT_ENABLE;
	if (intval != 0)
	{
		intval |= MPU9250_AG_INT_ENABLE_FIFO_OFLOW_EN;
		Write8(&regaddr, 1, intval);
	}
}

int AgmMpu9250::GetFifoLen()
{
	uint8_t regaddr = MPU9250_AG_FIFO_COUNT_H;
	uint16_t val;

	Read(&regaddr, 1, (uint8_t*)&val, 2);

	return EndianCvt16(val);
}

int AgmMpu9250::ReadFifo(uint8_t * const pBuff, int Len)
{
	uint8_t regaddr = MPU9250_AG_FIFO_COUNT_H;
	uint16_t val;
	int cnt = 0;

	Read(&regaddr, 1, (uint8_t*)&val, 2);
	val = EndianCvt16(val);

	if (val > 0)
	{
		Len = min(val, Len);
		regaddr = MPU9250_AG_FIFO_R_W;
		cnt = Read(&regaddr, 1, pBuff , Len);
	}

	return cnt;
}

void AgmMpu9250::ResetFifo()
{
	uint8_t regaddr;
	uint8_t intval;
	uint8_t fifoen;
	uint8_t usrctrl;

	// Save relevant registers
	regaddr = MPU9250_AG_INT_ENABLE;
	intval = Read8(&regaddr, 1);
	Write8(&regaddr, 1, 0);

	regaddr = MPU9250_AG_FIFO_EN;
	fifoen = Read8(&regaddr, 1);
	Write8(&regaddr, 1, 0);

	regaddr = MPU9250_AG_USER_CTRL;
	usrctrl = Read8(&regaddr, 1);
	Write8(&regaddr, 1, 0);

	Write8(&regaddr, 1, MPU9250_AG_USER_CTRL_FIFO_RST);

	msDelay(1);

	// restore register
	Write8(&regaddr, 1, usrctrl);

	msDelay(1);

	regaddr = MPU9250_AG_FIFO_EN;
	Write8(&regaddr, 1, fifoen);

	regaddr = MPU9250_AG_INT_ENABLE;
	Write8(&regaddr, 1, intval);
}

bool AgmMpu9250::InitDMP(uint32_t DmpStartAddr, uint8_t * const pDmpImage, int Len)
{
	bool res = false;
	uint8_t regaddr;
	uint8_t d[2];
//	uint8_t intval;

	if (pDmpImage == NULL || Len == 0)
		return false;

	// load external image
	res = UploadDMPImage(pDmpImage, Len);

	if (res)
	{
		vbDmpEnabled = true;

		d[0] = DmpStartAddr >> 8;//DMP_START_ADDR >> 8;
		d[1] = DmpStartAddr & 0xFF;//DMP_START_ADDR & 0xFF;

		// Write DMP program start address
		regaddr = MPU9250_DMP_PROG_START;
		Write(&regaddr, 1, d, 2);

		// Undocumented : Enable DMP
		regaddr = MPU9250_AG_USER_CTRL;
		d[0] = Read8(&regaddr, 1) | MPU9250_AG_USER_CTRL_DMP_EN;
		Write8(&regaddr, 1, d[0]);

		// DMP require fifo
		EnableFifo();

		res = true;
	}

	return res;
}

bool AgmMpu9250::UploadDMPImage(uint8_t *pDmpImage, int Len)
{
	int len = Len;
	uint8_t *p = pDmpImage;
	uint8_t regaddr;
	uint16_t memaddr = 0;
	uint8_t d[2];

	while (len > 0)
	{
		int l = min(len, MPU9250_DMP_MEM_PAGE_SIZE);

		regaddr = MPU9250_DMP_MEM_BANKSEL;
		d[0] = memaddr >> 8;
		d[1] = memaddr & 0xFF;

		Write(&regaddr, 1, d, 2);

		regaddr = MPU9250_DMP_MEM_RW;
		Write(&regaddr, 1, p, l);

		p += l;
		memaddr += l;
		len -= l;
	}

	len = Len;
	p = pDmpImage;
	memaddr = 0;

	// Verify
	while (len > 0)
	{
		uint8_t m[MPU9250_DMP_MEM_PAGE_SIZE];
		int l = min(len, MPU9250_DMP_MEM_PAGE_SIZE);

		regaddr = MPU9250_DMP_MEM_BANKSEL;
		d[0] = memaddr >> 8;
		d[1] = memaddr & 0xFF;

		Write(&regaddr, 1, d, 2);

		regaddr = MPU9250_DMP_MEM_RW;
		Read(&regaddr, 1, m, l);

		if (memcmp(p, m, l) != 0)
		{
			return false;
		}

		p += l;
		memaddr += l;
		len -= l;
	}

	return true;
}

bool MagMpu9250::Enable()
{
	uint8_t regaddr = MPU9250_MAG_CTRL1;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &vMagCtrl1Val, 1);

	return true;
}

void MagMpu9250::Disable()
{
	uint8_t regaddr = MPU9250_MAG_CTRL1;
	uint8_t d = 0;
	Write(MPU9250_MAG_I2C_DEVADDR, &regaddr, 1, &d, 1);
}
