/**-------------------------------------------------------------------------
@file	agm_lsm9ds1.h

@brief	Implementation of ST LSM9DS1 sensor
			Accel, Gyro, Mag


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

#ifndef __AGM_LSM9DS1_H__
#define __AGM_LSM9DS1_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"

// 7 bits I2C device addresses
#define LSM9DS1_AG_I2C_ADDR			0x6A
#define LSM9DS1_AG_I2C_ADDR			0x6B
#define LSM9DS1_MAG_I2C_ADDR		0x1C
#define LSM9DS1_MAG_I2C_ADDR		0x1E

// Activity threshold register
#define LSM9DS1_ACT_THS			0x04

#define LSM9DS1_ACT_THS_ACT_THS_MASK					(0x3f)
#define LSM9DS1_ACT_THS_SLEEP_ON_INECT_EN				(1<<7)

// Inactivity duration register
#define LSM9DS1_ACT_DUR			0x05

#define LSM9DS1_ACT_DUR_MASK							(0xff)

// Linear acceleration sensor interrupt generator configuration register
#define LSM9DS1_INT_GEN_CFG_XL	0x06

#define LSM9DS1_INT_GEN_CFG_XL_XLIE_XL					(1<<0)
#define LSM9DS1_INT_GEN_CFG_XL_XHIE_XL					(1<<1)
#define LSM9DS1_INT_GEN_CFG_XL_YLIE_XL					(1<<2)
#define LSM9DS1_INT_GEN_CFG_XL_YHIE_XL					(1<<3)
#define LSM9DS1_INT_GEN_CFG_XL_ZLIE_XL					(1<<4)
#define LSM9DS1_INT_GEN_CFG_XL_ZHIE_XL					(1<<5)
#define LSM9DS1_INT_GEN_CFG_XL_6D						(1<<6)
#define LSM9DS1_INT_GEN_CFG_XL_AOI_XL					(1<<7)


#pragma pack(push, 1)

#pragma pack(pop)

#ifdef __cplusplus

class AgmLsm9ds1 : public AccelSensor, public GyroSensor, public MagSensor {
public:
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
	virtual bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

	/**
	 * @brief	Initialize gyroscope sensor.
	 *
	 * NOTE : Accelerometer must be initialized first prior to this one.
	 *
	 * @param 	Cfg		: Accelerometer configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const GYROSENSOR_CFG &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL);

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
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	/**
	 * @brief	Enable/Disable wake on motion event
	 *
	 * @param bEnable
	 * @param Threshold
	 * @return
	 */
	virtual bool WakeOnEvent(bool bEnable, int Threshold);

	virtual bool StartSampling();
	virtual uint32_t LowPassFreq(uint32_t Freq);

	virtual uint16_t Scale(uint16_t Value);			// Accel
	virtual uint32_t Sensitivity(uint32_t Value);	// Gyro


	virtual bool Read(ACCELSENSOR_DATA &Data);
	virtual bool Read(GYROSENSOR_DATA &Data);
	virtual bool Read(MAGSENSOR_DATA &Data);

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	int Read(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	bool UpdateData();
	virtual void IntHandler();

private:
};

#endif // __cplusplus

#endif // __AGM_LSM9DS1_H__
