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
#define LSM9DS1_AG_I2C_ADDR0			0x6A
#define LSM9DS1_AG_I2C_ADDR1		0x6B
#define LSM9DS1_MAG_I2C_ADDR0		0x1C
#define LSM9DS1_MAG_I2C_ADDR1		0x1E

#define LSM9DS1_DEVID				0x68

// Activity threshold register
#define LSM9DS1_ACT_THS				0x04

#define LSM9DS1_ACT_THS_ACT_THS_MASK					(0x3F)
#define LSM9DS1_ACT_THS_SLEEP_ON_INECT_EN				(1<<7)

// Inactivity duration register
#define LSM9DS1_ACT_DUR				0x05

#define LSM9DS1_ACT_DUR_MASK							(0xFF)

// Linear acceleration sensor interrupt generator configuration register
#define LSM9DS1_INT_GEN_CFG_XL		0x06

#define LSM9DS1_INT_GEN_CFG_XL_XLIE_XL					(1<<0)
#define LSM9DS1_INT_GEN_CFG_XL_XHIE_XL					(1<<1)
#define LSM9DS1_INT_GEN_CFG_XL_YLIE_XL					(1<<2)
#define LSM9DS1_INT_GEN_CFG_XL_YHIE_XL					(1<<3)
#define LSM9DS1_INT_GEN_CFG_XL_ZLIE_XL					(1<<4)
#define LSM9DS1_INT_GEN_CFG_XL_ZHIE_XL					(1<<5)
#define LSM9DS1_INT_GEN_CFG_XL_6D						(1<<6)
#define LSM9DS1_INT_GEN_CFG_XL_AOI_XL					(1<<7)


// Linear acceleration sensor interrupt threshold register
#define LSM9DS1_INT_GEN_THS_X_XL	0x07

#define LSM9DS1_INT_GEN_THS_X_XL_MASK					(0xFF)

#define LSM9DS1_INT_GEN_THS_Y_XL	0x08

#define LSM9DS1_INT_GEN_THS_Y_XL_MASK					(0xFF)

#define LSM9DS1_INT_GEN_THS_Z_XL	0x09

#define LSM9DS1_INT_GEN_THS_Z_XL_MASK					(0xFF)

// Linear acceleration sensor interrupt duration register
#define LSM9DS1_INT_GEN_DUR_XL		0x0A

#define LSM9DS1_INT_GEN_DUR_XL_DUR_XL_MASK				(0x7F)
#define LSM9DS1_INT_GEN_DUR_XL_WAIT_XL					(1<<7)

// Angular rate sensor reference value register for digital high-pass filter (r/w).
#define LSM9DS1_REFERENCE_G			0x0B

#define LSM9DS1_REFERENCE_G_MASK						(0xFF)

// INT1_A/G pin control register
#define LSM9DS1_INT1_CTRL			0x0C

#define LSM9DS1_INT1_CTRL_INT_DRDY_XL					(1<<0)
#define LSM9DS1_INT1_CTRL_INT_DRDY_G					(1<<1)
#define LSM9DS1_INT1_CTRL_INT_BOOT						(1<<2)
#define LSM9DS1_INT1_CTRL_INT_FTH						(1<<3)
#define LSM9DS1_INT1_CTRL_INT_OVR						(1<<4)
#define LSM9DS1_INT1_CTRL_INT_FSS5						(1<<5)
#define LSM9DS1_INT1_CTRL_INT_IG_XL						(1<<6)
#define LSM9DS1_INT1_CTRL_INT_IG_G						(1<<7)

// INT2_A/G pin control register
#define LSM9DS1_INT2_CTRL			0x0D

#define LSM9DS1_INT2_CTRL_INT2_DRDY_XL					(1<<0)
#define LSM9DS1_INT2_CTRL_INT2_DRDY_G					(1<<1)
#define LSM9DS1_INT2_CTRL_INT2_DRDY_TEMP				(1<<2)
#define LSM9DS1_INT2_CTRL_INT2_FTH						(1<<3)
#define LSM9DS1_INT2_CTRL_INT2_OVR						(1<<4)
#define LSM9DS1_INT2_CTRL_INT2_FSS5						(1<<5)
#define LSM9DS1_INT2_CTRL_INT2_INACT					(1<<7)

// Who_AM_I register
#define LSM9DS1_WHO_AM_I			0x0F

#define LSM9DS1_WHO_AM_I_ID								LSM9DS1_DEVID

// Angular rate sensor Control Register 1
#define LSM9DS1_CTRL_REG1_G			0x10

#define LSM9DS1_CTRL_REG1_G_BW_G_MASK					(3<<0)
#define LSM9DS1_CTRL_REG1_G_FS_G_MASK					(3<<3)
#define LSM9DS1_CTRL_REG1_G_ODR_G_MASK					(7<<5)

// Angular rate sensor Control Register 2
#define LSM9DS1_CTRL_REG2_G			0x11

#define LSM9DS1_CTRL_REG2_G_OUT_SEL_MASK				(3<<0)
#define LSM9DS1_CTRL_REG2_G_INT_SEL_MASK				(3<<2)

// Angular rate sensor Control Register 3
#define LSM9DS1_CTRL_REG3_G			0x12

#define LSM9DS1_CTRL_REG3_G_HPCF_G_MASK					(0xF<<0)
#define LSM9DS1_CTRL_REG3_G_HP_EN						(1<<6)
#define LSM9DS1_CTRL_REG3_G_LP_MODE						(1<<7)

// Angular rate sensor sign and orientation register
#define LSM9DS1_ORIENT_CFG_G		0x13

#define LSM9DS1_ORIENT_CFG_G_ORIENT_MASK				(7<<0)
#define LSM9DS1_ORIENT_CFG_G_SIGNZ_G					(1<<3)
#define LSM9DS1_ORIENT_CFG_G_SIGNY_G					(1<<4)
#define LSM9DS1_ORIENT_CFG_G_SIGNX_G					(1<<5)

// Angular rate sensor interrupt source register
#define LSM9DS1_INT_GEN_SRC_G		0x14

#define LSM9DS1_INT_GEN_SRC_G_XL_G						(1<<0)
#define LSM9DS1_INT_GEN_SRC_G_XH_G						(1<<1)
#define LSM9DS1_INT_GEN_SRC_G_YL_G						(1<<2)
#define LSM9DS1_INT_GEN_SRC_G_YH_G						(1<<3)
#define LSM9DS1_INT_GEN_SRC_G_ZL_G						(1<<4)
#define LSM9DS1_INT_GEN_SRC_G_ZH_G						(1<<5)
#define LSM9DS1_INT_GEN_SRC_G_IA_G						(1<<6)

// Temperature data output register
#define LSM9DS1_OUT_TEMP_L			0x15
#define LSM9DS1_OUT_TEMP_H			0x16

// Status register
#define LSM9DS1_STATUS_REG			0x17

#define LSM9DS1_STATUS_REG_XLDA							(1<<0)
#define LSM9DS1_STATUS_REG_GDA							(1<<1)
#define LSM9DS1_STATUS_REG_TDA							(1<<2)
#define LSM9DS1_STATUS_REG_BOOT_STATUS					(1<<3)
#define LSM9DS1_STATUS_REG_INACT						(1<<4)
#define LSM9DS1_STATUS_REG_IG_G							(1<<5)
#define LSM9DS1_STATUS_REG_IG_XL						(1<<6)

// Gyro output data
#define LSM9DS1_OUT_X_G_L			0x18
#define LSM9DS1_OUT_X_G_H			0x19
#define LSM9DS1_OUT_Y_G_L			0x1A
#define LSM9DS1_OUT_Y_G_H			0x1B
#define LSM9DS1_OUT_Z_G_L			0x1C
#define LSM9DS1_OUT_Z_G_H			0x1D

// Control register 4
#define LSM9DS1_CTRL_REG4			0x1E

#define LSM9DS1_CTRL_REG4_4D_XL1						(1<<0)
#define LSM9DS1_CTRL_REG4_LIR_XL1						(1<<1)
#define LSM9DS1_CTRL_REG4_XEN_G							(1<<3)
#define LSM9DS1_CTRL_REG4_YEN_G							(1<<4)
#define LSM9DS1_CTRL_REG4_ZEN_G							(1<<5)

// Linear acceleration sensor Control Register 5
#define LSM9DS1_CTRL_REG5_XL		0x1F

#define LSM9DS1_CTRL_REG5_XL_XEN_XL						(1<<3)
#define LSM9DS1_CTRL_REG5_XL_YEN_XL						(1<<4)
#define LSM9DS1_CTRL_REG5_XL_ZEN_XL						(1<<5)
#define LSM9DS1_CTRL_REG5_XL_DEC_0						(1<<6)
#define LSM9DS1_CTRL_REG5_XL_DEC_1						(1<<7)

// Linear acceleration sensor Control Register 6
#define LSM9DS1_CTRL_REG6_XL		0x20

#define LSM9DS1_CTRL_REG6_XL_BW_XL_MASK					(3<<0)
#define LSM9DS1_CTRL_REG6_XL_BW_SCAL_ODR				(1<<2)
#define LSM9DS1_CTRL_REG6_XL_FS_XL_MASK					(3<<3)
#define LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK				(7<<5)

// Linear acceleration sensor Control Register 7
#define LSM9DS1_CTRL_REG7_XL		0x21

#define LSM9DS1_CTRL_REG7_XL_HPIS1						(1<<0)
#define LSM9DS1_CTRL_REG7_XL_FDS						(1<<2)
#define LSM9DS1_CTRL_REG7_XL_DCF_MASK					(3<<5)
#define LSM9DS1_CTRL_REG7_XL_HR							(1<<7)

// Control register 8
#define LSM9DS1_CTRL_REG8			0x22

#define LSM9DS1_CTRL_REG8_SW_RESET						(1<<0)
#define LSM9DS1_CTRL_REG8_BLE							(1<<1)
#define LSM9DS1_CTRL_REG8_IF_ADD_INC					(1<<2)
#define LSM9DS1_CTRL_REG8_SIM							(1<<3)
#define LSM9DS1_CTRL_REG8_PP_OD							(1<<4)
#define LSM9DS1_CTRL_REG8_H_LACTIVE						(1<<5)
#define LSM9DS1_CTRL_REG8_BDU							(1<<6)
#define LSM9DS1_CTRL_REG8_BOOT							(1<<7)

// Control register 9
#define LSM9DS1_CTRL_REG9			0x23

#define LSM9DS1_CTRL_REG9_STOP_ON_FTH					(1<<0)
#define LSM9DS1_CTRL_REG9_FIFO_EN						(1<<1)
#define LSM9DS1_CTRL_REG9_I2C_DISABLE					(1<<2)
#define LSM9DS1_CTRL_REG9_DRDY_MASK_BIT					(1<<3)
#define LSM9DS1_CTRL_REG9_FIFO_TEMP_EN					(1<<4)
#define LSM9DS1_CTRL_REG9_SLEEP_G						(1<<6)

// Control register 10
#define LSM9DS1_CTRL_REG10			0x24

#define LSM9DS1_CTRL_REG10_ST_XL						(1<<0)
#define LSM9DS1_CTRL_REG10_ST_G							(1<<2)

// Linear acceleration sensor interrupt source register
#define LSM9DS1_INT_GEN_SRC_XL		0x26

#define LSM9DS1_INT_GEN_SRC_XL_XL_XL					(1<<0)
#define LSM9DS1_INT_GEN_SRC_XL_XH_XL					(1<<1)
#define LSM9DS1_INT_GEN_SRC_XL_YL_XL					(1<<2)
#define LSM9DS1_INT_GEN_SRC_XL_YH_XL					(1<<3)
#define LSM9DS1_INT_GEN_SRC_XL_ZL_XL					(1<<4)
#define LSM9DS1_INT_GEN_SRC_XL_ZH_XL					(1<<5)
#define LSM9DS1_INT_GEN_SRC_XL_IA_XL					(1<<6)

// Status register
#define LSM9DS1_STATUS_REG2			0x27

#define LSM9DS1_STATUS_REG2_XLDA						(1<<0)
#define LSM9DS1_STATUS_REG2_GDA							(1<<1)
#define LSM9DS1_STATUS_REG2_TDA							(1<<2)
#define LSM9DS1_STATUS_REG2_BOOT_STATUS					(1<<3)
#define LSM9DS1_STATUS_REG2_INACT						(1<<4)
#define LSM9DS1_STATUS_REG2_IG_G						(1<<5)
#define LSM9DS1_STATUS_REG2_IG_XL						(1<<6)

// Accel output data
#define LSM9DS1_OUT_X_XL_L			0x28
#define LSM9DS1_OUT_X_XL_H			0x29
#define LSM9DS1_OUT_Y_XL_L			0x2A
#define LSM9DS1_OUT_Y_XL_H			0x2B
#define LSM9DS1_OUT_Z_XL_L			0x2C
#define LSM9DS1_OUT_Z_XL_H			0x2D

// FIFO control register
#define LSM9DS1_FIFO_CTRL			0x2E

#define LSM9DS1_FIFO_CTRL_FTH_MASK						(0x1F<<0)
#define LSM9DS1_FIFO_CTRL_FMODE_MASK					(7<<5)

// FIFO status control register
#define LSM9DS1_FIFO_SRC			0x2F

#define LSM9DS1_FIFO_SRC_FSS_MASK						(3F<<0)
#define LSM9DS1_FIFO_SRC_OVRN							(1<<6)
#define LSM9DS1_FIFO_SRC_FTH							(1<<7)

// Angular rate sensor interrupt generator configuration register
#define LSM9DS1_INT_GEN_CFG_G		0x30

#define LSM9DS1_INT_GEN_CFG_G_XLIE_G					(1<<0)
#define LSM9DS1_INT_GEN_CFG_G_XHIE_G					(1<<1)
#define LSM9DS1_INT_GEN_CFG_G_YLIE_G					(1<<2)
#define LSM9DS1_INT_GEN_CFG_G_YHIE_G					(1<<3)
#define LSM9DS1_INT_GEN_CFG_G_ZLIE_G					(1<<4)
#define LSM9DS1_INT_GEN_CFG_G_ZHIE_G					(1<<5)
#define LSM9DS1_INT_GEN_CFG_G_LIR_G						(1<<6)
#define LSM9DS1_INT_GEN_CFG_G_AOI_G						(1<<7)

// Angular rate sensor interrupt generator threshold registers
#define LSM9DS1_INT_GEN_THS_XL_G	0x31

#define LSM9DS1_INT_GEN_THS_XH_G	0x32

#define LSM9DS1_INT_GEN_THS_XH_G_THS_G_MASK				(0x7F)
#define LSM9DS1_INT_GEN_THS_XH_G_DCRM_G					(1<<7)

#define LSM9DS1_INT_GEN_THS_YL_G	0x33

#define LSM9DS1_INT_GEN_THS_YH_G	0x34
#define LSM9DS1_INT_GEN_THS_YH_G_THS_G_MASK				(0x7F)

#define LSM9DS1_INT_GEN_THS_ZL_G	0x33

#define LSM9DS1_INT_GEN_THS_ZH_G	0x34
#define LSM9DS1_INT_GEN_THS_ZH_G_THS_G_MASK				(0x7F)

// Angular rate sensor interrupt generator duration register
#define LSM9DS1_INT_GEN_DUR_G		0x37

#define LSM9DS1_INT_GEN_DUR_G_DUR_G_MASK				(0x7F)
#define LSM9DS1_INT_GEN_DUR_G_WAIT_G					(1<<7)


#pragma pack(push, 1)

#pragma pack(pop)

#ifdef __cplusplus

class AccelLsm9ds1 : public AccelSensor {
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
	virtual uint16_t Scale(uint16_t Value);			// Accel

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
};

class GyroLsm9ds1 : public GyroSensor {
public:
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
	virtual uint32_t Sensitivity(uint32_t Value);	// Gyro

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
};

class MagLsm9ds1 : public MagSensor {
public:
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

private:
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) = 0;
};

class AgmLsm9ds1 : public AccelLsm9ds1, public GyroLsm9ds1, public MagLsm9ds1 {
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
	virtual bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) {
		return AccelLsm9ds1::Init(Cfg, pIntrf, pTimer);
	}

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
	virtual bool Init(const GYROSENSOR_CFG &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL) {
		return GyroLsm9ds1::Init(Cfg, pIntrf, pTimer);
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
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf* const pIntrf, Timer * const pTimer = NULL) {
		return MagLsm9ds1::Init(Cfg, pIntrf, pTimer);
	}

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
	virtual bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL) { return true; }
};

#endif // __cplusplus

#endif // __AGM_LSM9DS1_H__
