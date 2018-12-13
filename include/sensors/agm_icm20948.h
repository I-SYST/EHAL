/**-------------------------------------------------------------------------
@file	agm_icm20948.h

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

#ifndef __AGM_ICM20948_H__
#define __AGM_ICM20948_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"

#define ICM20948_I2C_DEV_ADDR0			0x68		// AD0 low
#define ICM20948_I2C_DEV_ADDR1			0x69		// AD0 high

/// Register Bank 0

#define ICM20948_WHO_AM_I				0

#define ICM20948_WHO_AM_I_ID						(0xEA)

#define ICM20948_USER_CTRL				3

#define ICM20948_USER_CTRL_I2C_MST_RST				(1<<1)	// Reset I2C Master module
#define ICM20948_USER_CTRL_SRAM_RST					(1<<2)	// Reset SRAM module
#define ICM20948_USER_CTRL_DMP_RST					(1<<3)	// Reset DMP module
#define ICM20948_USER_CTRL_I2C_IF_DIS				(1<<4)	// Reset and disable I2C Slave module
#define ICM20948_USER_CTRL_I2C_MST_EN				(1<<5)	// Enable I2C master module
#define ICM20948_USER_CTRL_FIFO_EN					(1<<6)	// Enable FIFO operation mode
#define ICM20948_USER_CTRL_DMP_EN					(1<<7)	// Enable DMP feature

#define ICM20948_LP_CONFIG				5

#define ICM20948_LP_CONFIG_GYRO_CYCLE				(1<<4)	// Operate Gyro in duty cycled mode
#define ICM20948_LP_CONFIG_ACCEL_CYCLE				(1<<5)	// Operate Accel in duty cycled mode
#define ICM20948_LP_CONFIG_I2C_MST_CYCLE			(1<<4)	// Operate I2C master in duty cycled mode

#define ICM20948_PWR_MGMT_1				6

#define ICM20948_PWR_MGMT_1_CLKSEL_MASK				(7<<0)	// Clock source
#define ICM20948_PWR_MGMT_1_CLKSEL_BITPOS			(0)
#define ICM20948_PWR_MGMT_1_TEMP_DIS				(1<3)	// Disable temperature sensor
#define ICM20948_PWR_MGMT_1_LP_EN					(1<<5)	// Low Power enable
#define ICM20948_PWR_MGMT_1_SLEEP					(1<<6)	// Enter sleep
#define ICM20948_PWR_MGMT_1_DEVICE_RESET			(1<<7)	// Reset to default settings

#define ICM20948_PWR_MGMT_2				7

#define ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK		(7<<0)	// 0 : Gyro on, 7 : Gyro off
#define ICM20948_PWR_MGMT_2_DISABLE_GYRO_BITPOS		(0)
#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK		(7<<3)	// 0 : Accel om, 7 Accel off
#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL_BITPOS	(3)

#define ICM20948_INT_PIN_CFG			15

#define ICM20948_INT_PIN_CFG_BYPASS_EN				(1<<1)	// I2C master in bypass mode
#define ICM20948_INT_PIN_CFG_FSYNC_INT_MODE_EN		(1<<2)	// FSYNC pin used as interrupt
#define ICM20948_INT_PIN_CFG_ATCL_FSYNC				(1<<3)	// Logic level of FSYNC Interrupt. 1 : Active low, 0 : active high.
#define ICM20948_INT_PIN_CFG_INT_ANYRD_2CLEAR		(1<<4)	// Clear interrupt status
#define ICM20948_INT_PIN_CFG_INT1_LATCH_EN			(1<<5)	// Latch INT1
#define ICM20948_INT_PIN_CFG_INT1_OPEN				(1<<6)	// Open drain
#define ICM20948_INT_PIN_CFG_INT1_ACTL				(1<<7)	// Logic level.  1 : active low, 0 : active high

#define ICM20948_INT_ENABLE				16

#define ICM20948_INT_ENABLE_I2C_MST_INT_EN			(1<<0)	// Enable I2C master interrupt on pin 1
#define ICM20948_INT_ENABLE_DMP_INT1_EN				(1<<1)	// Enable DMP interrupt on pin 1
#define ICM20948_INT_ENABLE_PLL_RDY_EN				(1<<2)	// Enable interrupt on pin 1
#define ICM20948_INT_ENABLE_WOM_INT_EN				(1<<3)	// Enable wake on motion interrupt on pin 1
#define ICM20948_INT_ENABLE_REG_WOF_EN				(1<<7)	// Enable wake on FSYNC interrupt

#define ICM20948_INT_ENABLE_1			17

#define ICM20948_INT_ENABLE_1_RAW_DATA_0_DRY_EN		(1<<0)	// Enable raw data ready interrupt on pin 1

#define ICM20948_INT_ENABLE_2			18

#define ICM20948_INT_ENABLE_2_FIFO_OVERFLOW_EN_MASK	(0xf<<0)// Enable FIFO overflow interrupt on pin 1

#define ICM20948_INT_ENABLE_3			19

#define ICM20948_INT_ENABLE_3_FIFO_WM_EN_MASK		(0xf<<0)// Enable FIFO watermark interrupt on pin 1

#define ICM20948_I2C_MST_STATUS			23

#define ICM20948_I2C_MST_STATUS_I2C_SLV0_NACK		(1<<0)	// Slave 0 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV1_NACK		(1<<1)	// Slave 1 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV2_NACK		(1<<2)	// Slave 2 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV3_NACK		(1<<3)	// Slave 3 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV4_NACK		(1<<4)	// Slave 4 NACK
#define ICM20948_I2C_MST_STATUS_I2C_LOST_ARB		(1<<5)	// Lost arbitration
#define ICM20948_I2C_MST_STATUS_I2C_SLV4_DONE		(1<<6)	// Slave 4 transfer complete
#define ICM20948_I2C_MST_STATUS_PASS_THROUGH		(1<<7)	// FSYNC interrupt flag

#define ICM20948_INT_STATUS				25

#define ICM20948_INT_STATUS_I2C_MIST_INT			(1<<0)	// I2C master interrupt flag
#define ICM20948_INT_STATUS_DMP_INT1				(1<<1)	// DMP interrupt on pin 1
#define ICM20948_INT_STATUS_PLL_RDY_INT				(1<<2)	// PLL enable & ready interrupt flag
#define ICM20948_INT_STATUS_WOM_INT					(1<<3)	// Wake on motion interrupt flag

#pragma pack(push, 1)

#pragma pack(pop)

class AgmIcm20948 : public AccelSensor, public GyroSensor, public MagSensor {
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
	// Default base initialization. Does detection and set default config for all sensor.
	// All sensor init must call this first prio to initializing itself
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	bool vbSpi;
	bool vbInitialized;
	uint8_t vMagCtrl1Val;
	int16_t vMagSenAdj[3];
};


#endif // __AGM_ICM20948_H__

