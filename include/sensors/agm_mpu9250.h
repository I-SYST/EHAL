/**-------------------------------------------------------------------------
@file	agm_mpu9250.h

@brief	Implementation of TDK MPU-9250 accel, gyro, mag sensor

This only implement the sensor portion of the MPU-9250.  The DMP part is
implemented in a higher level in IMU class device.

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

#ifndef __AGM_MPU9250_H__
#define __AGM_MPU9250_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"

#define MPU9250_I2C_DEV_ADDR0			0x68		// AD0 low
#define MPU9250_I2C_DEV_ADDR1			0x69		// AD0 high


/// Accel & Gyro registers
///

#define MPU9250_AG_XG_OFFSET_H			0x13
#define MPU9250_AG_XG_OFFSET_L			0x14
#define MPU9250_AG_YG_OFFSET_H			0x15
#define MPU9250_AG_YG_OFFSET_L			0x16
#define MPU9250_AG_ZG_OFFSET_H			0x17
#define MPU9250_AG_ZG_OFFSET_L			0x18

#define MPU9250_AG_SMPLRT_DIV			0x19

#define MPU9250_AG_CONFIG				0x1A

#define MPU9250_AG_CONFIG_DLPF_CFG_MASK 					(7)
#define MPU9250_AG_CONFIG_DLPF_CFG_250HZ					(0)
#define MPU9250_AG_CONFIG_DLPF_CFG_184HZ					(1)
#define MPU9250_AG_CONFIG_DLPF_CFG_92HZ						(2)
#define MPU9250_AG_CONFIG_DLPF_CFG_41HZ						(3)
#define MPU9250_AG_CONFIG_DLPF_CFG_20HZ						(4)
#define MPU9250_AG_CONFIG_DLPF_CFG_10HZ						(5)
#define MPU9250_AG_CONFIG_DLPF_CFG_5HZ						(6)
#define MPU9250_AG_CONFIG_DLPF_CFG_3600HZ					(7)

#define MPU9250_AG_CONFIG_EXT_SYNC_SET_MASK					(7<<3)
#define MPU9250_AG_CONFIG_FIFO_MODE_BLOCKING				(1<<6)

#define MPU9250_AG_GYRO_CONFIG			0x1B

#define MPU9250_AG_GYRO_CONFIG_FCHOICE_MASK					(3)
#define MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_MASK				(3<<3)
#define MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_250DPS			(0<<3)
#define MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_500DPS			(1<<3)
#define MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_1000DPS			(2<<3)
#define MPU9250_AG_GYRO_CONFIG_GYRO_FS_SEL_2000DPS			(3<<3)


#define MPU9250_AG_GYRO_CONFIG_ZGYRO_SELFTEST_EN			(1<<5)
#define MPU9250_AG_GYRO_CONFIG_YGYRO_SELFTEST_EN			(1<<6)
#define MPU9250_AG_GYRO_CONFIG_XGYRO_SELFTEST_EN			(1<<7)

#define MPU9250_AG_ACCEL_CONFIG			0x1C
#define MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_MASK			(3<<3)		// Accel full scale select mask
#define MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_2G				(0<<3)		// 2g
#define MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_4G				(1<<3)		// 4g
#define MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_8G				(2<<3)		// 8g
#define MPU9250_AG_ACCEL_CONFIG_ACCEL_FS_SEL_16G			(3<<3)		// 16g
#define MPU9250_AG_ACCEL_CONFIG_AZ_ST_EN					(1<<5)		// Z accel self test enable
#define MPU9250_AG_ACCEL_CONFIG_AY_ST_EN					(1<<6)		// Y accel self test enable
#define MPU9250_AG_ACCEL_CONFIG_AX_ST_EN					(1<<7)		// X accel self test enable

#define MPU9250_AG_ACCEL_CONFIG2		0x1D

#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_MASK				(3)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_BITPOS			(0)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_5HZ				(6)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_10HZ				(5)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_20HZ				(4)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_41HZ				(3)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_92HZ				(2)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_184HZ			(1)
#define MPU9250_AG_ACCEL_CONFIG2_A_DLPFCFG_460HZ			(7)
#define MPU9250_AG_ACCEL_CONFIG2_ACCEL_FCHOICE_B			(1<<3)
// Undocumented FIFO size settings
// min needs to be 1024, otherwise big large
#define MPU9250_AG_ACCEL_CONFIG2_FIFO_SIZE_1024				(1<<6)
#define MPU9250_AG_ACCEL_CONFIG2_FIFO_SIZE_2048				(2<<6)
#define MPU9250_AG_ACCEL_CONFIG2_FIFO_SIZE_4096				(3<<6)

#define MPU9250_AG_LP_ACCEL_ODR			0x1E

#define MPU9250_AG_LP_ACCEL_ODR_LPOSC_CLKSEL_MASK			(7)

#define MPU9250_AG_WOM_THR				0x1F

#define MPU9250_AG_FIFO_EN				0x23

#define MPU9250_AG_FIFO_EN_SLV0								(1<<0)
#define MPU9250_AG_FIFO_EN_SLV1								(1<<1)
#define MPU9250_AG_FIFO_EN_SLV2								(1<<2)
#define MPU9250_AG_FIFO_EN_ACCEL							(1<<3)
#define MPU9250_AG_FIFO_EN_GYRO_ZOUT						(1<<4)
#define MPU9250_AG_FIFO_EN_GYRO_YOUT						(1<<5)
#define MPU9250_AG_FIFO_EN_GYRO_XOUT						(1<<6)
#define MPU9250_AG_FIFO_EN_TEMP_OUT							(1<<7)

#define MPU9250_AG_I2C_MST_CTRL			0x24

#define MPU9250_AG_I2C_MST_CTRL_I2C_MST_CLK_MASK			(0xF)
#define MPU9250_AG_I2C_MST_CTRL_I2C_MST_P_NSR				(1<<4)
#define MPU9250_AG_I2C_MST_CTRL_SLV3_FIFO_EN				(1<<5)
#define MPU9250_AG_I2C_MST_CTRL_WAIT_FOR_ES					(1<<6)
#define MPU9250_AG_I2C_MST_CTRL_MULT_MST_EN					(1<<7)

#define MPU9250_AG_I2C_SLV0_ADDR		0x25

#define MPU9250_AG_I2C_SLV0_ADDR_I2C_ID_MASK				(0x7F)
#define MPU9250_AG_I2C_SLV0_ADDR_I2C_SLVO_RD				(1<<7)

#define MPU9250_AG_I2C_SLV0_REG			0x26

#define MPU9250_AG_I2C_SLV0_CTRL		0x27

#define MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK			(0xF)
#define MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_GRP				(1<<4)
#define MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_REG_DIS			(1<<5)
#define MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_BYTE_SW			(1<<6)
#define MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_EN				(1<<7)

#define MPU9250_AG_I2C_SLV1_ADDR		0x28

#define MPU9250_AG_I2C_SLV1_I2C_ID_MASK						(0x7F)
#define MPU9250_AG_I2C_SLV1_I2C_SLV1_RNW					(1<<7)

#define MPU9250_AG_I2C_SLV1_REG			0x29

#define MPU9250_AG_I2C_SLV1_CTRL		0x2A

#define MPU9250_AG_I2C_SLV1_CTRL_I2C_SLV1_LENG_MASK			(0xF)
#define MPU9250_AG_I2C_SLV1_CTRL_I2C_SLV1_GRP				(1<<4)
#define MPU9250_AG_I2C_SLV1_CTRL_I2C_SLV1_REG_DIS			(1<<5)
#define MPU9250_AG_I2C_SLV1_CTRL_I2C_SLV1_BYTE_SW			(1<<6)
#define MPU9250_AG_I2C_SLV1_CTRL_I2C_SLV1_EN				(1<<7)

#define MPU9250_AG_I2C_SLV2_ADDR		0x2B

#define MPU9250_AG_I2C_SLV2_I2C_ID_MASK						(0x7F)
#define MPU9250_AG_I2C_SLV2_I2C_SLV2_RNW					(1<<7)

#define MPU9250_AG_I2C_SLV2_REG			0x2C

#define MPU9250_AG_I2C_SLV2_CTRL		0x2D

#define MPU9250_AG_I2C_SLV2_CTRL_I2C_SLV2_LENG_MASK			(0xF)
#define MPU9250_AG_I2C_SLV2_CTRL_I2C_SLV2_GRP				(1<<4)
#define MPU9250_AG_I2C_SLV2_CTRL_I2C_SLV2_REG_DIS			(1<<5)
#define MPU9250_AG_I2C_SLV2_CTRL_I2C_SLV2_BYTE_SW			(1<<6)
#define MPU9250_AG_I2C_SLV2_CTRL_I2C_SLV2_EN				(1<<7)

#define MPU9250_AG_I2C_SLV3_ADDR		0x2E

#define MPU9250_AG_I2C_SLV3_I2C_ID_MASK						(0x7F)
#define MPU9250_AG_I2C_SLV3_I2C_SLV3_RNW					(1<<7)

#define MPU9250_AG_I2C_SLV3_REG			0x2F

#define MPU9250_AG_I2C_SLV3_CTRL		0x30

#define MPU9250_AG_I2C_SLV3_CTRL_I2C_SLV3_LENG_MASK			(0xF)
#define MPU9250_AG_I2C_SLV3_CTRL_I2C_SLV3_GRP				(1<<4)
#define MPU9250_AG_I2C_SLV3_CTRL_I2C_SLV3_REG_DIS			(1<<5)
#define MPU9250_AG_I2C_SLV3_CTRL_I2C_SLV3_BYTE_SW			(1<<6)
#define MPU9250_AG_I2C_SLV3_CTRL_I2C_SLV3_EN				(1<<7)

#define MPU9250_AG_I2C_SLV4_ADDR		0x31

#define MPU9250_AG_I2C_SLV4_I2C_ID_MASK						(0x7F)
#define MPU9250_AG_I2C_SLV4_I2C_SLV3_RNW					(1<<7)

#define MPU9250_AG_I2C_SLV4_REG			0x32

#define MPU9250_AG_I2C_SLV4_DO			0x33

#define MPU9250_AG_I2C_SLV4_CTRL		0x34

#define MPU9250_AG_I2C_SLV4_CTRL_I2C_MST_DLY_MASK			(0x1F)
#define MPU9250_AG_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS			(1<<5)
#define MPU9250_AG_I2C_SLV4_CTRL_SLV4_DONE_INT_EN			(1<<6)
#define MPU9250_AG_I2C_SLV4_CTRL_I2C_SLV4_EN				(1<<7)

#define MPU9250_AG_I2C_SLV4_DI			0x35

#define MPU9250_AG_I2C_MST_STATUS		0x36

#define MPU9250_AG_I2C_MST_STATUS_I2C_SLV0_NACK				(1<<0)
#define MPU9250_AG_I2C_MST_STATUS_I2C_SLV1_NACK				(1<<1)
#define MPU9250_AG_I2C_MST_STATUS_I2C_SLV2_NACK				(1<<2)
#define MPU9250_AG_I2C_MST_STATUS_I2C_SLV3_NACK				(1<<3)
#define MPU9250_AG_I2C_MST_STATUS_I2C_SLV4_NACK				(1<<4)
#define MPU9250_AG_I2C_MST_STATUS_I2C_LOST_ARB				(1<<5)
#define MPU9250_AG_I2C_MST_STATUS_I2C_SLV4_DONE				(1<<6)
#define MPU9250_AG_I2C_MST_STATUS_PASS_THROUGH				(1<<7)

#define MPU9250_AG_INT_PIN_CFG			0x37

#define MPU9250_AG_INT_PIN_CFG_BYPASS_EN					(1<<1)
#define MPU9250_AG_INT_PIN_CFG_FSYNC_INT_MODE_EN			(1<<2)
#define MPU9250_AG_INT_PIN_CFG_ACTL_FSYNC					(1<<3)
#define MPU9250_AG_INT_PIN_CFG_INT_ANYRD_2CLEAR				(1<<4)
#define MPU9250_AG_INT_PIN_CFG_LATCH_EN						(1<<5)
#define MPU9250_AG_INT_PIN_CFG_OPEN							(1<<6)
#define MPU9250_AG_INT_PIN_CFG_ACTL							(1<<7)

#define MPU9250_AG_INT_ENABLE			0x38

#define MPU9250_AG_INT_ENABLE_RAW_RDY_EN					(1<<0)
#define MPU9250_AG_INT_ENABLE_DMP_EN						(1<<1)	// Undocumented
#define MPU9250_AG_INT_ENABLE_FSYNC_INT_EN					(1<<3)
#define MPU9250_AG_INT_ENABLE_FIFO_OFLOW_EN					(1<<4)
#define MPU9250_AG_INT_ENABLE_WOM_EN						(1<<6)

#define MPU9250_AG_INT_STATUS			0x3A

#define MPU9250_AG_INT_STATUS_RAW_DATA_RDY_INT				(1<<0)
#define MPU9250_AG_INT_STATUS_FSYNC_INT						(1<<3)
#define MPU9250_AG_INT_STATUS_FIFO_OFLOW_INT				(1<<4)
#define MPU9250_AG_INT_STATUS_WOM_INT						(1<<6)

#define MPU9250_AG_ACCEL_XOUT_H			0x3B
#define MPU9250_AG_ACCEL_XOUT_L			0x3C
#define MPU9250_AG_ACCEL_YOUT_H			0x3D
#define MPU9250_AG_ACCEL_YOUT_L			0x3E
#define MPU9250_AG_ACCEL_ZOUT_H			0x3F
#define MPU9250_AG_ACCEL_ZOUT_L			0x40

#define MPU9250_AG_TEMP_OUT_H			0x41
#define MPU9250_AG_TEMP_OUT_L			0x42

#define MPU9250_AG_GYRO_XOUT_H			0x43
#define MPU9250_AG_GYRO_XOUT_L			0x44
#define MPU9250_AG_GYRO_YOUT_H			0x45
#define MPU9250_AG_GYRO_YOUT_L			0x46
#define MPU9250_AG_GYRO_ZOUT_H			0x47
#define MPU9250_AG_GYRO_ZOUT_L			0x48

#define MPU9250_AG_EXT_SENS_DATA_COUNT	24					// Array of 24 bytes
#define MPU9250_AG_EXT_SENS_DATA_00		0x49

#define MPU9250_AG_I2C_SLV0_DO			0x63
#define MPU9250_AG_I2C_SLV1_DO			0x64
#define MPU9250_AG_I2C_SLV2_DO			0x65
#define MPU9250_AG_I2C_SLV3_DO			0x66

#define MPU9250_AG_I2C_MST_DELAY_CTRL	0x67

#define MPU9250_AG_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN		(1<<0)
#define MPU9250_AG_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN		(1<<1)
#define MPU9250_AG_I2C_MST_DELAY_CTRL_I2C_SLV2_DLY_EN		(1<<2)
#define MPU9250_AG_I2C_MST_DELAY_CTRL_I2C_SLV3_DLY_EN		(1<<3)
#define MPU9250_AG_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN		(1<<4)
#define MPU9250_AG_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW		(1<<7)

#define MPU9250_AG_SIGNAL_PATH_RESET	0x68

#define MPU9250_AG_SIGNAL_PATH_RESET_TEMP_RST				(1<<0)
#define MPU9250_AG_SIGNAL_PATH_RESET_ACCEL_RST				(1<<1)
#define MPU9250_AG_SIGNAL_PATH_RESET_GYRO_RST				(1<<2)

#define MPU9250_AG_MOT_DETECT_CTRL		0x69

#define MPU9250_AG_MOT_DETECT_CTRL_ACCEL_INTEL_MODE			(1<<6)
#define MPU9250_AG_MOT_DETECT_CTRL_ACCEL_INTEL_EN			(1<<7)

#define MPU9250_AG_USER_CTRL			0x6A

#define MPU9250_AG_USER_CTRL_SIG_COND_RST					(1<<0)
#define MPU9250_AG_USER_CTRL_I2C_MST_RST					(1<<1)
#define MPU9250_AG_USER_CTRL_FIFO_RST						(1<<2)
#define MPU9250_AG_USER_CTRL_I2C_IF_DIS						(1<<4)
#define MPU9250_AG_USER_CTRL_I2C_MST_EN						(1<<5)
#define MPU9250_AG_USER_CTRL_FIFO_EN						(1<<6)
#define MPU9250_AG_USER_CTRL_DMP_EN							(1<<7)	// Undocumented

#define MPU9250_AG_PWR_MGMT_1			0x6B

#define MPU9250_AG_PWR_MGMT_1_CLKSEL_MASK					(7)
#define MPU9250_AG_PWR_MGMT_1_CLKSEL_INTERNAL				(0)
#define MPU9250_AG_PWR_MGMT_1_CLKSEL_AUTO					(1)
#define MPU9250_AG_PWR_MGMT_1_CLKSEL_STOP					(7)

#define MPU9250_AG_PWR_MGMT_1_PD_PTAT						(1<<3)
#define MPU9250_AG_PWR_MGMT_1_GYRO_STANDBY					(1<<4)
#define MPU9250_AG_PWR_MGMT_1_CYCLE							(1<<5)
#define MPU9250_AG_PWR_MGMT_1_SLEEP							(1<<6)
#define MPU9250_AG_PWR_MGMT_1_H_RESET						(1<<7)

#define MPU9250_AG_PWR_MGMT_2			0x6C

#define MPU9250_AG_PWR_MGMT_2_DIS_ZG						(1<<0)
#define MPU9250_AG_PWR_MGMT_2_DIS_YG						(1<<1)
#define MPU9250_AG_PWR_MGMT_2_DIS_XG						(1<<2)
#define MPU9250_AG_PWR_MGMT_2_DIS_ZA						(1<<3)
#define MPU9250_AG_PWR_MGMT_2_DIS_YA						(1<<4)
#define MPU9250_AG_PWR_MGMT_2_DIS_XA						(1<<5)

// Undocumented registers
#define MPU9250_DMP_MEM_BANKSEL			0x6D
#define MPU9250_DMP_MEM_STARTADDR		0x6E
#define MPU9250_DMP_MEM_RW				0x6F
#define MPU9250_DMP_PROG_START			0x70

#define MPU9250_AG_FIFO_COUNT_H			0x72
#define MPU9250_AG_FIFO_COUNT_L			0x73
#define MPU9250_AG_FIFO_R_W				0x74

#define MPU9250_AG_WHO_AM_I				0x75

#define MPU9250_AG_WHO_AM_I_ID			0x71

#define MPU9250_AG_XA_OFFSET_H			0x77
#define MPU9250_AG_XA_OFFSET_L			0x78
#define MPU9250_AG_YA_OFFSET_H			0x7A
#define MPU9250_AG_YA_OFFSET_L			0x7B
#define MPU9250_AG_ZA_OFFSET_H			0x7D
#define MPU9250_AG_ZA_OFFSET_L			0x7E

// Mag registers
// AK8963C
#define MPU9250_MAG_I2C_DEVADDR			0xC

#define MPU9250_MAG_WIA					0x00		// Mag device ID

#define MPU9250_MAG_WIA_DEVICE_ID							(0x48)

#define MPU9250_MAG_INFO				0x01

#define MPU9250_MAG_ST1					0x02		// Status 1

#define MPU9250_MAG_ST1_DRDY								(1<<0)
#define MPU9250_MAG_ST1_DOR									(1<<1)

#define MPU9250_MAG_HXL					0x03
#define MPU9250_MAG_HXH					0x04
#define MPU9250_MAG_HYL					0x05
#define MPU9250_MAG_HYH					0x06
#define MPU9250_MAG_HZL					0x07
#define MPU9250_MAG_HZH					0x08

#define MPU9250_MAG_ST2					0x09

#define MPU9250_MAG_ST2_HOFL								(1<<3)
#define MPU9250_MAG_ST2_BITM								(1<<4)

#define MPU9250_MAG_CTRL1				0x0A

#define MPU9250_MAG_CTRL1_MODE_MASK							(0xF)
#define MPU9250_MAG_CTRL1_MODE_PWRDOWN						(0)
#define MPU9250_MAG_CTRL1_MODE_SINGLE						(1)
#define MPU9250_MAG_CTRL1_MODE_8HZ							(2)
#define MPU9250_MAG_CTRL1_MODE_100HZ						(6)
#define MPU9250_MAG_CTRL1_MODE_EXT_TRIG						(4)
#define MPU9250_MAG_CTRL1_MODE_SELFTEST						(8)
#define MPU9250_MAG_CTRL1_MODE_FUSEROM_ACCESS				(0xF)
#define MPU9250_MAG_CTRL1_BIT_16							(1<<4)


#define MPU9250_MAG_CTRL2				0x0B

#define MPU9250_MAG_CTRL2_SRST								(1<<0)

#define MPU9250_MAG_ASTC				0x0C

#define MPU9250_MAG_ASTC_SELF								(1<<6)

#define MPU9250_MAG_TS1					0x0D
#define MPU9250_MAG_TS2					0x0E

#define MPU9250_MAG_I2CDIS				0x0F

#define MPU9250_MAG_ASAX				0x10
#define MPU9250_MAG_ASAY				0x11
#define MPU9250_MAG_ASAZ				0x12

#define MPU9250_MAG_MAX_FLUX_DENSITY	4912
#define MPU9250_ACC_MAX_RANGE			32767

#define MPU9250_DMP_MEM_PAGE_SIZE			256		// DMP memory page size

#pragma pack(push, 1)

#pragma pack(pop)

#ifdef __cplusplus

class AgmMpu9250 : public AccelSensor, public GyroSensor, public MagSensor {
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


	virtual bool Read(ACCELSENSOR_RAWDATA &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(ACCELSENSOR_DATA &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(GYROSENSOR_RAWDATA &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(GYROSENSOR_DATA &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(MAGSENSOR_RAWDATA &Data) { return MagSensor::Read(Data); }
	virtual bool Read(MAGSENSOR_DATA &Data) { return MagSensor::Read(Data); }

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	int Read(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	bool UpdateData();
	virtual void IntHandler();
	int GetFifoLen();
	int ReadFifo(uint8_t * const pBuff, int Len);
	void ResetFifo();
	bool InitDMP(uint32_t DmpStartAddr, uint8_t * const pDmpImage, int Len);

private:
	// Default base initialization. Does detection and set default config for all sensor.
	// All sensor init must call this first prio to initializing itself
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);
	bool UploadDMPImage(uint8_t * const pDmpImage, int Len);
	void EnableFifo();

	bool vbInitialized;
	bool vbDmpEnabled;
	uint8_t vMagCtrl1Val;
	int16_t vMagSenAdj[3];
	bool vbSensorEnabled[3];
	int vTemperature;
};

#endif // __cplusplus

#endif // __AGM_MPU9250_H__
