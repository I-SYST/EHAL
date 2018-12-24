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

#define ICM20948_REG_BANK_SEL			127

#define ICM20948_REG_BANK_SEL_USER_BANK_BITPOS		(4)
#define ICM20948_REG_BANK_SEL_USER_BANK_MASK		(3<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_0			(0<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_1			(1<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_2			(2<<4)
#define ICM20948_REG_BANK_SEL_USER_BANK_3			(3<<4)

// concatenate bank select to second byte of register address making
// register address a 16 bits value
#define ICM20948_REG_BANK0				(0<<8)
#define ICM20948_REG_BANK1				(1<<8)
#define ICM20948_REG_BANK2				(2<<8)
#define ICM20948_REG_BANK3				(3<<8)

//*** Register Bank 0

#define ICM20948_WHO_AM_I				(ICM20948_REG_BANK0 | 0)

#define ICM20948_WHO_AM_I_ID						(0xEA)

#define ICM20948_USER_CTRL				(ICM20948_REG_BANK0 | 3)

#define ICM20948_USER_CTRL_I2C_MST_RST				(1<<1)	// Reset I2C Master module
#define ICM20948_USER_CTRL_SRAM_RST					(1<<2)	// Reset SRAM module
#define ICM20948_USER_CTRL_DMP_RST					(1<<3)	// Reset DMP module
#define ICM20948_USER_CTRL_I2C_IF_DIS				(1<<4)	// Reset and disable I2C Slave module
#define ICM20948_USER_CTRL_I2C_MST_EN				(1<<5)	// Enable I2C master module
#define ICM20948_USER_CTRL_FIFO_EN					(1<<6)	// Enable FIFO operation mode
#define ICM20948_USER_CTRL_DMP_EN					(1<<7)	// Enable DMP feature

#define ICM20948_LP_CONFIG				(ICM20948_REG_BANK0 | 5)

#define ICM20948_LP_CONFIG_GYRO_CYCLE				(1<<4)	// Operate Gyro in duty cycled mode
#define ICM20948_LP_CONFIG_ACCEL_CYCLE				(1<<5)	// Operate Accel in duty cycled mode
#define ICM20948_LP_CONFIG_I2C_MST_CYCLE			(1<<4)	// Operate I2C master in duty cycled mode

#define ICM20948_PWR_MGMT_1				(ICM20948_REG_BANK0 | 6)

#define ICM20948_PWR_MGMT_1_CLKSEL_MASK				(7<<0)	// Clock source
#define ICM20948_PWR_MGMT_1_CLKSEL_BITPOS			(0)
#define ICM20948_PWR_MGMT_1_CLKSEL_STOP				(7<<0)
#define ICM20948_PWR_MGMT_1_TEMP_DIS				(1<3)	// Disable temperature sensor
#define ICM20948_PWR_MGMT_1_LP_EN					(1<<5)	// Low Power enable
#define ICM20948_PWR_MGMT_1_SLEEP					(1<<6)	// Enter sleep
#define ICM20948_PWR_MGMT_1_DEVICE_RESET			(1<<7)	// Reset to default settings

#define ICM20948_PWR_MGMT_2				(ICM20948_REG_BANK0 | 7)

#define ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK		(7<<0)	// 0 : Gyro on, 7 : Gyro off
#define ICM20948_PWR_MGMT_2_DISABLE_GYRO_BITPOS		(0)
#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK		(7<<3)	// 0 : Accel om, 7 Accel off
#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL_BITPOS	(3)

#define ICM20948_INT_PIN_CFG			(ICM20948_REG_BANK0 | 15)

#define ICM20948_INT_PIN_CFG_BYPASS_EN				(1<<1)	// I2C master in bypass mode
#define ICM20948_INT_PIN_CFG_FSYNC_INT_MODE_EN		(1<<2)	// FSYNC pin used as interrupt
#define ICM20948_INT_PIN_CFG_ATCL_FSYNC				(1<<3)	// Logic level of FSYNC Interrupt. 1 : Active low, 0 : active high.
#define ICM20948_INT_PIN_CFG_INT_ANYRD_2CLEAR		(1<<4)	// Clear interrupt status
#define ICM20948_INT_PIN_CFG_INT1_LATCH_EN			(1<<5)	// Latch INT1
#define ICM20948_INT_PIN_CFG_INT1_OPEN				(1<<6)	// Open drain
#define ICM20948_INT_PIN_CFG_INT1_ACTL				(1<<7)	// Logic level.  1 : active low, 0 : active high

#define ICM20948_INT_ENABLE				(ICM20948_REG_BANK0 | 16)

#define ICM20948_INT_ENABLE_I2C_MST_INT_EN			(1<<0)	// Enable I2C master interrupt on pin 1
#define ICM20948_INT_ENABLE_DMP_INT1_EN				(1<<1)	// Enable DMP interrupt on pin 1
#define ICM20948_INT_ENABLE_PLL_RDY_EN				(1<<2)	// Enable interrupt on pin 1
#define ICM20948_INT_ENABLE_WOM_INT_EN				(1<<3)	// Enable wake on motion interrupt on pin 1
#define ICM20948_INT_ENABLE_REG_WOF_EN				(1<<7)	// Enable wake on FSYNC interrupt

#define ICM20948_INT_ENABLE_1			(ICM20948_REG_BANK0 | 17)

#define ICM20948_INT_ENABLE_1_RAW_DATA_0_DRY_EN		(1<<0)	// Enable raw data ready interrupt on pin 1

#define ICM20948_INT_ENABLE_2			(ICM20948_REG_BANK0 | 18)

#define ICM20948_INT_ENABLE_2_FIFO_OVERFLOW_EN_MASK	(0xf<<0)// Enable FIFO overflow interrupt on pin 1

#define ICM20948_INT_ENABLE_3			(ICM20948_REG_BANK0 | 19)

#define ICM20948_INT_ENABLE_3_FIFO_WM_EN_MASK		(0xf<<0)// Enable FIFO watermark interrupt on pin 1

#define ICM20948_I2C_MST_STATUS			(ICM20948_REG_BANK0 | 23)

#define ICM20948_I2C_MST_STATUS_I2C_SLV0_NACK		(1<<0)	// Slave 0 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV1_NACK		(1<<1)	// Slave 1 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV2_NACK		(1<<2)	// Slave 2 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV3_NACK		(1<<3)	// Slave 3 NACK
#define ICM20948_I2C_MST_STATUS_I2C_SLV4_NACK		(1<<4)	// Slave 4 NACK
#define ICM20948_I2C_MST_STATUS_I2C_LOST_ARB		(1<<5)	// Lost arbitration
#define ICM20948_I2C_MST_STATUS_I2C_SLV4_DONE		(1<<6)	// Slave 4 transfer complete
#define ICM20948_I2C_MST_STATUS_PASS_THROUGH		(1<<7)	// FSYNC interrupt flag

#define ICM20948_INT_STATUS				(ICM20948_REG_BANK0 | 25)

#define ICM20948_INT_STATUS_I2C_MIST_INT			(1<<0)	// I2C master interrupt flag
#define ICM20948_INT_STATUS_DMP_INT1				(1<<1)	// DMP interrupt on pin 1
#define ICM20948_INT_STATUS_PLL_RDY_INT				(1<<2)	// PLL enable & ready interrupt flag
#define ICM20948_INT_STATUS_WOM_INT					(1<<3)	// Wake on motion interrupt flag

#define ICM20948_INT_STATUS_1			(ICM20948_REG_BANK0 | 26)

#define ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT	(1<<0)	// Raw date ready interrupt

#define ICM20948_INT_STATUS_2			(ICM20948_REG_BANK0 | 27)

#define ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK	(0xf<<0)	// FIFO overflow interrupt

#define ICM20948_INT_STATUS_3			(ICM20948_REG_BANK0 | 28)

#define ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK	(0xf<<0)	// Watermark interrupt for FIFO

#define ICM20948_DELAY_TIMEH			(ICM20948_REG_BANK0 | 40)
#define ICM20948_DELAY_TIMEL			(ICM20948_REG_BANK0 | 41)

#define ICM20948_ACCEL_XOUT_H			(ICM20948_REG_BANK0 | 45)
#define ICM20948_ACCEL_XOUT_L			(ICM20948_REG_BANK0 | 46)
#define ICM20948_ACCEL_YOUT_H			(ICM20948_REG_BANK0 | 47)
#define ICM20948_ACCEL_YOUT_L			(ICM20948_REG_BANK0 | 48)
#define ICM20948_ACCEL_ZOUT_H			(ICM20948_REG_BANK0 | 49)
#define ICM20948_ACCEL_ZOUT_L			(ICM20948_REG_BANK0 | 50)

#define ICM20948_GYRO_XOUT_H			(ICM20948_REG_BANK0 | 51)
#define ICM20948_GYRO_XOUT_L			(ICM20948_REG_BANK0 | 52)
#define ICM20948_GYRO_YOUT_H			(ICM20948_REG_BANK0 | 53)
#define ICM20948_GYRO_YOUT_L			(ICM20948_REG_BANK0 | 54)
#define ICM20948_GYRO_ZOUT_H			(ICM20948_REG_BANK0 | 55)
#define ICM20948_GYRO_ZOUT_L			(ICM20948_REG_BANK0 | 56)

#define ICM20948_TEMP_OUT_H				(ICM20948_REG_BANK0 | 57)
#define ICM20948_TEMP_OUT_L				(ICM20948_REG_BANK0 | 58)

// External I2C access
#define ICM20948_EXT_SLV_SENS_DATA_00	(ICM20948_REG_BANK0 | 59)
#define ICM20948_EXT_SLV_SENS_DATA_23	(ICM20948_REG_BANK0 | 82)
#define ICM20948_EXT_SLV_SENS_DATA_MAX	(ICM20948_EXT_SLV_SENS_DATA_23 - ICM20948_EXT_SLV_SENS_DATA_00)

#define ICM20948_FIFO_EN_1				(ICM20948_REG_BANK0 | 102)

#define ICM20948_FIFO_EN_1_SLV_0_FIFO_EN		(1<<0)	// Enable Slave 0 FIFO
#define ICM20948_FIFO_EN_1_SLV_1_FIFO_EN		(1<<1)	// Enable Slave 1 FIFO
#define ICM20948_FIFO_EN_1_SLV_2_FIFO_EN		(1<<2)	// Enable Slave 2 FIFO
#define ICM20948_FIFO_EN_1_SLV_3_FIFO_EN		(1<<3)	// Enable Slave 3 FIFO

#define ICM20948_FIFO_EN_2				(ICM20948_REG_BANK0 | 103)

#define ICM20948_FIFO_EN_2_TEMP_FIFO_EN			(1<<0)	// Temp out fifo enable
#define ICM20948_FIFO_EN_2_GYRO_X_FIFO_EN		(1<<1)	// Gyro X out fifo enable
#define ICM20948_FIFO_EN_2_GYRO_Y_FIFO_EN		(1<<2)	// Gyro Y out fifo enable
#define ICM20948_FIFO_EN_2_GYRO_Z_FIFO_EN		(1<<3)	// Gyro Z out fifo enable
#define ICM20948_FIFO_EN_2_ACCEL_FIFO_EN		(1<<1)	// Accel out fifo enable

#define ICM20948_FIFO_RST				(ICM20948_REG_BANK0 | 104)

#define ICM20948_FIFO_RST_FIFO_RESET_MASK		(0x1f<<0)	// Software fifo reset

#define ICM20948_FIFO_MODE				(ICM20948_REG_BANK0 | 105)

#define ICM20948_FIFO_MODE_FIFO_MODE_MASK		(0x1f<<0)	//

#define ICM20948_FIFO_COUNTH			(ICM20948_REG_BANK0 | 112)

#define ICM20948_FIFO_COUNTH_FIFO_CNT_MASK		(0x1f<<0)

#define ICM20948_FIFO_COUNTL			(ICM20948_REG_BANK0 | 113)

#define ICM20948_FIFO_R_W				(ICM20948_REG_BANK0 | 114)

#define ICM20948_DATA_RDY_STATUS		(ICM20948_REG_BANK0 | 116)

#define ICM20948_DATA_RDY_STATUS_RAW_DATA_RDY_MASK	(0xf<<0)
#define ICM20948_DATA_RDY_STATUS_WOF_STATUS			(1<<7)		// Wake on FSYNC interrupt status

#define ICM20948_FIFO_CFG				(ICM20948_REG_BANK0 | 118)

#define ICM20948_FIFO_CFG_FIFO_CFG					(1<<0)		// Set to 1 of interrupt status for each sensor is required

//*** Register Bank 1

#define ICM20948_SELF_TEST_X_GYRO		(ICM20948_REG_BANK1 | 2)
#define ICM20948_SELF_TEST_Y_GYRO		(ICM20948_REG_BANK1 | 3)
#define ICM20948_SELF_TEST_Z_GYRO		(ICM20948_REG_BANK1 | 4)

#define ICM20948_SELF_TEST_X_ACCEL		(ICM20948_REG_BANK1 | 14)
#define ICM20948_SELF_TEST_Y_ACCEL		(ICM20948_REG_BANK1 | 15)
#define ICM20948_SELF_TEST_Z_ACCEL		(ICM20948_REG_BANK1 | 16)

#define ICM20948_XA_OFFS_H				(ICM20948_REG_BANK1 | 20)
#define ICM20948_XA_OFFS_L				(ICM20948_REG_BANK1 | 21)

#define ICM20948_YA_OFFS_H				(ICM20948_REG_BANK1 | 23)
#define ICM20948_YA_OFFS_L				(ICM20948_REG_BANK1 | 24)

#define ICM20948_ZA_OFFS_H				(ICM20948_REG_BANK1 | 26)
#define ICM20948_ZA_OFFS_L				(ICM20948_REG_BANK1 | 27)

#define ICM20948_TIMEBASE_CORRECTION_PLL	(ICM20948_REG_BANK1 | 40)

//*** Register Bank 2

#define ICM20948_GYRO_SMPLRT_DIV		(ICM20948_REG_BANK2 | 0)	// Gyro sample rate divider

#define ICM20948_GYRO_CONFIG_1			(ICM20948_REG_BANK2 | 1)

#define ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE					(1<<0)

#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_MASK				(3<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_250DPS			(0<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_500DPS			(1<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS			(2<<1)
#define ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_2000DPS			(3<<1)

#define ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_MASK			(7<<3)

#define ICM20948_GYRO_CONFIG_2			(ICM20948_REG_BANK2 | 2)

#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_MASK				(7<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_1X				(0<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_2X				(1<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_4X				(2<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_8X				(3<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_16X				(4<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_32X				(5<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_64X				(6<<0)
#define ICM20948_GYRO_CONFIG_2_GYRO_AVGCFG_128X				(7<<0)

#define ICM20948_GYRO_CONFIG_2_ZGYRO_CTEN					(1<<3)	// Z Gyro self test enable
#define ICM20948_GYRO_CONFIG_2_YGYRO_CTEN					(1<<4)	// Z Gyro self test enable
#define ICM20948_GYRO_CONFIG_2_XGYRO_CTEN					(1<<5)	// Z Gyro self test enable

#define ICM20948_XG_OFFS_USRH			(ICM20948_REG_BANK2 | 3)
#define ICM20948_XG_OFFS_USRL			(ICM20948_REG_BANK2 | 4)
#define ICM20948_YG_OFFS_USRH			(ICM20948_REG_BANK2 | 5)
#define ICM20948_YG_OFFS_USRL			(ICM20948_REG_BANK2 | 6)
#define ICM20948_ZG_OFFS_USRH			(ICM20948_REG_BANK2 | 7)
#define ICM20948_ZG_OFFS_USRL			(ICM20948_REG_BANK2 | 8)

#define ICM20948_ODR_ALIGN_EN			(ICM20948_REG_BANK2 | 9)

#define ICM20948_ODR_ALIGN_EN_ODR_ALIGN_EN					(1<<0)	// Enable ODR start time alignment

#define ICM20948_ACCEL_SMPLRT_DIV_1		(ICM20948_REG_BANK2 | 16)

#define ICM20948_ACCEL_SMPLRT_DIV_1_ACCEL_SMPLRT_DIV_MASK	(0xf<<0)

#define ICM20948_ACCEL_SMPLRT_DIV_2		(ICM20948_REG_BANK2 | 17)

#define ICM20948_ACCEL_INTEL_CTRL		(ICM20948_REG_BANK2 | 18)

#define ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_INT		(1<<0)	// Select WOM algorithm
#define ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN			(1<<1)	// Enable WOM logic

#define ICM20948_ACCEL_WOM_THR			(ICM20948_REG_BANK2 | 19)

#define ICM20948_ACCEL_CONFIG			(ICM20948_REG_BANK2 | 20)

#define ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE					(1<<0)	// Enable accel DLPF

#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_MASK				(3<<1)	// Full scale select mask
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_2G				(0<<1)	// Full scale select 2g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_4G				(1<<1)	// Full scale select 4g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_8G				(2<<1)	// Full scale select 8g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_16G				(3<<1)	// Full scale select 16g

#define ICM20948_ACCEL_CONFIG_DLPFCFG_MASK					(7<<3)	// Low pass filter config

#define ICM20948_ACCEL_CONFIG_2			(ICM20948_REG_BANK2 | 21)

#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_MASK				(3<<0)	// Control the number of sample averaged
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_4					(0<<0)	// 1 or 4 samples
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_8					(1<<0)	// 8 samples
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_16					(2<<0)	// 16 samples
#define ICM20948_ACCEL_CONFIG_2_DEC3_CFG_32					(3<<0)	// 32 samples

#define ICM20948_ACCEL_CONFIG_2_AZ_ST_EN_REG				(1<<2)	// Z accel self test enable
#define ICM20948_ACCEL_CONFIG_2_AY_ST_EN_REG				(1<<3)	// Y accel self test enable
#define ICM20948_ACCEL_CONFIG_2_AX_ST_EN_REG				(1<<4)	// X accel self test enable

#define ICM20948_FSYNC_CONFIG			(ICM20948_REG_BANK2 | 82)

#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_MASK				(0xf<<0)// Enable FSYNC pin data to be sampled
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_DIS				(0<<0)	// Disable FSYNC
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_TEMP_OUT_L		(1<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_GYRO_XOUT_L		(2<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_GYRO_YOUT_L		(3<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_GYRO_ZOUT_L		(4<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_ACCEL_XOUT_L		(5<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_ACCEL_YOUT_L		(6<<0)
#define ICM20948_FSYNC_CONFIG_EXT_SYNC_SET_ACCEL_ZOUT_L		(7<<0)

#define ICM20948_FSYNC_CONFIG_WOF_EDGE_INT					(1<<4)	// FSYNC interrupt level
#define ICM20948_FSYNC_CONFIG_WOF_DEGLITCH_EN				(1<<5)	// Enable digital deglitching of FSYNC inpiut for Wake on FSYNC
#define ICM20948_FSYNC_CONFIG_DELAY_TIME_EN					(1<<7)	// Enable delay time measurement between FSYNC event and the first ODR event

#define ICM20948_TEMP_CONFIG			(ICM20948_REG_BANK2 | 83)

#define ICM20948_TEMP_CONFIG_MASK							(7<<0)	// Low pass filter for temperature sensor

#define ICM20948_MOD_CTRL_USR			(ICM20948_REG_BANK2 | 84)

#define ICM20948_MOD_CTRL_USR_REG_LP_DMP_EN					(1<<0)	// Enable turning on DMP in low power accel mode

//*** Register Bank 3

#define ICM20948_I2C_MST_ODR_CONFIG		(ICM20948_REG_BANK3 | 0)

#define ICM20948_I2C_MST_ODR_CONFIG_MASK					(0xf<<0)

#define ICM20948_I2C_MST_CTRL			(ICM20948_REG_BANK3 | 1)

#define ICM20948_I2C_MST_CTRL_I2C_MST_CLK_MASK				(0xf<<0)// I2C master clock freq

#define ICM20948_I2C_MST_CTRL_I2C_MST_P_NSR					(1<<4)	// Control I2C master transition from one slave to the next
#define ICM20948_I2C_MST_CTRL_MULT_MST_EN					(1<<7)	// Enable multi-master capability

#define ICM20948_I2C_MST_DELAY_CTRL		(ICM20948_REG_BANK3 | 2)

#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV0_DELAY_EN		(1<<0)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV1_DELAY_EN		(1<<1)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV2_DELAY_EN		(1<<2)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV3_DELAY_EN		(1<<3)
#define ICM20948_I2C_MST_DELAY_CTRL_I2C_SLV4_DELAY_EN		(1<<4)
#define ICM20948_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW			(1<<7)	// Delays shadowing of external sensor data

#define ICM20948_I2C_SLV0_ADDR			(ICM20948_REG_BANK3 | 3)

#define ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK				(0x7f<<0)	// Physical I2C address slave 0
#define ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV0_ADDR_I2C_SLV0_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV0_REG			(ICM20948_REG_BANK3 | 4)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV0_CTRL			(ICM20948_REG_BANK3 | 5)

#define ICM20948_I2C_SLV_MAXLEN								(15)
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave

#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_GRP					(1<<4)	//
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN					(1<<7)	// Enable reading data from slave

#define ICM20948_I2C_SLV0_DO			(ICM20948_REG_BANK3 | 6)						// Data out to slave

#define ICM20948_I2C_SLV1_ADDR			(ICM20948_REG_BANK3 | 7)

#define ICM20948_I2C_SLV1_ADDR_I2C_ID_1_MASK				(0x7f<<0)	// Physical I2C address slave 1
#define ICM20948_I2C_SLV1_ADDR_I2C_SLV1_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV1_ADDR_I2C_SLV1_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV1_REG			(ICM20948_REG_BANK3 | 8)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV1_CTRL			(ICM20948_REG_BANK3 | 9)

#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave

#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_GRP					(1<<4)	//
#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV1_CTRL_I2C_SLV1_EN					(1<<7)

#define ICM20948_I2C_SLV1_DO			(ICM20948_REG_BANK3 | 10)						// Data out to slave

#define ICM20948_I2C_SLV2_ADDR			(ICM20948_REG_BANK3 | 11)

#define ICM20948_I2C_SLV2_ADDR_I2C_ID_2_MASK				(0x7f<<0)	// Physical I2C address slave 2
#define ICM20948_I2C_SLV2_ADDR_I2C_SLV2_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV2_ADDR_I2C_SLV2_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV2_REG			(ICM20948_REG_BANK3 | 12)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV2_CTRL			(ICM20948_REG_BANK3 | 13)

#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave

#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_GRP					(1<<4)	//
#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV2_CTRL_I2C_SLV2_EN					(1<<7)

#define ICM20948_I2C_SLV2_DO			(ICM20948_REG_BANK3 | 14)						// Data out to slave

#define ICM20948_I2C_SLV3_ADDR			(ICM20948_REG_BANK3 | 15)

#define ICM20948_I2C_SLV3_ADDR_I2C_ID_3_MASK				(0x7f<<0)	// Physical I2C address slave 3
#define ICM20948_I2C_SLV3_ADDR_I2C_SLV3_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV3_ADDR_I2C_SLV3_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV3_REG			(ICM20948_REG_BANK3 | 16)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV3_CTRL			(ICM20948_REG_BANK3 | 17)

#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave

#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_GRP					(1<<4)	//
#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV3_CTRL_I2C_SLV3_EN					(1<<7)

#define ICM20948_I2C_SLV3_DO			(ICM20948_REG_BANK3 | 18)						// Data out to slave

#define ICM20948_I2C_SLV4_ADDR			(ICM20948_REG_BANK3 | 19)

#define ICM20948_I2C_SLV4_ADDR_I2C_ID_4_MASK				(0x7f<<0)	// Physical I2C address slave 4
#define ICM20948_I2C_SLV4_ADDR_I2C_SLV4_RD					(1<<7)	// Read transfer
#define ICM20948_I2C_SLV4_ADDR_I2C_SLV4_WR					(0<<7)	// Read transfer

#define ICM20948_I2C_SLV4_REG			(ICM20948_REG_BANK3 | 20)						// I2C slave 0 register address from where to begin data transfer.

#define ICM20948_I2C_SLV4_CTRL			(ICM20948_REG_BANK3 | 21)

#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_LENG_MASK			(0xf<<0)	// Number of bytes to be read from I2C slave

#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_GRP					(1<<4)	//
#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS				(1<<5)	// When set, the transaction does not write a register value,
																// it will only read data, or write data
#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_BYTE_SW				(1<<6)
#define ICM20948_I2C_SLV4_CTRL_I2C_SLV4_EN					(1<<7)

#define ICM20948_I2C_SLV4_DO			(ICM20948_REG_BANK3 | 22)						// Data out to slave

#define ICM20948_I2C_SLV4_DI			(ICM20948_REG_BANK3 | 23)

//*** Mag registers AK09916

#define AK09916_I2C_ADDR1				0xC						// AK09916 I2C 7 bits address
#define AK09916_I2C_ADDR2				0xE						// AK09916 I2C 7 bits address

#define ICM20948_AK09916_WIA1			0x0						// Device ID

#define ICM20948_AK09916_WIA1_ID							(0x48)

#define ICM20948_AK09916_WIA2			0x1						// Device ID

#define ICM20948_AK09916_WIA2_ID							(9)

#define ICM20948_AK09916_ST1			0x10

#define ICM20948_ST1_DRDY									(1<<0)	// Data ready
#define ICM20948_ST1_DOR									(1<<1)	// Data skipped

#define ICM20948_AK09916_HXL			0x11
#define ICM20948_AK09916_HXH			0x12
#define ICM20948_AK09916_HYL			0x13
#define ICM20948_AK09916_HYH			0x14
#define ICM20948_AK09916_HZL			0x15
#define ICM20948_AK09916_HZH			0x16

#define ICM20948_ST2					0x18

#define ICM20948_ST2_HOFL									(1<<3)	// Magnetic sensor overflow

#define ICM20948_AK09916_CNTL2			0x31

#define ICM20948_AK09916_CNTL2_MODE_MASK					(0x1f<<0)
#define ICM20948_AK09916_CNTL2_MODE_PWRDWN					(0<<0)	// Power down
#define ICM20948_AK09916_CNTL2_MODE_SINGLE					(1<<0)	// Single measurement
#define ICM20948_AK09916_CNTL2_MODE_CONT1					(2<<0)	// Continuous mode 1
#define ICM20948_AK09916_CNTL2_MODE_CONT2					(4<<0)	// Continuous mode 2
#define ICM20948_AK09916_CNTL2_MODE_CONT3					(6<<0)	// Continuous mode 3
#define ICM20948_AK09916_CNTL2_MODE_CONT4					(8<<0)	// Continuous mode 4
#define ICM20948_AK09916_CNTL2_MODE_SELFTEST				(0x10<<0)	// Self test

#define ICM20948_AK09916_CNTL3			0x32

#define ICM20948_AK09916_CNTL3_SRST							(1<<0)	// Soft-reset


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

	int Read(uint16_t RegAddr, uint8_t *pBuff, int BuffLen) {
		return Read((uint8_t*)&RegAddr, 2, pBuff, BuffLen);
	}
	int Write(uint16_t RegAddr, uint8_t *pData, int DataLen) {
		return Write((uint8_t*)&RegAddr, 2, pData, DataLen);
	}

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	int Read(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	bool SelectBank(uint8_t BankNo);

	bool UpdateData();
	virtual void IntHandler();

private:
	// Default base initialization. Does detection and set default config for all sensor.
	// All sensor init must call this first prio to initializing itself
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	bool vbInitialized;
	uint8_t vMagCtrl1Val;
	int16_t vMagSenAdj[3];
	uint8_t vCurrBank;
};


#endif // __AGM_ICM20948_H__

