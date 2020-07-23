/**-------------------------------------------------------------------------
@file	am_lsm303agr.h

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

#ifndef __AM_LSM303AGR_H__
#define __AM_LSM303AGR_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/mag_sensor.h"
#include "sensors/temp_sensor.h"

#define LSM303AGR_ACCEL_I2C_DEVADDR			0x19	//!< 7 bit i2c address for Accel
#define LSM303AGR_MAG_I2C_DEVADDR			0x1E	//!< 7 bit i2c address for Mag

// ***** XL
#define LSM303AGR_WHO_AM_I_A_REG			0x0F
#define LSM303AGR_WHO_AM_I_A_ID									0x33

#define LSM303AGR_STATUS_REG_AUX_A_REG		0x7
#define LSM303AGR_STATUS_REG_AUX_A_TDA							(1<<2)	//!< Temperature new data avail
#define LSM303AGR_STATUS_REG_AUX_A_TOR							(1<<6)	//!< Temperature data overrun

#define LSM303AGR_OUT_TEMP_L_A_REG			0xC		//!< Temperature data
#define LSM303AGR_OUT_TEMP_H_A_REG			0xD		//!< Temperature data

#define LSM303AGR_INT_COUNTER_REG_A_REG		0xE

#define LSM303AGR_TEMP_CFG_REG_A_REG		0x1F
#define LSM303AGR_TEMP_CFG_REG_A_MASK							(3<<6)	//!< 00b-disable, 11b-enable
#define LSM303AGR_TEMP_CFG_REG_A_ENABLE							(3<<6)

#define LSM303AGR_CTRL_REG1_A_REG			0x20
#define LSM303AGR_CTRL_REG1_A_XEN								(1<<0)	//!< X axis enable
#define LSM303AGR_CTRL_REG1_A_YEN								(1<<1)	//!< Y axis enable
#define LSM303AGR_CTRL_REG1_A_ZEN								(1<<2)	//!< Z axis enable
#define LSM303AGR_CTRL_REG1_A_LPEN								(1<<3)	//!< Low power enable
#define LSM303AGR_CTRL_REG1_A_ODR_MASK							(0xF<<4)	//!< Output data rate
#define LSM303AGR_CTRL_REG1_A_ODR_PWRDOWN						(0<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_1HZ							(1<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_10HZ							(2<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_25HZ							(3<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_50HZ							(4<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_100HZ							(5<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_200HZ							(6<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_400HZ							(7<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_1620HZ						(8<<4)
#define LSM303AGR_CTRL_REG1_A_ODR_1344_5376HZ					(9<<4)	// Normal : 1.62KHz, LP : 5.376KHZ

#define LSM303AGR_CTRL_REG2_A_REG			0x21
#define LSM303AGR_CTRL_REG2_A_HPIS1								(1<<0)	//!< High pass filter enable for AOI on interrupt 1
#define LSM303AGR_CTRL_REG2_A_HPIS2								(1<<1)	//!< High pass filter enable for AOI on interrupt 2
#define LSM303AGR_CTRL_REG2_A_HPCLICK							(1<<2)	//!< High pass filter enable for CLICK
#define LSM303AGR_CTRL_REG2_A_FDS								(1<<3)	//!< High pass filter data selection
#define LSM303AGR_CTRL_REG2_A_HPCF_MASK							(3<<4)	//!< High pass filter cutoff freq
#define LSM303AGR_CTRL_REG2_A_HPM_MASK							(3<<6)	//!< High pass filter mode selection
#define LSM303AGR_CTRL_REG2_A_HPM_REF							(1<<6)
#define LSM303AGR_CTRL_REG2_A_HPM_NORMAL						(2<<6)
#define LSM303AGR_CTRL_REG2_A_HPM_AUTO_RESET					(3<<6)


#define LSM303AGR_CTRL_REG3_A_REG			0x22
#define LSM303AGR_CTRL_REG3_A_I1_OVERRUN						(1<<1)	//!< FIFO overrun on INT1
#define LSM303AGR_CTRL_REG3_A_I1_WTM							(1<<2)	//!< FIFO watermark on INT1
#define LSM303AGR_CTRL_REG3_A_I1_DRDY2							(1<<3)	//!< DRDY2 on INT1
#define LSM303AGR_CTRL_REG3_A_I1_DRDY1							(1<<4)	//!< DRDY1 on INT1
#define LSM303AGR_CTRL_REG3_A_I1_AOI2							(1<<5)	//!< AOI2 on interrupt on INT1
#define LSM303AGR_CTRL_REG3_A_I1_AOI1							(1<<6)	//!< AOI1 on interrupt INT1
#define LSM303AGR_CTRL_REG3_A_I1CLICK							(1<<7)	//!< CLICK on INT1

#define LSM303AGR_CTRL_REG4_A_REG			0x23
#define LSM303AGR_CTRL_REG4_A_SPI_3WIRE_ENABLE					(1<<0)	//!< SPI enable 3 wire
#define LSM303AGR_CTRL_REG4_A_ST_MASK							(3<<1)
#define LSM303AGR_CTRL_REG4_A_ST_NORMAL							(0<<1)
#define LSM303AGR_CTRL_REG4_A_ST_SELF_TEST0						(1<<1)
#define LSM303AGR_CTRL_REG4_A_ST_SELF_TEST1						(1<<1)
#define LSM303AGR_CTRL_REG4_A_HR								(1<<3)	//!< Operating mode selection
#define LSM303AGR_CTRL_REG4_A_FS_MASK							(3<<4)	//!< Full scale selection
#define LSM303AGR_CTRL_REG4_A_FS_2G								(0<<4)
#define LSM303AGR_CTRL_REG4_A_FS_4G								(1<<4)
#define LSM303AGR_CTRL_REG4_A_FS_8G								(2<<4)
#define LSM303AGR_CTRL_REG4_A_FS_16G							(3<<4)
#define LSM303AGR_CTRL_REG4_A_BLE								(1<<6)	//!< 0-Little, 1-Big endian
#define LSM303AGR_CTRL_REG4_A_BDU								(1<<7)	//!< Block data update continuous

#define LSM303AGR_CTRL_REG5_A_REG			0x24
#define LSM303AGR_CTRL_REG5_A_D4D_INT2							(1<<0)	//!< 4D enable on INT2
#define LSM303AGR_CTRL_REG5_A_LIR_INT2							(1<<1)	//!< Latch interrupt INT2
#define LSM303AGR_CTRL_REG5_A_D4D_INT1							(1<<2)	//!< 4D enable on INT1
#define LSM303AGR_CTRL_REG5_A_LIR_INT1							(1<<3)	//!< Latch interrupt INT1
#define LSM303AGR_CTRL_REG5_A_FIFO_EN							(1<<6)	//!< FIFO enable
#define LSM303AGR_CTRL_REG5_A_BOOT								(1<<7)	//!< Reboot accel

#define LSM303AGR_CTRL_REG6_A_REG			0x25
#define LSM303AGR_CTRL_REG6_A_H_LACTIVE							(1<<1)	//!< Interrupt active low
#define LSM303AGR_CTRL_REG6_A_P2_ACT							(1<<3)	//!< Activity interrupt on INT2 enable
#define LSM303AGR_CTRL_REG6_A_BOOT_I2							(1<<4)	//!< Boot on INT2 enable
#define LSM303AGR_CTRL_REG6_A_I2_INT2							(1<<5)	//!< Interrupt 2 enable on INT2
#define LSM303AGR_CTRL_REG6_A_I2_INT1							(1<<6)	//!< Interrupt 1 enable on INT2
#define LSM303AGR_CTRL_REG6_A_I2_CLICK							(1<<7)	//!< Click interrupt enable on INT2

#define LSM303AGR_REF_DATACAPTURE_A_REG		0x26

#define LSM303AGR_STATUS_REG_A_REG			0x27
#define LSM303AGR_STATUS_REG_A_XDA								(1<<0)	//!< X axis data avail
#define LSM303AGR_STATUS_REG_A_YDA								(1<<1)	//!< Y axis data avail
#define LSM303AGR_STATUS_REG_A_ZDA								(1<<2)	//!< Z axis data avail
#define LSM303AGR_STATUS_REG_A_ZYXDA							(1<<3)	//!< X, Y, Z axis data avail
#define LSM303AGR_STATUS_REG_A_XOR								(1<<4)	//!< X axis data overrun
#define LSM303AGR_STATUS_REG_A_YOR								(1<<5)	//!< Y axis data overrun
#define LSM303AGR_STATUS_REG_A_ZOR								(1<<6)	//!< Z axis data overrun
#define LSM303AGR_STATUS_REG_A_ZYXOR							(1<<7)	//!< X, Y, Z data overrun

#define LSM303AGR_OUT_X_L_A_REG				0x28
#define LSM303AGR_OUT_X_H_A_REG				0x29

#define LSM303AGR_OUT_Y_L_A_REG				0x2A
#define LSM303AGR_OUT_Y_H_A_REG				0x2B

#define LSM303AGR_OUT_Z_L_A_REG				0x2C
#define LSM303AGR_OUT_Z_H_A_REG				0x2D

#define LSM303AGR_FIFO_CTRL_REG				0x2E
#define LSM303AGR_FIFO_CTRL_FTH_MASK							(0x1F<<0)	//!< Fifo threshold
#define LSM303AGR_FIFO_CTRL_TR_INT1								(0<<5)		//!< Trigger selection on INT1
#define LSM303AGR_FIFO_CTRL_TR_INT2								(1<<5)		//!< Trigger selection on INT2
#define LSM303AGR_FIFO_CTRL_FM_MASK								(3<<6)		//!< Fifo mode
#define LSM303AGR_FIFO_CTRL_FM_BYPASS							(0<<6)		//!< Bypass
#define LSM303AGR_FIFO_CTRL_FM_FIFO								(1<<6)		//!< FIFO mode
#define LSM303AGR_FIFO_CTRL_FM_STREAM							(2<<6)		//!< Stream mode
#define LSM303AGR_FIFO_CTRL_FM_STREAM_TO_FIFO					(3<<6)		//!< Stream-to-FIFO mode

#define LSM303AGR_FIFO_SRC_REG				0x2F
#define LSM303AGR_FIFO_SRC_FSS_MASK								(0x1F<<0)	//!< Fifo stored data level
#define LSM303AGR_FIFO_SRC_EMPTY								(1<<5)		//!< Fifo empty
#define LSM303AGR_FIFO_SRC_OVRN									(1<<6)		//!< Fifo full
#define LSM303AGR_FIFO_SRC_WTM									(1<<7)		//!< Fifo exceed water mark

#define LSM303AGR_INT1_CFG_A_REG			0x30
#define LSM303AGR_INT1_CFG_A_XLIE								(1<<0)		//!< X low interrupt enable
#define LSM303AGR_INT1_CFG_A_XHIE								(1<<1)		//!< X high interrupt enable
#define LSM303AGR_INT1_CFG_A_YLIE								(1<<2)		//!< Y low interrupt enable
#define LSM303AGR_INT1_CFG_A_YHIE								(1<<3)		//!< Y high interrupt enable
#define LSM303AGR_INT1_CFG_A_ZLIE								(1<<4)		//!< Z low interrupt enable
#define LSM303AGR_INT1_CFG_A_ZHIE								(1<<5)		//!< Z high interrupt enable
#define LSM303AGR_INT1_CFG_A_6D									(1<<6)		//!< 6 direction detection enable
#define LSM303AGR_INT1_CFG_A_AOI								(1<<7)		//!< And/Or

#define LSM303AGR_INT1_SRC_A_REG			0x31
#define LSM303AGR_INT1_SRC_A_XL									(1<<0)
#define LSM303AGR_INT1_SRC_A_XH									(1<<1)
#define LSM303AGR_INT1_SRC_A_YL									(1<<2)
#define LSM303AGR_INT1_SRC_A_YH									(1<<3)
#define LSM303AGR_INT1_SRC_A_ZL									(1<<4)
#define LSM303AGR_INT1_SRC_A_ZH									(1<<5)
#define LSM303AGR_INT1_SRC_A_IA									(1<<6)		//!< Interrupt active

#define LSM303AGR_INT1_THS_A_REG			0x32
#define LSM303AGR_INT1_THS_A_MASK								(0x7F<<0)

#define LSM303AGR_INT1_DURATION_A_REG		0x33
#define LSM303AGR_INT1_DURATION_A_MASK							(0x7F<<0)

#define LSM303AGR_INT2_CFG_A_REG			0x34
#define LSM303AGR_INT2_CFG_A_XLIE								(1<<0)
#define LSM303AGR_INT2_CFG_A_XHIE								(1<<1)
#define LSM303AGR_INT2_CFG_A_YLIE								(1<<2)
#define LSM303AGR_INT2_CFG_A_YHIE								(1<<3)
#define LSM303AGR_INT2_CFG_A_ZLIE								(1<<4)
#define LSM303AGR_INT2_CFG_A_ZHIE								(1<<5)
#define LSM303AGR_INT2_CFG_A_6D									(1<<6)		//!< 6 direction detection enable
#define LSM303AGR_INT2_CFG_A_AOI								(1<<7)		//!< AND/OR combination of interrupt events

#define LSM303AGR_INT2_SRC_A_REG			0x35
#define LSM303AGR_INT2_SRC_A_XL									(1<<0)
#define LSM303AGR_INT2_SRC_A_XH									(1<<1)
#define LSM303AGR_INT2_SRC_A_YL									(1<<2)
#define LSM303AGR_INT2_SRC_A_YH									(1<<3)
#define LSM303AGR_INT2_SRC_A_ZL									(1<<4)
#define LSM303AGR_INT2_SRC_A_ZH									(1<<5)
#define LSM303AGR_INT2_SRC_A_IA									(1<<6)		//!< Interrupt active

#define LSM303AGR_INT2_THS_A_REG			0x36
#define LSM303AGR_INT2_THS_A_MASK								(0x7F<<0)

#define LSM303AGR_INT2_DURATION_A_REG		0x37
#define LSM303AGR_INT2_DURATION_A_MASK							(0x7F<<0)

#define LSM303AGR_CLICK_CFG_A_REG			0x38
#define LSM303AGR_CLICK_CFG_A_XS								(1<<0)		//!< Enable interrupt single click on X
#define LSM303AGR_CLICK_CFG_A_XD								(1<<1)		//!< Enable interrupt double click on X
#define LSM303AGR_CLICK_CFG_A_YS								(1<<2)		//!< Enable interrupt single click on Y
#define LSM303AGR_CLICK_CFG_A_YD								(1<<3)		//!< Enable interrupt double click on Y
#define LSM303AGR_CLICK_CFG_A_ZS								(1<<4)		//!< Enable interrupt single click on Z
#define LSM303AGR_CLICK_CFG_A_ZD								(1<<5)		//!< Enable interrupt double click on Z

#define LSM303AGR_CLICK_SRC_A_REG			0x39
#define LSM303AGR_CLICK_SRC_A_X									(1<<0)		//!< X click detected
#define LSM303AGR_CLICK_SRC_A_Y									(1<<1)		//!< Y click detected
#define LSM303AGR_CLICK_SRC_A_Z									(1<<2)		//!< Z click detected
#define LSM303AGR_CLICK_SRC_A_SIGN_NEG							(1<<3)		//!< Negative detection
#define LSM303AGR_CLICK_SRC_A_SCLICK							(1<<4)		//!< Single click enable
#define LSM303AGR_CLICK_SRC_A_DCLICK							(1<<5)		//!< Double click enable
#define LSM303AGR_CLICK_SRC_A_IA								(1<<6)		//!< Interrupt active

#define LSM303AGR_CLICK_THS_A_REG			0x3A
#define LSM303AGR_CLICK_THS_A_MASK								(0x7F<<0)

#define LSM303AGR_TIME_LIMIT_A_REG			0x3B
#define LSM303AGR_TIME_LIMIT_A_MASK								(0x7F<<0)

#define LSM303AGR_TIME_LATENCY_A_REG		0x3C

#define LSM303AGR_TIME_WINDOW_A_REG			0x3D

#define LSM303AGR_ACT_THS_A_REG				0x3E
#define LSM303AGR_ACT_THS_A_MASK								(0x7F<<0)

#define LSM303AGR_ACT_DUR_A_REG				0x3F

// ***** MAG
#define LSM303AGR_OFFSET_X_REG_L_M_REG		0x45
#define LSM303AGR_OFFSET_X_REG_H_M_REG		0x46
#define LSM303AGR_OFFSET_Y_REG_L_M_REG		0x47
#define LSM303AGR_OFFSET_Y_REG_H_M_REG		0x48
#define LSM303AGR_OFFSET_Z_REG_L_M_REG		0x49
#define LSM303AGR_OFFSET_Z_REG_H_M_REG		0x4A

#define LSM303AGR_WHO_AM_I_M_REG			0x4F
#define LSM303AGR_WHO_AM_I_M_ID									0x40

#define LSM303AGR_CTRL_REG_A_M_REG			0x60
#define LSM303AGR_CTRL_REG_A_M_MD_MASK							(3<<0)
#define LSM303AGR_CTRL_REG_A_M_MD_CONTINUOUS					(0<<0)
#define LSM303AGR_CTRL_REG_A_M_MD_SINGLE						(1<<0)
#define LSM303AGR_CTRL_REG_A_M_MD_IDLE							(3<<0)
#define LSM303AGR_CTRL_REG_A_M_ODR_MASK							(3<<2)
#define LSM303AGR_CTRL_REG_A_M_ODR_10HZ							(0<<2)
#define LSM303AGR_CTRL_REG_A_M_ODR_20HZ							(1<<2)
#define LSM303AGR_CTRL_REG_A_M_ODR_50HZ							(2<<2)
#define LSM303AGR_CTRL_REG_A_M_ODR_100HZ						(3<<2)
#define LSM303AGR_CTRL_REG_A_M_LP								(1<<4)
#define LSM303AGR_CTRL_REG_A_M_SOFT_RST							(1<<5)
#define LSM303AGR_CTRL_REG_A_M_REBOOT							(1<<6)
#define LSM303AGR_CTRL_REG_A_M_TEMP_EN							(1<<7)

#define LSM303AGR_CFG_REG_B_M_REG				0x61
#define LSM303AGR_CFG_REG_B_M_LPF								(1<<0)		//!< Low pass filter enable
#define LSM303AGR_CFG_REG_B_M_OFF_CANC							(1<<1)		//!< Enable offset cancellation
#define LSM303AGR_CFG_REG_B_M_SET_FREQ							(1<<2)		//!< Select freq of set pulse
#define LSM303AGR_CFG_REG_B_M_INT_ON_DATAOFF					(1<<3)		//!< Check data after hard iron correction
#define LSM303AGR_CFG_REG_B_M_OFF_CANC_ONE_SHOT					(1<<4)		//!< Enable offset cancellation in single measurement

#define LSM303AGR_CFG_REG_C_M_REG				0x62
#define LSM303AGR_CFG_REG_C_M_INT_MAG							(1<<0)		//!< Configure DRDY pin as digital output
#define LSM303AGR_CFG_REG_C_M_SELF_TEST							(1<<1)		//!< Enable self test
#define LSM303AGR_CFG_REG_C_M_BLE								(1<<3)		//!< Low/High
#define LSM303AGR_CFG_REG_C_M_BDU								(1<<4)		//!<
#define LSM303AGR_CFG_REG_C_M_I2C_DIS							(1<<5)		//!< Disable I2C
#define LSM303AGR_CFG_REG_C_M_INT_MAG_PIN						(1<<6)		//!< Enable interrupt on INT_MAG_PIN

#define LSM303AGR_INT_CTRL_REG_M_REG			0x63
#define LSM303AGR_INT_CTRL_REG_M_IEN							(1<<0)		//!< Interrupt enable
#define LSM303AGR_INT_CTRL_REG_M_IEL							(1<<1)		//!< Interrupt latch enable
#define LSM303AGR_INT_CTRL_REG_M_IEA							(1<<2)		//!< Interrupt polarity high
#define LSM303AGR_INT_CTRL_REG_M_ZIEN							(1<<5)
#define LSM303AGR_INT_CTRL_REG_M_YIEN							(1<<6)
#define LSM303AGR_INT_CTRL_REG_M_XIEN							(1<<7)

#define LSM303AGR_INT_SOURCE_REG_M_REG			0x64
#define LSM303AGR_INT_SOURCE_REG_M_INT							(1<<0)		//!< Interrupt occured
#define LSM303AGR_INT_SOURCE_REG_M_MROI							(1<<1)
#define LSM303AGR_INT_SOURCE_REG_M_NTH_Z						(1<<2)
#define LSM303AGR_INT_SOURCE_REG_M_NTH_Y						(1<<3)
#define LSM303AGR_INT_SOURCE_REG_M_NTH_X						(1<<4)
#define LSM303AGR_INT_SOURCE_REG_M_PTH_Z						(1<<5)
#define LSM303AGR_INT_SOURCE_REG_M_PTH_Y						(1<<6)
#define LSM303AGR_INT_SOURCE_REG_M_PTH_X						(1<<7)

#define LSM303AGR_INT_THS_L_REG_M_REG			0x65
#define LSM303AGR_INT_THS_H_REG_M_REG			0x66

#define LSM303AGR_STATUS_REG_M_REG				0x67
#define LSM303AGR_STATUS_REG_M_XDA								(1<<0)
#define LSM303AGR_STATUS_REG_M_YDA								(1<<1)
#define LSM303AGR_STATUS_REG_M_ZDA								(1<<2)
#define LSM303AGR_STATUS_REG_M_ZYXDA							(1<<3)
#define LSM303AGR_STATUS_REG_M_XOR								(1<<4)
#define LSM303AGR_STATUS_REG_M_YOR								(1<<5)
#define LSM303AGR_STATUS_REG_M_ZOR								(1<<6)
#define LSM303AGR_STATUS_REG_M_ZYXOR							(1<<7)

#define LSM303AGR_OUTX_L_REG_M_REG				0x68
#define LSM303AGR_OUTX_H_REG_M_REG				0x69
#define LSM303AGR_OUTY_L_REG_M_REG				0x6A
#define LSM303AGR_OUTY_H_REG_M_REG				0x6B
#define LSM303AGR_OUTZ_L_REG_M_REG				0x6C
#define LSM303AGR_OUTZ_H_REG_M_REG				0x6D

#define LSM303AGR_MAG_SENSITTIVITY				150		// nanoTesla per bit

// Accel, Mag, Temperature
class AccelLsm303agr : public AccelSensor, public TempSensor  {
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
	virtual bool Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);
	virtual uint8_t Scale(uint8_t Value);

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t FilterFreq(uint32_t Freq);


	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual void PowerOff();

	virtual bool Read(ACCELSENSOR_RAWDATA &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(ACCELSENSOR_DATA &Data) { return AccelSensor::Read(Data); }

	bool UpdateData();

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
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);

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
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

	/**
	 * @brief	Interrupt handler (optional)
	 *
	 * Sensor that supports interrupt can implement this to handle interrupt.
	 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
	 */
	virtual void IntHandler();
	virtual bool StartSampling() { return true; }

private:
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	int32_t vRShift;	//!< Data are left justify. This var contains shift right value to right justify
};

class MagLsm303agr : public MagSensor  {
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
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	uint32_t SamplingFrequency(uint32_t Freq);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	bool UpdateData();

	virtual bool Read(MAGSENSOR_RAWDATA &Data) { return MagSensor::Read(Data); }
	virtual bool Read(MAGSENSOR_DATA &Data) { return MagSensor::Read(Data); }

	/**
	 * @brief	Interrupt handler (optional)
	 *
	 * Sensor that supports interrupt can implement this to handle interrupt.
	 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
	 */
	virtual void IntHandler();
	virtual bool StartSampling() { return true; }

private:

	int16_t vOffset[3];
};



#endif // __AM_LSM303AGR_H__
