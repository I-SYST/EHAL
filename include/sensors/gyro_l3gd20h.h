/**-------------------------------------------------------------------------
@file	gyro_l3gd20h.h

@brief	Implementation of ST L3GD20H gyro sensor


@author	Hoang Nguyen Hoan
@date	Aug. 2, 2019

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

#ifndef __GYRO_L3GD20H_H__
#define __GYRO_L3GD20H_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/gyro_sensor.h"
#include "sensors/temp_sensor.h"

#define L3GD20H_I2C_7BITS_DEVADDR0			0x6A	//!< 7 bit i2c address SA0 = 0
#define L3GD20H_I2C_7BITS_DEVADDR1			0x6B	//!< 7 bit i2c address SA0 = 1

#define L3GD20H_WHO_AM_I_A_REG				0x0F
#define L3GD20H_WHO_AM_I_A_ID									0xD7
#define L3GD20_WHO_AM_I_A_ID									0xD4		//!< Old ID


#define L3GD20H_CTRL1_REG					0x20

#define L3GD20H_CTRL1_XEN										(1<<0)		//!< X axis enable
#define L3GD20H_CTRL1_YEN										(1<<1)		//!< Y axis enable
#define L3GD20H_CTRL1_ZEN										(1<<2)		//!< Z axis enable
#define L3GD20H_CTRL1_PD										(1<<3)		//!< Power mode
#define L3GD20H_CTRL1_BW_MASK									(3<<4)		//!< Bandwidth selection
#define L3GD20H_CTRL1_DR_MASK									(3<<6)		//!< Output data rate selection

#define L3GD20H_CTRL2_REG					0x21

#define L3GD20H_CTRL2_HPCF_MASK									(0xF<<0)	//!< High pass filter cut off freq.
#define L3GD20H_CTRL2_HPM_MASK									(3<<4)		//!< High pass filter mode
#define L3GD20H_CTRL2_LVLEN										(1<<6)		//!< Level sensitive trigger enable
#define L3GD20H_CTRL2_EXTREN									(1<<7)		//!< Edge sensitive trigger enable

#define L3GD20H_CTRL3_REG					0x22

#define L3GD20H_CTRL3_INT2_EMPTY								(1<<0)		//!< Fifo empty interrupt on INT2 pin
#define L3GD20H_CTRL3_INT2_ORUN									(1<<1)		//!< Fifo overrun interrupt on INT2 pin
#define L3GD20H_CTRL3_INT2_FTH									(1<<2)		//!< Fifo threshold interrupt on INT2 pin
#define L3GD20H_CTRL3_INT2_DRDY									(1<<3)		//!< Data ready interrupt on INT2 pin
#define L3GD20H_CTRL3_PP_OD										(1<<4)		//!< 0 : Push-pull / 1 : Open drain
#define L3GD20H_CTRL3_H_LACTIVE									(1<<5)		//!< Interrupt active on INT. (0:high, 1:low)
#define L3GD20H_CTRL3_INT1_BOOT									(1<<6)		//!< Boot status available on INT1 pin
#define L3GD20H_CTRL3_INT1_IG									(1<<7)		//!< Interrupt enable on INT1 pin

#define L3GD20H_CTRL4_REG					0x23
#define L3GD20H_CTRL4_SIM										(1<<0)		//!< SPI mode (0: wire, 1: 3 wire)
#define L3GD20H_CTRL4_ST_MASK									(3<<1)		//!< Self test
#define L3GD20H_CTRL4_IMPEN										(1<<3)		//!< Level sensitive latched enable
#define L3GD20H_CTRL4_FS_MASK									(3<<4)		//!< Full scale selection
#define L3GD20H_CTRL4_BLE										(1<<6)		//!< Big endian selection
#define L3GD20H_CTRL4_BDU_SINGLE								(1<<7)		//!< Block data update

#define L3GD20H_CTRL5_REG					0x24
#define L3GD20H_CTRL5_OUT_SEL_MASK								(3<<0)		//!< Out selection
#define L3GD20H_CTRL5_IG_SEL_MASK								(3<<2)		//!< INT generator selection
#define L3GD20H_CTRL5_HPEN										(1<<4)		//!< High pass filter enable
#define L3GD20H_CTRL5_STOPONFTH_LIMITED							(1<<5)		//!< Sensing chain FIfo stop value
#define L3GD20H_CTRL5_FIFO_EN									(1<<6)		//!< Fifo enable
#define L3GD20H_CTRL5_BOOT										(1<<7)		//!< Reboot memory content

#define L3GD20H_REFERENCE_REG				0x25

#define L3GD20H_OUT_TEMP_REG				0x26

#define L3GD20H_STATUS_REG					0x27

#define L3GD20H_OUT_X_L_REG					0x28
#define L3GD20H_OUT_X_H_REG					0x29

#define L3GD20H_OUT_Y_L_REG					0x2A
#define L3GD20H_OUT_Y_H_REG					0x2B

#define L3GD20H_OUT_Z_L_REG					0x2C
#define L3GD20H_OUT_Z_H_REG					0x2D

#define L3GD20H_FIFO_CTRL_REG				0x2E

#define L3GD20H_FIFO_CTRL_FTH_MASK								(0x1F<<0)	//!< Fifo threshold
#define L3GD20H_FIFO_CTRL_FM_MASK								(7<<5)		//!< Fifo mode

#define L3GD20H_FIFO_SRC_REG				0x2F

#define L3GD20H_FIFO_SRC_FSS_MASK								(0x1F<<0)	//!< Fifo stored data level
#define L3GD20H_FIFO_SRC_EMPTY									(1<<5)		//!< Fifo empty
#define L3GD20H_FIFO_SRC_OVRN									(1<<6)		//!< Fifo overrun
#define L3GD20H_FIFO_SRC_FTH									(1<<7)		//!< Fifo threshold status

#define L3GD20H_IG_CFG_REG					0x30

#define L3GD20H_IG_CFG_XLIE										(1<<0)		//!< Enable interrupt X low
#define L3GD20H_IG_CFG_XHIE										(1<<1)		//!< Enable interrupt X high
#define L3GD20H_IG_CFG_YLIE										(1<<2)		//!< Enable interrupt Y low
#define L3GD20H_IG_CFG_YHIE										(1<<3)		//!< Enable interrupt Y high
#define L3GD20H_IG_CFG_ZLIE										(1<<4)		//!< Enable interrupt Z low
#define L3GD20H_IG_CFG_ZHIE										(1<<5)		//!< Enable interrupt Z high
#define L3GD20H_IG_CFG_LIR										(1<<6)		//!< Latch interrupt
#define L3GD20H_IG_CFG_ANDOR									(1<<7)		//!< Combination interrupt 0: OR, 1: AND

#define L3GD20H_IG_SRC_REG					0x31

#define L3GD20H_IG_SRC_XL										(1<<0)		//!< X low event
#define L3GD20H_IG_SRC_XH										(1<<1)		//!< X high event
#define L3GD20H_IG_SRC_YL										(1<<2)		//!< Y low event
#define L3GD20H_IG_SRC_YH										(1<<3)		//!< Y high event
#define L3GD20H_IG_SRC_ZL										(1<<4)		//!< Z low event
#define L3GD20H_IG_SRC_ZH										(1<<5)		//!< Z high event
#define L3GD20H_IG_SRC_IA										(1<<6)		//!< Interrupt event

#define L3GD20H_IG_THS_XH_REG				0x32

#define L3GD20H_IG_THS_XH_THSX_MASK								(0x7F<<0)	//!< Interrupt threshold
#define L3GD20H_IG_THS_XH_DCRM									(1<<7)		//!< Interrupt generation counter decrement

#define L3GD20H_IG_THS_XL_REG				0x33

#define L3GD20H_IG_THS_YH_REG				0x34

#define L3GD20H_IG_THS_YH_THSY_MASK								(0x7F<<0)

#define L3GD20H_IG_THS_YL_REG				0x35

#define L3GD20H_IG_THS_ZH_REG				0x36

#define L3GD20H_IG_THS_ZH_THSZ_MASK								(0x7F<<0)

#define L3GD20H_IG_THS_ZL_REG				0x37

#define L3GD20H_IG_DURATION_REG				0x38

#define L3GD20H_IG_DURATION_D_MASK								(0x7F<<0)	//!< Duration value
#define L3GD20H_IG_DURATION_WAIT								(1<<7)		//!< Wait enable

#define L3GD20H_LOW_ODR_REG					0x39

#define L3GD20H_LOW_ODR_LOW_ODR									(1<<0)		//!< Low speed ODR
#define L3GD20H_LOW_ODR_SW_RES									(1<<2)		//!< Soft reset
#define L3GD20H_LOW_ODR_I2C_DIS									(1<<3)		//!< Disable I2C
#define L3GD20H_LOW_ODR_DRDY_ACTIVE_LOW							(1<<5)		//!< DRDY/INT2 pin active level

/// L3GD20H Gyro device class
class GyroL3gd20h : public GyroSensor, public TempSensor  {
public:
	/**
	 * @brief	Initialize gyro sensor.
	 *
	 * NOTE: This sensor must be the first to be initialized.
	 *
	 * @param 	Cfg		: Gyro configuration data
	 * @param 	pIntrf	: Pointer to communication interface
	 * @param 	pTimer	: Pointer to Timer use for time stamp
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const GYROSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual bool Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	virtual bool Read(GYROSENSOR_RAWDATA &Data) { return GyroSensor::Read(Data); }
	virtual bool Read(GYROSENSOR_DATA &Data) { return GyroSensor::Read(Data); }

	bool UpdateData() { return true; }

	virtual bool StartSampling() { return true; }

private:

	bool vbInitialized;
};



#endif // __GYRO_L3GD20H_H__
