/**-------------------------------------------------------------------------
@file	am_lsm303c.h

@brief	Implementation of ST LSM303C accel, mag sensor

This device is a combination of 2 independent entities.  Therefore the
implementation consists of 2 independent objects.

@author	Hoang Nguyen Hoan
@date	July 28, 2019

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

#ifndef __AM_LSM303C_H__
#define __AM_LSM303C_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/mag_sensor.h"
#include "sensors/temp_sensor.h"

#define LSM303C_ACCEL_I2C_DEVADDR			0x1D	//!< 7 bit i2c address for Accel
#define LSM303C_MAG_I2C_DEVADDR				0x1E	//!< 7 bit i2c address for Mag

// ***** XL
#define LSM303C_WHO_AM_I_A_REG				0x0F
#define LSM303C_WHO_AM_I_A_ID									0x41

#define LSM303C_ACT_THS_A_REG				0x1E

#define LSM303C_ACT_DUR_A_REG				0x1F

#define LSM303C_CTRL_REG1_A_REG				0x20

#define LSM303C_CTRL_REG1_A_XEN									(1<<0)	//!< X axis enable
#define LSM303C_CTRL_REG1_A_YEN									(1<<1)	//!< Y axis enable
#define LSM303C_CTRL_REG1_A_ZEN									(1<<2)	//!< Z axis enable
#define LSM303C_CTRL_REG1_A_BDU_SINGLE							(1<<3)	//!< Block data update
#define LSM303C_CTRL_REG1_A_ODR_MASK							(7<<4)	//!< Output data rate
#define LSM303C_CTRL_REG1_A_HR									(1<<7)	//!< High res mode

#define LSM303C_CTRL_REG2_A_REG				0x21

#define LSM303C_CTRL_REG2_A_HPIS_MASK							(3<<0)	//!< High pass filter enable
#define LSM303C_CTRL_REG2_A_FDS									(1<<2)	//!< High pass filter data selection
#define LSM303C_CTRL_REG2_A_HPM_MASK							(3<<3)	//!< High pass filter mode selection
#define LSM303C_CTRL_REG2_A_DFC_MASK							(3<<5)	//!< High pass filter cutoff freq

#define LSM303C_CTRL_REG3_A_REG				0x22

#define LSM303C_CTRL_REG3_A_INT_XL_DRDY							(1<<0)	//!< Data ready XL
#define LSM303C_CTRL_REG3_A_INT_XL_FTH							(1<<1)	//!< Fifo threshold
#define LSM303C_CTRL_REG3_A_INT_XL_OVR							(1<<2)	//!< Fifo overrun
#define LSM303C_CTRL_REG3_A_INT_XL_IG1							(1<<3)	//!< Interrupt generator 1 enable
#define LSM303C_CTRL_REG3_A_INT_XL_IG2							(1<<4)	//!< Interrupt generator 2 enable
#define LSM303C_CTRL_REG3_A_INT_XL_INACT						(1<<5)	//!< Inactivity interrupt enable
#define LSM303C_CTRL_REG3_A_STOP_FTH							(1<<6)	//!< Enable Fifo threshold level
#define LSM303C_CTRL_REG3_A_FIFO_EN								(1<<7)	//!< Fifo enable

#define LSM303C_CTRL_REG4_A_REG				0x23

#define LSM303C_CTRL_REG4_A_SIM_SPI_RW							(1<<0)	//!< SPI mode read/write
#define LSM303C_CTRL_REG4_A_I2C_DISABLE							(1<<1)	//!< Disable I2C
#define LSM303C_CTRL_REG4_A_IF_ADD_INC							(1<<2)	//!< Enable register address auto-increment
#define LSM303C_CTRL_REG4_A_BW_SCALE_ODR						(1<<3)	//!< Bandwidth select BW
#define LSM303C_CTRL_REG4_A_FS_MASK								(3<<4)	//!< Full scale selection
#define LSM303C_CTRL_REG4_A_FS_2G								(0<<4)
#define LSM303C_CTRL_REG4_A_FS_4G								(2<<4)
#define LSM303C_CTRL_REG4_A_FS_8G								(3<<4)
#define LSM303C_CTRL_REG4_A_BW_MASK								(3<<6)	//!< Anti aliasing filter bandwidth
#define LSM303C_CTRL_REG4_A_BW_400HZ							(0<<6)
#define LSM303C_CTRL_REG4_A_BW_200HZ							(1<<6)
#define LSM303C_CTRL_REG4_A_BW_100HZ							(2<<6)
#define LSM303C_CTRL_REG4_A_BW_50HZ								(3<<5)

#define LSM303C_CTRL_REG5_A_REG				0x24

#define LSM303C_CTRL_REG5_A_PP_OD								(1<<0)	//!< Open-drain selection
#define LSM303C_CTRL_REG5_A_H_LACTIVE							(1<<1)	//!< Interrupt active low
#define LSM303C_CTRL_REG5_A_ST_MASK								(3<<2)	//!< Self-test enable
#define LSM303C_CTRL_REG5_A_DEC_MASK							(3<<4)	//!< Decimation of acceleration data
#define LSM303C_CTRL_REG5_A_DEC_DISABLE							(0<<4)
#define LSM303C_CTRL_REG5_A_DEC_2SAMPLES						(1<<4)
#define LSM303C_CTRL_REG5_A_DEC_4SAMPLES						(2<<4)
#define LSM303C_CTRL_REG5_A_DEC_8SAMPLES						(3<<4)
#define LSM303C_CTRL_REG5_A_SOFT_RESET							(1<<6)	//!< Soft-reset
#define LSM303C_CTRL_REG5_A_DEBUG								(1<<7)	//!< Debug stepping enable

#define LSM303C_CTRL_REG6_A_REG				0x25

#define LSM303C_CTRL_REG6_A_BOOT								(1<<7)	//!< Force boot

#define LSM303C_CTRL_REG7_A_REG				0x26

#define LSM303C_CTRL_REG7_A_4D_IG_MASK							(3<<0)	//!< Interrupt 4D enable
#define LSM303C_CTRL_REG7_A_LIR_MASK							(3<<2)	//!< Latched interrupt
#define LSM303C_CTRL_REG7_A_DCRM_MASK							(3<<4)	//!< interrupt counter duration

#define LSM303C_STATUS_REG_A_REG			0x27

#define LSM303C_STATUS_REG_A_XDA								(1<<0)	//!< X axis data avail
#define LSM303C_STATUS_REG_A_YDA								(1<<1)	//!< Y axis data avail
#define LSM303C_STATUS_REG_A_ZDA								(1<<2)	//!< Z axis data avail
#define LSM303C_STATUS_REG_A_ZYXDA								(1<<3)	//!< X, Y, Z axis data avail
#define LSM303C_STATUS_REG_A_XOR								(1<<4)	//!< X axis data overrun
#define LSM303C_STATUS_REG_A_YOR								(1<<5)	//!< Y axis data overrun
#define LSM303C_STATUS_REG_A_ZOR								(1<<6)	//!< Z axis data overrun
#define LSM303C_STATUS_REG_A_ZYXOR								(1<<7)	//!< X, Y, Z data overrun

#define LSM303C_OUT_X_L_A_REG				0x28
#define LSM303C_OUT_X_H_A_REG				0x29

#define LSM303C_OUT_Y_L_A_REG				0x2A
#define LSM303C_OUT_Y_H_A_REG				0x2B

#define LSM303C_OUT_Z_L_A_REG				0x2C
#define LSM303C_OUT_Z_H_A_REG				0x2D

#define LSM303C_FIFO_CTRL_REG				0x2E

#define LSM303C_FIFO_CTRL_FTH_MASK								(0x1F<<0)	//!< Fifo threshold
#define LSM303C_FIFO_CTRL_FMODE_MASK							(7<<5)		//!< Fifo mode

#define LSM303C_FIFO_SRC_REG				0x2F

#define LSM303C_FIFO_SRC_FSS_MASK								(0x1F<<0)	//!< Fifo stored data level
#define LSM303C_FIFO_SRC_EMPTY									(1<<5)		//!< Fifo empty
#define LSM303C_FIFO_SRC_OVR									(1<<6)		//!< Fifo full
#define LSM303C_FIFO_SRC_FTH									(1<<7)		//!< Fifo threshold status

#define LSM303C_IG_CFG1_A_REG				0x30

#define LSM303C_IG_CFG1_A_XLIE									(1<<0)		//!< X low interrupt enable
#define LSM303C_IG_CFG1_A_XHIE									(1<<1)		//!< X high interrupt enable
#define LSM303C_IG_CFG1_A_YLIE									(1<<2)		//!< Y low interrupt enable
#define LSM303C_IG_CFG1_A_YHIE									(1<<3)		//!< Y high interrupt enable
#define LSM303C_IG_CFG1_A_ZLIE									(1<<4)		//!< Z low interrupt enable
#define LSM303C_IG_CFG1_A_ZHIE									(1<<5)		//!< Z high interrupt enable
#define LSM303C_IG_CFG1_A_6D									(1<<6)		//!< 6 direction detection enable
#define LSM303C_IG_CFG1_A_AOI									(1<<7)		//!< And/Or

#define LSM303C_IG_SRC1_A_REG				0x31

#define LSM303C_IG_SRC1_A_XL									(1<<0)
#define LSM303C_IG_SRC1_A_XH									(1<<1)
#define LSM303C_IG_SRC1_A_YL									(1<<2)
#define LSM303C_IG_SRC1_A_YH									(1<<3)
#define LSM303C_IG_SRC1_A_ZL									(1<<4)
#define LSM303C_IG_SRC1_A_ZH									(1<<5)
#define LSM303C_IG_SRC1_A_IA									(1<<6)		//!< Interrupt active

#define LSM303C_IG_THS_X1_A_REG				0x32
#define LSM303C_IG_THS_Y1_A_REG				0x33
#define LSM303C_IG_THS_Z1_A_REG				0x34

#define LSM303C_IG_DUR1_A_REG				0x35

#define LSM303C_IG_DUR1_A_DUR_MASK								(0x7F<<0)
#define LSM303C_IG_DUR1_A_WAIT									(1<<7)

#define LSM303C_IG_CFG2_A_REG				0x36

#define LSM303C_IG_CFG2_A_XLIE									(1<<0)
#define LSM303C_IG_CFG2_A_XHIE									(1<<1)
#define LSM303C_IG_CFG2_A_YLIE									(1<<2)
#define LSM303C_IG_CFG2_A_YHIE									(1<<3)
#define LSM303C_IG_CFG2_A_ZLIE									(1<<4)
#define LSM303C_IG_CFG2_A_ZHIE									(1<<5)
#define LSM303C_IG_CFG2_A_6D									(1<<6)		//!< 6 direction detection enable
#define LSM303C_IG_CFG2_A_AOI									(1<<7)		//!< AND/OR combination of interrupt events

#define LSM303C_IG_SRC2_A_REG				0x37

#define LSM303C_IG_SRC2_A_XL									(1<<0)
#define LSM303C_IG_SRC2_A_XH									(1<<1)
#define LSM303C_IG_SRC2_A_YL									(1<<2)
#define LSM303C_IG_SRC2_A_YH									(1<<3)
#define LSM303C_IG_SRC2_A_ZL									(1<<4)
#define LSM303C_IG_SRC2_A_ZH									(1<<5)
#define LSM303C_IG_SRC2_A_IA									(1<<6)		//!< Interrupt active

#define LSM303C_THS2_A_REG					0x38

#define LSM303C_DUR2_A_REG					0x39

#define LSM303C_DUR2_A_DUR_MASK									(0x7F<<0)
#define LSM303C_DUR2_A_WAIT										(1<<7)

#define LSM303C_XL_REFERENCE_REG			0x3A
#define LSM303C_XH_REFERENCE_REG			0x3B
#define LSM303C_YL_REFERENCE_REG			0x3C
#define LSM303C_YH_REFERENCE_REG			0x3D
#define LSM303C_ZL_REFERENCE_REG			0x3E
#define LSM303C_ZH_REFERENCE_REG			0x3F

// ***** MAG
#define LSM303C_WHO_AM_I_M_REG				0xF

#define LSM303C_WHO_AM_I_M_ID									0x3D

#define LSM303C_CTRL_REG1_M_REG				0x20

#define LSM303C_CTRL_REG1_M_ST									(1<<0)
#define LSM303C_CTRL_REG1_M_DO_MASK								(7<<2)
#define LSM303C_CTRL_REG1_M_OM_MASK								(3<<5)
#define LSM303C_CTRL_REG1_M_TEMP_EN								(1<<7)

#define LSM303C_CTRL_REG2_M_REG				0x21

#define LSM303C_CTRL_REG2_M_SOFT_RST							(1<<2)
#define LSM303C_CTRL_REG2_M_REBOOT								(1<<3)
#define LSM303C_CTRL_REG2_M_FS_MASK								(3<<5)

#define LSM303C_CTRL_REG3_M_REG				0x22

#define LSM303C_CTRL_REG3_M_MD_MASK								(3<<0)
#define LSM303C_CTRL_REG3_M_SIM									(1<<2)
#define LSM303C_CTRL_REG3_M_LP									(1<<5)
#define LSM303C_CTRL_REG3_M_I2C_DISABLE							(1<<7)

#define LSM303C_CTRL_REG4_M_REG				0x23

#define LSM303C_CTRL_REG4_M_BLE									(1<<1)
#define LSM303C_CTRL_REG4_M_OMZ_MASK							(3<<2)

#define LSM303C_CTRL_REG5_M_REG				0x24

#define LSM303C_CTRL_REG5_M_BDU									(1<<6)

#define LSM303C_STATUS_REG_M_REG			0x27

#define LSM303C_STATUS_REG_M_XDA								(1<<0)
#define LSM303C_STATUS_REG_M_YDA								(1<<1)
#define LSM303C_STATUS_REG_M_ZDA								(1<<2)
#define LSM303C_STATUS_REG_M_ZYXDA								(1<<3)
#define LSM303C_STATUS_REG_M_XOR								(1<<4)
#define LSM303C_STATUS_REG_M_YOR								(1<<5)
#define LSM303C_STATUS_REG_M_ZOR								(1<<6)
#define LSM303C_STATUS_REG_M_ZYXOR								(1<<7)

#define LSM303C_OUT_X_L_M_REG				0x28
#define LSM303C_OUT_X_H_M_REG				0x29
#define LSM303C_OUT_Y_L_M_REG				0x2A
#define LSM303C_OUT_Y_H_M_REG				0x2B
#define LSM303C_OUT_Z_L_M_REG				0x2C
#define LSM303C_OUT_Z_H_M_REG				0x2D
#define LSM303C_TEMP_L_M_REG				0x2E
#define LSM303C_TEMP_H_M_REG				0x2F

#define LSM303C_INT_CFG_M_REG				0x30

#define LSM303C_INT_CFG_M_IEN									(1<<0)
#define LSM303C_INT_CFG_M_IEL									(1<<1)
#define LSM303C_INT_CFG_M_IEA									(1<<2)
#define LSM303C_INT_CFG_M_1_MUST								(1<<3)
#define LSM303C_INT_CFG_M_ZIEN									(1<<5)
#define LSM303C_INT_CFG_M_YIEN									(1<<6)
#define LSM303C_INT_CFG_M_XIEN									(1<<7)

#define LSM303C_INT_SRC_M_REG				0x31

#define LSM303C_INT_SRC_M_INT									(1<<0)
#define LSM303C_INT_SRC_M_MROI									(1<<1)
#define LSM303C_INT_SRC_M_NTH_Z									(1<<2)
#define LSM303C_INT_SRC_M_NTH_Y									(1<<3)
#define LSM303C_INT_SRC_M_NTH_X									(1<<4)
#define LSM303C_INT_SRC_M_PTH_Z									(1<<5)
#define LSM303C_INT_SRC_M_PTH_Y									(1<<6)
#define LSM303C_INT_SRC_M_PTH_X									(1<<7)

#define LSM303C_THS_L_M_REG					0x32
#define LSM303C_THS_H_M_REG					0x33

#define LSM303C_THS_H_M_THS_MASK								(0x7F)


// Accel, Mag, Temperature
class AccLsm303c : public AccelSensor, public TempSensor  {
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
	virtual uint16_t Scale(uint16_t Value) { return 0; }			// Accel

	virtual bool Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	virtual bool Read(ACCELSENSOR_RAWDATA &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(ACCELSENSOR_DATA &Data) { return AccelSensor::Read(Data); }

	bool UpdateData() { return false; }

	virtual bool StartSampling() { return false; }

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

private:
	// Default base initialization. Does detection and set default config for all sensor.
	// All sensor init must call this first prio to initializing itself
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	bool vbInitialized;
};

class MagLsm303c : public MagSensor  {
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

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	virtual bool Read(MAGSENSOR_RAWDATA &Data) { return MagSensor::Read(Data); }
	virtual bool Read(MAGSENSOR_DATA &Data) { return MagSensor::Read(Data); }

	bool UpdateData() { return false; }

	virtual bool StartSampling() { return false; }

private:
	// Default base initialization. Does detection and set default config for all sensor.
	// All sensor init must call this first prio to initializing itself
	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer);

	bool vbInitialized;
};



#endif // __AM_LSM303C_H__
