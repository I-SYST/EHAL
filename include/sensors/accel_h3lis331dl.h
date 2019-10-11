/**-------------------------------------------------------------------------
@file	accel_h3lis331dl.h

@brief	Implementation of ST H3LIS331DL accel. sensor


@author	Hoang Nguyen Hoan
@date	Oct. 2, 2019

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

#ifndef __ACCEL_H3LIS331DL_H__
#define __ACCEL_H3LIS331DL_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"

#define H3LIS331DL_I2C_DEVADDR				0x18	//!< 7 bits i2c address for SA0 = 0
#define H3LIS331DL_I2C_DEVADDR1				0x19	//!< 7 bits i2c address for SA0 = 1

// ***** XL
#define H3LIS331DL_WHO_AM_I_REG				0x0F
#define H3LIS331DL_WHO_AM_I_ID								0x32

#define H3LIS331DL_CTRL_REG1_REG			0x20

#define H3LIS331DL_CTRL_REG1_XEN								(1<<0)	//!< X axis enable
#define H3LIS331DL_CTRL_REG1_YEN								(1<<1)	//!< Y axis enable
#define H3LIS331DL_CTRL_REG1_ZEN								(1<<2)	//!< Z axis enable
#define H3LIS331DL_CTRL_REG1_DR_MASK							(3<<3)	//!< Data rate mask
#define H3LIS331DL_CTRL_REG1_DR_ODR50HZ							(0<<3)	//!< ODR 50 Hz
#define H3LIS331DL_CTRL_REG1_DR_ODR100HZ						(1<<3)	//!< ODR 100 Hz
#define H3LIS331DL_CTRL_REG1_DR_ODR400HZ						(2<<3)	//!< ODR 400 Hz
#define H3LIS331DL_CTRL_REG1_DR_ODR1KHZ							(3<<3)	//!< ODR 1 KHz
#define H3LIS331DL_CTRL_REG1_PM_MASK							(7<<5)	//!< Power mode
#define H3LIS331DL_CTRL_REG1_PM_OFF								(0<<5)	//!< Power off
#define H3LIS331DL_CTRL_REG1_PM_NORMAL							(1<<5)	//!< Normal mode -> ODR settings
#define H3LIS331DL_CTRL_REG1_PM_LP05HZ							(2<<5)	//!< Low power 0.5 Hz
#define H3LIS331DL_CTRL_REG1_PM_LP1HZ							(3<<5)	//!< Low power 1 Hz
#define H3LIS331DL_CTRL_REG1_PM_LP2HZ							(4<<5)	//!< Low power 2 Hz
#define H3LIS331DL_CTRL_REG1_PM_LP5HZ							(5<<5)	//!< Low power 5 Hz
#define H3LIS331DL_CTRL_REG1_PM_LP10HZ							(6<<5)	//!< Low power 10 Hz

#define H3LIS331DL_CTRL_REG2_REG			0x21

#define H3LIS331DL_CTRL_REG2_HPCF_MASK							(3<<0)	//!< High pass filter enable
#define H3LIS331DL_CTRL_REG2_HPEN1								(1<<2)	//!< High pass filter enable for interrupt1
#define H3LIS331DL_CTRL_REG2_HPEN2								(1<<3)	//!< High pass filter enable for interrupt2
#define H3LIS331DL_CTRL_REG2_FDS								(1<<4)	//!< High pass filter data selection
#define H3LIS331DL_CTRL_REG2_HPM_MASK							(3<<5)	//!< High pass filter mode selection
#define H3LIS331DL_CTRL_REG2_BOOT								(1<<7)	//!< Reboot memory content

#define H3LIS331DL_CTRL_REG3_A_REG			0x22

#define H3LIS331DL_CTRL_REG3_I1_CFG_MASK						(3<<0)	//!< Data signal on INT1
#define H3LIS331DL_CTRL_REG3_LIR1								(1<<2)	//!< Latch interrupt on INT1
#define H3LIS331DL_CTRL_REG3_I2_CFG_MASK						(3<<3)	//!< Data signal on INT2
#define H3LIS331DL_CTRL_REG3_LIR2								(1<<5)	//!< Latch interrupt on INT2
#define H3LIS331DL_CTRL_REG3_PP_OD								(1<<6)	//!< Open drain
#define H3LIS331DL_CTRL_REG3_IHL_LOW							(1<<7)	//!< Interrupt active low

#define H3LIS331DL_CTRL_REG4_REG			0x23

#define H3LIS331DL_CTRL_REG4_SIM_3WIRE							(1<<0)	//!< SPI 3 wire mode
#define H3LIS331DL_CTRL_REG4_FS_MASK							(3<<4)	//!< Full scale select mask
#define H3LIS331DL_CTRL_REG4_FS_100G							(0<<4)	//!< 100G
#define H3LIS331DL_CTRL_REG4_FS_200G							(1<<4)	//!< 200G
#define H3LIS331DL_CTRL_REG4_FS_400G							(3<<4)	//!< 400G
#define H3LIS331DL_CTRL_REG4_BLE_BIG							(1<<6)	//!< Big endian MSB first
#define H3LIS331DL_CTRL_REG4_BDU_SINGLE							(1<<7)	//!< Block data update single

#define H3LIS331DL_CTRL_REG5_REG			0x24

#define H3LIS331DL_CTRL_REG5_TURNON_MASK						(3<<0)	//!< Turn on mode mask (sleep-to-wake)
#define H3LIS331DL_CTRL_REG5_TURNON_DISABLE						(0<<0)	//!< Turn on (sleep-to-wake) disable
#define H3LIS331DL_CTRL_REG5_TURNON_LP							(3<<0)	//!< Wake in low power mode

#define H3LIS331DL_HP_FILTER_RESET_REG		0x25	//!< Reading this results in reset the content of high pass filter

#define H3LIS331DL_REFERENCE_REG			0x26	//!< Reference value for high pass filter

#define H3LIS331DL_STATUS_REG_REG			0x27

#define H3LIS331DL_STATUS_REG_XDA								(1<<0)	//!< X axis data avail
#define H3LIS331DL_STATUS_REG_YDA								(1<<1)	//!< Y axis data avail
#define H3LIS331DL_STATUS_REG_ZDA								(1<<2)	//!< Z axis data avail
#define H3LIS331DL_STATUS_REG_ZYXDA								(1<<3)	//!< X, Y, Z axis data avail
#define H3LIS331DL_STATUS_REG_XOR								(1<<4)	//!< X axis data overrun
#define H3LIS331DL_STATUS_REG_YOR								(1<<5)	//!< Y axis data overrun
#define H3LIS331DL_STATUS_REG_ZOR								(1<<6)	//!< Z axis data overrun
#define H3LIS331DL_STATUS_REG_ZYXOR								(1<<7)	//!< X, Y, Z data overrun

#define H3LIS331DL_OUT_X_L_REG				0x28
#define H3LIS331DL_OUT_X_H_REG				0x29

#define H3LIS331DL_OUT_Y_L_REG				0x2A
#define H3LIS331DL_OUT_Y_H_REG				0x2B

#define H3LIS331DL_OUT_Z_L_REG				0x2C
#define H3LIS331DL_OUT_Z_H_REG				0x2D

#define H3LIS331DL_INT1_CFG_REG				0x30

#define H3LIS331DL_INT1_CFG_XLIE								(1<<0)		//!< X low interrupt enable
#define H3LIS331DL_INT1_CFG_XHIE								(1<<1)		//!< X high interrupt enable
#define H3LIS331DL_INT1_CFG_YLIE								(1<<2)		//!< Y low interrupt enable
#define H3LIS331DL_INT1_CFG_YHIE								(1<<3)		//!< Y high interrupt enable
#define H3LIS331DL_INT1_CFG_ZLIE								(1<<4)		//!< Z low interrupt enable
#define H3LIS331DL_INT1_CFG_ZHIE								(1<<5)		//!< Z high interrupt enable
#define H3LIS331DL_INT1_CFG_AOI									(1<<7)		//!< And/Or

#define H3LIS331DL_INT1_SRC_REG				0x31

#define H3LIS331DL_INT1_SRC_XL									(1<<0)
#define H3LIS331DL_INT1_SRC_XH									(1<<1)
#define H3LIS331DL_INT1_SRC_YL									(1<<2)
#define H3LIS331DL_INT1_SRC_YH									(1<<3)
#define H3LIS331DL_INT1_SRC_ZL									(1<<4)
#define H3LIS331DL_INT1_SRC_ZH									(1<<5)
#define H3LIS331DL_INT1_SRC_IA									(1<<6)		//!< Interrupt active

#define H3LIS331DL_INT1_THS_REG				0x32
#define H3LIS331DL_INT1_THS_MASK								(0x7F<<0)

#define H3LIS331DL_INT1_DURATION_REG		0x33
#define H3LIS331DL_INT1_DURATION_MASK							(0x7F<<0)

#define H3LIS331DL_INT2_CFG_REG				0x34

#define H3LIS331DL_INT2_CFG_XLIE								(1<<0)
#define H3LIS331DL_INT2_CFG_XHIE								(1<<1)
#define H3LIS331DL_INT2_CFG_YLIE								(1<<2)
#define H3LIS331DL_INT2_CFG_YHIE								(1<<3)
#define H3LIS331DL_INT2_CFG_ZLIE								(1<<4)
#define H3LIS331DL_INT2_CFG_ZHIE								(1<<5)
#define H3LIS331DL_INT2_CFG_AOI									(1<<7)		//!< AND/OR combination of interrupt events

#define H3LIS331DL_INT2_SRC_REG				0x35

#define H3LIS331DL_INT2_SRC_XL									(1<<0)
#define H3LIS331DL_INT2_SRC_XH									(1<<1)
#define H3LIS331DL_INT2_SRC_YL									(1<<2)
#define H3LIS331DL_INT2_SRC_YH									(1<<3)
#define H3LIS331DL_INT2_SRC_ZL									(1<<4)
#define H3LIS331DL_INT2_SRC_ZH									(1<<5)
#define H3LIS331DL_INT2_SRC_IA									(1<<6)		//!< Interrupt active

#define H3LIS331DL_INT2_THS_REG				0x36
#define H3LIS331DL_INT2_THS_MASK								(0x7F<<0)

#define H3LIS331DL_INT2_DURATION_REG		0x37
#define H3LIS331DL_INT2_DURATION_MASK							(0x7F<<0)


class AccelH3lis331dl : public AccelSensor {
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
	bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);

	uint16_t Scale(uint16_t Value);
	/**
	 * @brief	Set sampling frequency.
	 *
	 * The sampling frequency is relevant only in continuous mode.
	 *
	 * @return	Frequency in mHz (milliHerz)
	 */
	uint32_t SamplingFrequency(uint32_t Freq);

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	uint32_t FilterFreq(uint32_t Freq);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	void IntHandler();

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
	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);

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
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

private:
};

#endif // __ACCEL_H3LIS331DL_H__
