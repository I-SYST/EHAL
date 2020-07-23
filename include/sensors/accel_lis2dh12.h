/**-------------------------------------------------------------------------
@file	accel_lis22dh12.h

@brief	Implementation of ST LIS2DH12 accel. sensor


@author	Hoang Nguyen Hoan
@date	Jan. 17, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#ifndef __ACCEL_LIS2DH12_H__
#define __ACCEL_LIS2DH12_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/temp_sensor.h"

#define LIS2DH12_I2C_DEVADDR				0x18	//!< 7 bits i2c address for SA0 = 0
#define LIS2DH12_I2C_DEVADDR1				0x19	//!< 7 bits i2c address for SA0 = 1

#define LIS2DH12_WHO_AM_I_REG				0x0F
#define LIS2DH12_WHO_AM_I_ID								0x33

#define LIS2DH12_STATUS_REG_AUX				0x07
#define LIS2DH12_STATUS_REG_AUX_TDA							(1<<2)	//!< Temperature new data avail
#define LIS2DH12_STATUS_REG_AUX_TOR							(1<<6)	//!< Temperature data overrun

#define LIS2DH12_OUT_TEMP_L					0x0C
#define LIS2DH12_OUT_TEMP_H					0x0D

#define LIS2DH12_CTRL_REG0					0x1E
#define LIS2DH12_CTRL_REG0_SDO_PU_DISC						(1<<7)	//!< Disconnect SDO/SA0 pullup

#define LIS2DH12_TEMP_CFG_REG				0x1F
#define LIS2DH12_TEMP_CFG_EN								(3<<6)	//!< Temperature sensor enable

#define LIS2DH12_CTRL_REG1					0x20
#define LIS2DH12_CTRL_REG1_XEN								(1<<0)	//!< X axis enable
#define LIS2DH12_CTRL_REG1_YEN								(1<<1)	//!< Y axis enable
#define LIS2DH12_CTRL_REG1_ZEN								(1<<2)	//!< Z axis enable
#define LIS2DH12_CTRL_REG1_LPEN								(1<<3)	//!< Low power enable
#define LIS2DH12_CTRL_REG1_ODR_MASK							(0xF<<4)
#define LIS2DH12_CTRL_REG1_ODR_PWRDWN						(0<<4)	//!< Power down
#define LIS2DH12_CTRL_REG1_ODR_1HZ							(1<<4)	//!< 1 Hz
#define LIS2DH12_CTRL_REG1_ODR_10HZ							(2<<4)	//!< 10 Hz
#define LIS2DH12_CTRL_REG1_ODR_25HZ							(3<<4)	//!< 25 Hz
#define LIS2DH12_CTRL_REG1_ODR_50HZ							(4<<4)	//!< 50 Hz
#define LIS2DH12_CTRL_REG1_ODR_100HZ						(5<<4)	//!< 100 Hz
#define LIS2DH12_CTRL_REG1_ODR_200HZ						(6<<4)	//!< 200 Hz
#define LIS2DH12_CTRL_REG1_ODR_400HZ						(7<<4)	//!< 400 Hz
#define LIS2DH12_CTRL_REG1_ODR_1620HZ						(8<<4)	//!< Low power mode 1.62 KHz
#define LIS2DH12_CTRL_REG1_ODR_HR_LP						(9<<4)	//!< HR/ Normal 1.344 KHz, Low power 5.376 KHz

#define LIS2DH12_CTRL_REG2					0x21
#define LIS2DH12_CTRL_REG2_HP_IA1							(1<<0)	//!< High pass filter enable for AOI on int1
#define LIS2DH12_CTRL_REG2_HP_IA2							(1<<1)	//!< High pass filter enable for AOI on int2
#define LIS2DH12_CTRL_REG2_HP_CLICK							(1<<2)	//!< High pass filter enable on click function
#define LIS2DH12_CTRL_REG2_FDS								(1<<3)	//!< Filtered data
#define LIS2DH12_CTRL_REG2_HPCF_MASK						(3<<4)	//!< High pass cut off freq
#define LIS2DH12_CTRL_REG2_HPM_MASK							(3<<6)	//!< High pass filter mode
#define LIS2DH12_CTRL_REG2_HPM_NORMALRST					(0<<6)	//!< Normal mode (reset by reading register)
#define LIS2DH12_CTRL_REG2_HPM_REFSIG						(1<<6)
#define LIS2DH12_CTRL_REG2_HPM_NORMAL						(2<<6)
#define LIS2DH12_CTRL_REG2_HPM_AUTORST						(3<<6)	//!< Auto reset on interrupt event

#define LIS2DH12_CTRL_REG3					0x22
#define LIS2DH12_CTRL_REG3_I1_OVERRUN						(1<<1)	//!< Enable Fifo overrun on int1
#define LIS2DH12_CTRL_REG3_I1_WTM							(1<<2)	//!< Enable Fifo watermark on int1
#define LIS2DH12_CTRL_REG3_I1_ZYXDA							(1<<4)	//!< Enable ZYXDA on int1
#define LIS2DH12_CTRL_REG3_I1_IA2							(1<<5)	//!< Enable IA2 on int1
#define LIS2DH12_CTRL_REG3_I1_IA1							(1<<6)	//!< Enable IA1 on int1
#define LIS2DH12_CTRL_REG3_I1_CLICK							(1<<7)	//!< Enable CLICK on int1

#define LIS2DH12_CTRL_REG4					0x23
#define LIS2DH12_CTRL_REG4_SIM_3WIRE						(1<<0)	//!< SPI serial 3 wire mode
#define LIS2DH12_CTRL_REG4_ST_MASK							(3<<1)	//!< Self test select mask
#define LIS2DH12_CTRL_REG4_ST_NORMAL						(0<<1)	//!< Normal operation
#define LIS2DH12_CTRL_REG4_ST_SELFTEST0						(1<<1)
#define LIS2DH12_CTRL_REG4_ST_SELFTEST1						(2<<1)
#define LIS2DH12_CTRL_REG4_HR								(1<<3)	//!< HR mode
#define LIS2DH12_CTRL_REG4_FS_MASK							(3<<4)	//!< Full scale selection mask
#define LIS2DH12_CTRL_REG4_FS_2G							(0<<4)
#define LIS2DH12_CTRL_REG4_FS_4G							(1<<4)
#define LIS2DH12_CTRL_REG4_FS_8G							(2<<4)
#define LIS2DH12_CTRL_REG4_FS_16G							(3<<4)
#define LIS2DH12_CTRL_REG4_BLE								(1<<6)	//!< Big endian
#define LIS2DH12_CTRL_REG4_BDU								(1<<7)	//!< Blocking data update

#define LIS2DH12_CTRL_REG5					0x24
#define LIS2DH12_CTRL_REG5_D4D_INT2							(1<<0)	//!< Enable 4D detection on int2
#define LIS2DH12_CTRL_REG5_LIR_INT2							(1<<1)	//!< Interrupt latched mode on int2
#define LIS2DH12_CTRL_REG5_D4D_INT1							(1<<2)	//!< Enable 4D detection on int1
#define LIS2DH12_CTRL_REG5_LIR_INT1							(1<<3)	//!< Interrupt latched mode on int1
#define LIS2DH12_CTRL_REG5_FIFO_EN							(1<<6)	//!< Enable Fifo
#define LIS2DH12_CTRL_REG5_BOOT								(1<<7)	//!< Reboot memory content

#define LIS2DH12_CTRL_REG6					0x25
#define LIS2DH12_CTRL_REG6_INT_POLARITY_LOW					(1<<1)	//!< Interrupt active low
#define LIS2DH12_CTRL_REG6_I2_ACT							(1<<3)	//!< Enable activity on int2
#define LIS2DH12_CTRL_REG6_I2_BOOT							(1<<4)	//!< Enable boot on int2
#define LIS2DH12_CTRL_REG6_I2_IA2							(1<<5)	//!< Enable interrupt2 functions on int2
#define LIS2DH12_CTRL_REG6_I2_IA1							(1<<6)	//!< Enable interrupt1 function on int2
#define LIS2DH12_CTRL_REG6_I2_CLICK							(1<<7)	//!< Enable CLICK on int2

#define LIS2DH12_REFERENCE					0x26	//!< Reference value for interrupt generation

#define LIS2DH12_STATUS_REG					0x27
#define LIS2DH12_STATUS_REG_XDA								(1<<0)	//!< New X-axis data avail
#define LIS2DH12_STATUS_REG_YDA								(1<<1)	//!< New Y-axis data avail
#define LIS2DH12_STATUS_REG_ZDA								(1<<2)	//!< New Z-axis data avail
#define LIS2DH12_STATUS_REG_XZDA							(1<<3)	//!< New XYZ-axis data avail
#define LIS2DH12_STATUS_REG_XOR								(1<<4)	//!< New X-axis data overrun
#define LIS2DH12_STATUS_REG_YOR								(1<<5)	//!< New Y-axis data overrun
#define LIS2DH12_STATUS_REG_ZOR								(1<<6)	//!< New Z-axis data overrun
#define LIS2DH12_STATUS_REG_XYZOR							(1<<7)	//!< New XYZ-axis data overrun

#define LIS2DH12_OUT_X_L					0x28
#define LIS2DH12_OUT_X_H					0x29
#define LIS2DH12_OUT_Y_L					0x2A
#define LIS2DH12_OUT_Y_H					0x2B
#define LIS2DH12_OUT_Z_L					0x2C
#define LIS2DH12_OUT_Z_H					0x2D

#define LIS2DH12_FIFO_CTRL_REG				0x2E
#define LIS2DH12_FIFO_CTRL_REG_FTH_MASK						(0x1F<<0)	//!<
#define LIS2DH12_FIFO_CTRL_REG_TR_MASK						(1<<5)
#define LIS2DH12_FIFO_CTRL_REG_TR_INT1						(0<<5)	//!< Trigger event on int1
#define LIS2DH12_FIFO_CTRL_REG_TR_INT2						(1<<5)	//!< Trigger event on int2
#define LIS2DH12_FIFO_CTRL_REG_FM_MASK						(3<<6)	//!< Fifo mode selection mask
#define LIS2DH12_FIFO_CTRL_REG_FM_BYPASS					(0<<6)
#define LIS2DH12_FIFO_CTRL_REG_FM_FIFO						(1<<6)	//!< Fifo mode
#define LIS2DH12_FIFO_CTRL_REG_FM_STREAM					(2<<6)	//!< Stream mode
#define LIS2DH12_FIFO_CTRL_REG_FM_STREAM_FIFO				(3<<6)	//!< Stream to fifo

#define LIS2DH12_FIFO_SRC_REG				0x2F
#define LIS2DH12_FIFO_SRC_REG_FSS_MASK						(0x1F<<0)	//!< Nb of samples in fifo
#define LIS2DH12_FIFO_SRC_REG_EMPTY							(1<<5)	//!< Fifo empty
#define LIS2DH12_FIFO_SRC_REG_OVRN_FIFO						(1<<6)	//!< Fifo overrun (full)
#define LIS2DH12_FIFO_SRC_REG_WTM							(1<<7)	//!< Fifo threshold reached

#define LIS2DH12_INT1_CFG					0x30
#define LIS2DH12_INT1_CFG_XLIE								(1<<0)	//!< Enable interrupt on XL
#define LIS2DH12_INT1_CFG_XHIE								(1<<1)	//!< Enable interrupt on XH
#define LIS2DH12_INT1_CFG_YLIE								(1<<2)	//!< Enable interrupt on YL
#define LIS2DH12_INT1_CFG_YHIE								(1<<3)	//!< Enable interrupt on YH
#define LIS2DH12_INT1_CFG_ZLIE								(1<<4)	//!< Enable interrupt on ZL
#define LIS2DH12_INT1_CFG_ZHIE								(1<<5)	//!< Enable interrupt on ZH
#define LIS2DH12_INT1_CFG_6D								(1<<6)	//!< Enable 6 direction detection
#define LIS2DH12_INT1_CFG_AOI_AND							(1<<7)	//!< AND interrupt event

#define LIS2DH12_INT1_SRC					0x31
#define LIS2DH12_INT1_SRC_XL								(1<<0)	//!< XL interrupt event
#define LIS2DH12_INT1_SRC_XH								(1<<1)	//!< XH interrupt event
#define LIS2DH12_INT1_SRC_YL								(1<<2)	//!< YL interrupt event
#define LIS2DH12_INT1_SRC_YH								(1<<3)	//!< YH interrupt event
#define LIS2DH12_INT1_SRC_ZL								(1<<4)	//!< ZL interrupt event
#define LIS2DH12_INT1_SRC_ZH								(1<<5)	//!< ZH interrupt event
#define LIS2DH12_INT1_SRC_IA								(1<<6)	//!< Interrupt event

#define LIS2DH12_INT1_THS					0x32
#define LIS2DH12_INT1_THS_MASK								(0x7F)

#define LIS2DH12_INT1_DURATION				0x33

#define LIS2DH12_INT2_CFG					0x34
#define LIS2DH12_INT2_CFG_XLIE								(1<<0)	//!< Enable interrupt on XL
#define LIS2DH12_INT2_CFG_XHIE								(1<<1)	//!< Enable interrupt on XH
#define LIS2DH12_INT2_CFG_YLIE								(1<<2)	//!< Enable interrupt on YL
#define LIS2DH12_INT2_CFG_YHIE								(1<<3)	//!< Enable interrupt on YH
#define LIS2DH12_INT2_CFG_ZLIE								(1<<4)	//!< Enable interrupt on ZL
#define LIS2DH12_INT2_CFG_ZHIE								(1<<5)	//!< Enable interrupt on ZH
#define LIS2DH12_INT2_CFG_6D								(1<<6)	//!< Enable 6 direction detection
#define LIS2DH12_INT2_CFG_AOI_AND							(1<<7)	//!< AND interrupt event

#define LIS2DH12_INT2_SRC					0x35
#define LIS2DH12_INT2_SRC_XL								(1<<0)	//!< XL interrupt event
#define LIS2DH12_INT2_SRC_XH								(1<<1)	//!< XH interrupt event
#define LIS2DH12_INT2_SRC_YL								(1<<2)	//!< YL interrupt event
#define LIS2DH12_INT2_SRC_YH								(1<<3)	//!< YH interrupt event
#define LIS2DH12_INT2_SRC_ZL								(1<<4)	//!< ZL interrupt event
#define LIS2DH12_INT2_SRC_ZH								(1<<5)	//!< ZH interrupt event
#define LIS2DH12_INT2_SRC_IA								(1<<6)	//!< Interrupt event

#define LIS2DH12_INT2_THS					0x36
#define LIS2DH12_INT2_THS_MASK								(0x7F)

#define LIS2DH12_INT2_DURATION				0x37

#define LIS2DH12_CLICK_CFG					0x38
#define LIS2DH12_CLICK_CFG_XS									(1<<0)	//!< Enable single click interrupt on X axis
#define LIS2DH12_CLICK_CFG_XD									(1<<1)	//!< Enable double click interrupt on X axis
#define LIS2DH12_CLICK_CFG_YS									(1<<2)	//!< Enable single click interrupt on Y axis
#define LIS2DH12_CLICK_CFG_YD									(1<<3)	//!< Enable double click interrupt on Y axis
#define LIS2DH12_CLICK_CFG_ZS									(1<<4)	//!< Enable single click interrupt on Z axis
#define LIS2DH12_CLICK_CFG_ZD									(1<<5)	//!< Enable double click interrupt on Z axis

#define LIS2DH12_CLICK_SRC					0x39
#define LIS2DH12_CLICK_SRC_X									(1<<0)	//!< Click dtected on X axis
#define LIS2DH12_CLICK_SRC_Y									(1<<1)	//!< Click dtected on Y axis
#define LIS2DH12_CLICK_SRC_Z									(1<<2)	//!< Click dtected on Z axis
#define LIS2DH12_CLICK_SRC_SIGN_NEG								(1<<3)	//!< Click sign negative
#define LIS2DH12_CLICK_SRC_SCLICK								(1<<4)	//!< Enable single click detection
#define LIS2DH12_CLICK_SRC_DCLICK								(1<<5)	//!< Enable double click detection
#define LIS2DH12_CLICK_SRC_IA									(1<<6)	//!< Click interrupt flag

#define LIS2DH12_CLICK_THS					0x3A
#define LIS2DH12_CLICK_THS_THS_MASK								(0x7F)
#define LIS2DH12_CLICK_THS_LIR_CLICK							(1<<7)	//!< Interrupt duration

#define LIS2DH12_TIME_LIMIT					0x3B
#define LIS2DH12_TIME_LIMIT_MASK								(0x7F)	//!< Click time limit

#define LIS2DH12_TIME_LATENCY				0x3C		//!< Click time latency

#define LIS2DH12_TIME_WINDOW				0x3D		//!< Click time window

#define LIS2DH12_ACT_THS					0x3E
#define LIS2DH12_ACT_THS_MASK									(0x7F)	//!< Sleep-to-wake activation threshold

#define LIS2DH12_ACT_DUR					0x3F		//!< Sleep-to-wake duration

#define LIS2DH12_TEMP_MAX_C					127

class AccelLis2dh12 : public AccelSensor, public TempSensor {
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

	/**
	 * @brief	Initialize sensor (require implementation).
	 *
	 * @param 	CfgData : Reference to configuration data
	 * @param	pIntrf 	: Pointer to interface to the sensor.
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 * @param	pTimer	: Pointer to timer for retrieval of time stamp
	 * 					  This pointer will be kept internally
	 * 					  for all access to device.
	 * 					  DONOT delete this object externally
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	bool Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

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

	/**
	 * @brief	Power off the device completely.
	 *
	 * If supported, this will put the device in complete power down.
	 * Full re-intialization is required to re-enable the device.
	 */
	void PowerOff();
	void IntHandler();
	bool UpdateData();

	bool Read(ACCELSENSOR_RAWDATA &Data) { return AccelSensor::Read(Data); }
	bool Read(ACCELSENSOR_DATA &Data) { return AccelSensor::Read(Data); }
	void Read(TEMPSENSOR_DATA &Data) { TempSensor::Read(Data); }
	bool StartSampling() { return true; }

private:

	bool Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
};

#endif // __ACCEL_LIS2DH12_H__
