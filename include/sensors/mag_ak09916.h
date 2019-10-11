/**-------------------------------------------------------------------------
@file	mag_ak09916.h

@brief	Implementation of Asahi Kasei AK09916 mag sensor


@author	Hoang Nguyen Hoan
@date	Sep. 8, 2019

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

#ifndef __MAG_AK09916_H__
#define __MAG_AK09916_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/mag_sensor.h"

#define AK09916_I2C_7BITS_DEVADDR							0xC

#define AK09916_WIA1_REG		0
#define AK09916_WIA1_COMPANY_ID								0x48

#define AK09916_WIA2_REG		1
#define AK09916_WIA2_DEVICE_ID								0x9

#define AK09916_COMPANY_DEVICE_ID							0x948		// Combine company & device id

#define AK09916_ST1_REG				0x10	//!< Status register 1
#define AK09916_ST1_DRDY									(1<<0)		//!< Data ready
#define AK09916_ST1_DOR										(1<<1)		//!< Data overrun

#define AK09916_HXL_REG				0x11
#define AK09916_HXH_REG				0x12
#define AK09916_HYL_REG				0x13
#define AK09916_HYH_REG				0x14
#define AK09916_HZL_REG				0x15
#define AK09916_HZH_REG				0x16

#define AK09916_ST2_REG				0x18	//!< Status register 2
#define AK09916_ST2_HOFL									(1<<3)		//!< Magnetic sensor overflow

#define AK09916_CTRL2_REG			0x31	//!< Control 2
#define AK09916_CTRL2_MODE_MASK								(0x1F<<0)
#define AK09916_CTRL2_MODE_POWERDWN							(0<<0)		//!< Power down
#define AK09916_CTRL2_MODE_SINGLE							(1<<0)		//!< Single measurement
#define AK09916_CTRL2_MODE_CONTINUOUS_10HZ					(2<<0)
#define AK09916_CTRL2_MODE_CONTINUOUS_20HZ					(4<<0)
#define AK09916_CTRL2_MODE_CONTINUOUS_50HZ					(6<<0)
#define AK09916_CTRL2_MODE_CONTINUOUS_100HZ					(8<<0)
#define AK09916_CTRL2_MODE_SELFTEST							(0x10<<0)	//!< Self-test

#define AK09916_CTRL3_REG			0x32
#define AK09916_CTRL3_SRST									(1<<0)

#define AK09916_ADC_RANGE					32752
#define AK09916_FLUX_DENSITY				4912000	//!< Max flux density in nT

#pragma pack(push, 1)


#pragma pack(pop)

#ifdef __cplusplus

class MagAk09916 : public MagSensor {
public:
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);

	/**
	 * @brief	Power off the device completely.
	 *
	 * If supported, this will put the device in complete power down.
	 * Full re-initialization is required to re-enable the device.
	 */
	virtual void PowerOff();
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
};


#endif // __cplusplus

#endif // __AG_AK09916_H__
