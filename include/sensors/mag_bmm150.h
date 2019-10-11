/**-------------------------------------------------------------------------
@file	mag_bmm150.h

@brief	Implementation of BOSCH BMM150 mag sensor


@author	Hoang Nguyen Hoan
@date	Aug. 23, 2019

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

#ifndef __MAG_BMM150_H__
#define __MAG_BMM150_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/mag_sensor.h"

#define BMM150_I2C_7BITS_DEVADDR							0x10	// CSB = 0, SDO = 0
#define BMM150_I2C_7BITS_DEVADDR_ALT1						0x11	// CSB = 0, SDO = 1
#define BMM150_I2C_7BITS_DEVADDR_ALT2						0x12	// CSB = 1, SDO = 0
#define BMM150_I2C_7BITS_DEVADDR_ALT3						0x13	// CSB = 1, SDO = 1

#define BMM150_CHIP_ID_REG          0x40 // Note: this is readable only when bit0 of reg 0x4B is enabled

#define BMM150_CHIP_ID                                      0x32

#define BMM150_DATA_X_LSB_REG		0x42
#define BMM150_DATA_X_LSB_SELFTEST							(1<<0)
#define BMM150_DATA_X_LSB_MASK								(0x1F<<3) //!< Data bit 0:4
#define BMM150_DATA_X_MSB_REG		0x43 //!< Data bit 5:12

#define BMM150_DATA_Y_LSB_REG		0x44
#define BMM150_DATA_Y_LSB_SELFTEST							(1<<0)
#define BMM150_DATA_Y_LSB_MASK								(0x1F<<3) //!< Data bit 0:4
#define BMM150_DATA_Y_MSB_REG		0x45 //!< Data bit 5:12

#define BMM150_DATA_Z_LSB_REG		0x46
#define BMM150_DATA_Z_LSB_SELFTEST							(1<<0)
#define BMM150_DATA_Z_LSB_MASK								(0x7F<<1) //!< Data bit 0:6
#define BMM150_DATA_Z_MSB_REG		0x47 //!< Data bit 7:14

#define BMM150_RHALL_LSB_REG		0x48
#define BMM150_RHALL_LSB_DATA_RDY							(1<<0)
#define BMM150_RHALL_LSB_MASK								(0x3F<<2) //!< Data bit 0:5
#define BMM150_RHALL_MSB_REG		0x49 //!< Data bit 6:13

#define BMM150_INT_STATUS_REG		0x4A
#define BMM150_INT_STATUS_LOW_X								(1<<0)
#define BMM150_INT_STATUS_LOW_Y								(1<<1)
#define BMM150_INT_STATUS_LOW_Z								(1<<2)
#define BMM150_INT_STATUS_HIGH_X							(1<<3)
#define BMM150_INT_STATUS_HIGH_Y							(1<<4)
#define BMM150_INT_STATUS_HIGH_Z							(1<<5)
#define BMM150_INT_STATUS_OVERFLOW							(1<<6)
#define BMM150_INT_STATUS_DATA_OVERRUN						(1<<7)


#define BMM150_CTRL1_REG			0x4B
#define BMM150_CTRL1_POWER_ON								(1<<0)
#define BMM150_CTRL1_SOFT_RESET								(1<<1)
#define BMM150_CTRL1_SPI3_EN								(1<<2)
#define BMM150_CTRL1_SOFT_RESET2							(1<<7)

#define BMM150_CTRL2_REG			0x4C
#define BMM150_CTRL2_SELFTEST								(1<<0)
#define BMM150_CTRL2_OPMODE_MASK							(3<<1)
#define BMM150_CTRL2_OPMODE_NORMAL							(0<<1)
#define BMM150_CTRL2_OPMODE_FORCED							(1<<1)
#define BMM150_CTRL2_OPMODE_SLEEP							(3<<1)
#define BMM150_CTRL2_ODR_MASK								(7<<3)
#define BMM150_CTRL2_ODR_10HZ								(0<<3)
#define BMM150_CTRL2_ODR_2HZ								(1<<3)
#define BMM150_CTRL2_ODR_6HZ								(2<<3)
#define BMM150_CTRL2_ODR_8HZ								(3<<3)
#define BMM150_CTRL2_ODR_15HZ								(4<<3)
#define BMM150_CTRL2_ODR_20HZ								(5<<3)
#define BMM150_CTRL2_ODR_25HZ								(6<<3)
#define BMM150_CTRL2_ODR_30HZ								(7<<3)
#define BMM150_CTRL2_ADV_SELFTEST0							(1<<6)
#define BMM150_CTRL2_ADV_SELFTEST1							(1<<7)

#define BMM150_INT_ENABLE_REG		0x4D
#define BMM150_INT_ENABLE_LOW_X								(1<<0)
#define BMM150_INT_ENABLE_LOW_Y								(1<<1)
#define BMM150_INT_ENABLE_LOW_Z								(1<<2)
#define BMM150_INT_ENABLE_HIGH_X							(1<<3)
#define BMM150_INT_ENABLE_HIGH_Y							(1<<4)
#define BMM150_INT_ENABLE_HIGH_Z							(1<<5)
#define BMM150_INT_ENABLE_OVERFLOW							(1<<6)
#define BMM150_INT_ENABLE_DATA_OVERRUN						(1<<7)

#define BMM150_CTRL3_REG			0x4E
#define BMM150_CTRL3_INT_POL								(1<<0) //!< Interrupt polarity 0-active low, 1-active high
#define BMM150_CTRL3_INT_LATCH								(1<<1)
#define BMM150_CTRL3_DATA_RDY_POL							(1<<2) //!< Data ready polarity
#define BMM150_CTRL3_CHAN_X_DIS								(1<<3)
#define BMM150_CTRL3_CHAN_Y_DIS								(1<<4)
#define BMM150_CTRL3_CHAN_Z_DIS								(1<<5)
#define BMM150_CTRL3_INT_PIN_EN								(1<<6)
#define BMM150_CTRL3_DATA_RDY_PIN_EN						(1<<7)

#define BMM150_LOW_THRESHOLD_REG	0x4F
#define BMM150_HIGH_THRESHOLD_REG	0x50
#define BMM150_REPETITION_XY_REG	0x51
#define BMM150_REPETITION_Z_REG		0x52

#define BMI150_ADC_RANGE_XY			((1<<12) - 1)	// 13 bits
#define BMI150_ADC_RANGE_Z			((1<<14) - 1)	// 13 bits

#define BMI150_FLUX_DENSITY_XY		1300000 //!< max flux density in nT
#define BMI150_FLUX_DENSITY_Z		2500000 //!< max flux density in nT

#pragma pack(push, 1)


#pragma pack(pop)

#ifdef __cplusplus

class MagBmm150 : public MagSensor {
public:
	virtual bool Init(const MAGSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual uint32_t SamplingFrequency(uint32_t Freq);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
};


#endif // __cplusplus

#endif // __AG_BMI160_H__
