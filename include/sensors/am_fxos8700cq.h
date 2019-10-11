/**-------------------------------------------------------------------------
@file	am_fxos8700cq.h

@brief	Implementation of NXP FXOS8700CQ accel, mag sensor

This device is a combo accelerometer and magnetometer

@author	Hoang Nguyen Hoan
@date	Sep. 5, 2019

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

#ifndef __AM_FXOS8700CQ_H__
#define __AM_FXOS8700CQ_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "sensors/accel_sensor.h"
#include "sensors/mag_sensor.h"
#include "sensors/temp_sensor.h"

#define FXOS8700CQ_I2C_DEVADDR				0x1E	//!< 7 bit i2c address SA0 = 0, SA1 = 0
#define FXOS8700CQ_I2C_DEVADDR1				0x1D	//!< 7 bit i2c address SA0 = 1, SA1 = 0
#define FXOS8700CQ_I2C_DEVADDR2				0x1C	//!< 7 bit i2c address SA0 = 0, SA1 = 1
#define FXOS8700CQ_I2C_DEVADDR3				0x1F	//!< 7 bit i2c address SA0 = 1, SA1 = 1

// ***** XL
#define FXOS8700CQ_WHO_AM_I_A_REG			0xD
#define FXOS8700CQ_WHO_AM_I_A_ID								0xC7

#define FXOS8700CQ_STATUS_REG				0x0
#define FXOS8700CQ_OUT_X_MSB_REG			0x1



// Accel, Mag, Temperature
class AmFxos8700cq : public AccelSensor, public MagSensor {
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

	virtual uint16_t Scale(uint16_t Value) { return 0; }			// Accel

	virtual bool Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf = NULL, Timer * const pTimer = NULL);

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

	virtual bool Read(ACCELSENSOR_RAWDATA &Data) { return AccelSensor::Read(Data); }
	virtual bool Read(ACCELSENSOR_DATA &Data) { return AccelSensor::Read(Data); }
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

#endif // __AM_FXOS8700CQ_H__
