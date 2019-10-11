/**-------------------------------------------------------------------------
@file	led_ncp5623b.h

@brief	Implementation of NCP5623B LED controller class

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

#ifndef __LED_NCP5623B_H__
#define __LED_NCP5623B_H__

#include "led.h"

#define NCP5623B_I2C_DEV_ADDR			0x38U	// 7 bits I2C address

#define NCP5623B_SHUTDOWN				0
#define NCP5623B_ILED_REG				(1<<5)
#define NCP5623B_PWM1_REG				(2<<5)
#define NCP5623B_PWM2_REG				(3<<5)
#define NCP5623B_PWM3_REG				(4<<5)
#define NCP5623B_GDUP_REG				(5<<5)
#define NCP5623B_GDDWN_REG				(6<<5)
#define NCP5623B_GDRUN_REG				(7<<5)
#define NCP5623B_CMD_MASK				(7<<5)
#define NCP5623B_DATA_MASK				0x1F

class LedNcp5623b : public LedDevice, public Device  {
public:
	bool Init(DeviceIntrf *const pIntrf);

	/**
	 * Turns 1st LED 100% on
	 *
	 *
	 */
	virtual void On() { Level(0xFFFFFFFF); }

	/**
	 * Turns first LED off
	 */
	virtual void Off() { Level(0); }

	/**
	 * Toggle or invert all LED dimming level
	 *
	 * NOT SUPPORTED
	 */
	virtual void Toggle() {}

	/**
	 * @brief	Set LED level
	 *
	 * This function set the dimming level of the LED 0-255.  On multi-color LED can be
	 * used to mix color.  Usually used for PWM analog led
	 *
	 * @param Level	: LED dimming Level 0-255.  0 = Off, 255 = 100% On. Up to 4 LEDs can be dimmed.
	 * 					Bits 0-7  	: LED 0
	 * 					Bits 8-15 	: LED 1
	 * 					Bits 16-23	: LED 2
	 * 					Bits 24-31	: LED 3
	 *
	 */
	virtual void Level(uint32_t Val);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable() { return true; }

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 */
	virtual void Disable() {}

	/**
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset();

private:
};

#endif // __LED_NCP5623B_H__
