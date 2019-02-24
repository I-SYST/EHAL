/**-------------------------------------------------------------------------
@file	led.h

@brief	Generic implementation of LED device


@author	Hoang Nguyen Hoan
@date	Feb. 13, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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
#ifndef __LED_H__
#define __LED_H__

#include "device.h"
#include "device_intrf.h"
#include "pwm.h"

/** @addtogroup MiscDev
  * @{
  */

#define LED_PIN_MAX			4

/// LED types
typedef enum __LED_Type {
	LED_TYPE_GPIO,			//!< Single led Gpio On/Off
	LED_TYPE_PWM,			//!< Single led Gpio with pwm
	LED_TYPE_STRIP			//!< Digital interface strip led
} LED_TYPE;

/// LED logic level.
/// This defines only for gpio type
typedef enum __LED_Logic_level {
	LED_LOGIC_LOW = 0,		//!< LED_LOGIC_LOW
	LED_LOGIC_HIGH = 1		//!< LED_LOGIC_HIGH
} LED_LOGIC;

typedef struct __LED_Dev {
	int Chan;
	uint8_t Port;
	uint8_t Pin;
	LED_LOGIC Act;
	LED_TYPE Type;
} LED_DEV;

/// LED device abstract base class
///
class LedDevice {
public:

	/**
	 * Turns all LED 100% on
	 */
	virtual void On() = 0;

	/**
	 * Turns all LED off
	 */
	virtual void Off() = 0;

	/**
	 * Toggle or invert all LED dimming level
	 */
	virtual void Toggle() = 0;

	/**
	 * Get LED type
	 *
	 * @return	LED type
	 */
	LED_TYPE Type() { return vType; }

	/**
	 * Set LED type
	 *
	 * @return	LED type set.
	 */
	LED_TYPE Type(LED_TYPE Type) { vType = Type; return vType; }

protected:
private:
	LED_TYPE vType;
};

/// Basic Led type controlled by GPIO on/off
class Led : public LedDevice {
public:
	/**
	 * @brief	Initialize as standard GPIO LED
	 *
	 * This function initializes a single LED connected on a GPIO without PWM
	 * dimming.
	 *
	 * @param	Port 		: GPIO port number
	 * @param	Pin 		: GPIO pin number
	 * @param	ActLevel	: LED active logic level
	 *
	 * @return	true on success
	 */
	virtual bool Init(int Port, int Pin, LED_LOGIC ActLevel);

	/**
	 * Turns LED on.
	 */
	virtual void On();

	/**
	 * Turns LED off.
	 */
	virtual void Off();

	/**
	 * Toggle LED.
	 */
	virtual void Toggle();

private:

	LED_TYPE vType;
	int vPort;
	int vPin;
	LED_LOGIC vActLevel;
};

/// Led type controlled by PWM
class LedPwm : public LedDevice {
public:
	/**
	 * @brief	Initialize as GPIO PWM LED
	 *
	 * This function initializes LED with GPIO PWM dimming.  Dimming level 0-255.
	 * Can be used for single, bi-color, tri-color or quad-color led
	 *
	 *
	 * @param	pPwm		: Pointer to PWM object, this pointer is kept internally.
	 * 							Do not delete this before the Led object
	 * @param	pChanCfg	: Array of Pwm channel configuration data
	 * @param	NbChan		: Number of PWM channels, 1 channel config is required per LED
	 *
	 * @return	true on success
	 */
	virtual bool Init(Pwm * const pPwm, PWM_CHAN_CFG * const pPwmChan, int NbChan);

	/**
	 * @brief	Set LED level
	 *
	 * This function set the dimming level of the LED 0-255.  On multi-color LED can be
	 * used to mix color.
	 *
	 * @param Level	: LED dimming Level 0-255.  0 = Off, 255 = 100% On. Up to 4 LEDs can be dimmed.
	 * 					Bits 0-7  	: LED 0
	 * 					Bits 8-15 	: LED 1
	 * 					Bits 16-23	: LED 2
	 * 					Bits 24-31	: LED 3
	 *
	 */
	virtual void Level(uint32_t Level);

	/**
	 * Turns all LED 100% on
	 */
	virtual void On();

	/**
	 * Turns all LED off
	 */
	virtual void Off();

	/**
	 * Invert all LED dimming level
	 */
	virtual void Toggle();

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 */
	virtual void Disable();

	/**
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset();

private:
	uint32_t vLevel;
	int	vNbLeds;
	Pwm *vpPwm;
	PWM_CHAN_CFG vPwmChanCfg[LED_PIN_MAX];
};

/** @} end group IMU */

#endif // __LED_H__

