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

class Led : public Device {
public:
	/**
	 * @brief	Initialize as standard GPIO LED
	 *
	 * This function initializes a single LED connected on a GPIO without PWM
	 * dimming.
	 *
	 * @param	Port 	: GPIO port number
	 * @param	Pin 	: GPIO pin number
	 * @param	Active	: LED active logic level
	 *
	 * @return	true on success
	 */
	virtual bool Init(int Port, int Pin, LED_LOGIC Active);//, Pwm * const pPwm = NULL);

	/**
	 * @brief	Initialize as GPIO PWM LED
	 *
	 * This function initializes LED with PWM dimming of types
	 *    LED_TYPE_SINGLE_PWM
	 *    LED_TYPE_BICOLOR
	 *    LED_TYPE_TRICOLOR
	 *    LED_TYPE_QCOLOR
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
	 * @brief	Initialize as digital LED device
	 *
	 * This function initializes digital LED type.  This type of LED usually used as strip LED.
	 * The LED is controlled via a serial interface.
	 *
	 * @param	DevAddr	: Device address
	 * @param	pIntrf	: Pointer to control interface
	 * @param	NbLeds	: Number of LEDs to control
	 *
	 * @return	true on success
	 */
	virtual bool Init(uint8_t DevAddr, DeviceIntrf * const pIntrf, int NbLeds);

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

	virtual void Level(uint8_t * const pLedData, int Len);
	virtual void Level(uint32_t Level);
	virtual void On();
	virtual void Off();
	virtual void Toggle();

private:
	LED_DEV vLeds[LED_PIN_MAX];
	int	vNbLeds;
	Pwm *vpPwm;
	PWM_CHAN_CFG vPwmChanCfg[LED_PIN_MAX];
};

#endif // __LED_H__

