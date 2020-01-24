/**-------------------------------------------------------------------------
@file	led_apa102.h

@brief	Implementation of APA102 LED device class

@author	Hoang Nguyen Hoan
@date	Feb. 27, 2019

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

#ifndef __LED_APA102_H__
#define __LED_APA102_H__

#include "led.h"

#define APA102_BLUE			0xFF0000
#define APA102_GREEN		0x00FF00
#define APA102_RED			0x0000FF

#define APA102_OFF			0
#define APA102_ON			0xFFFFFF

typedef struct __APA102_Config {
	int NbLed;
	uint8_t CIPortNo;
	uint8_t CIPinNo;
	uint8_t DIPortNo;
	uint8_t DIPinNo;
	uint8_t Brightness;
} APA102_CFG;

class LedApa102 : public LedDevice {
public:
	bool Init(APA102_CFG &Cfg);

	/**
	 * Turns 1st LED 100% on
	 *
	 *
	 */
	virtual void On();

	/**
	 * Turns first LED off
	 */
	virtual void Off();

	/**
	 * Toggle or invert all LED dimming level
	 *
	 * NOT SUPPORTED
	 */
	virtual void Toggle() {}

	/**
	 * @brief	Set LED level
	 *
	 * This function set the dimming level of the LED 0-255.  On multicolor LED can be
	 * used to mix color.  Usually used for PWM analog led
	 *
	 * @param Level	: LED dimming Level 0-255.  0 = Off, 255 = 100% On. Up to 4 LEDs can be dimmed.
	 * 				  For APA102 :
	 * 					Bits 0-7  	: LED 0 (Red)
	 * 					Bits 8-15 	: LED 1 (Green)
	 * 					Bits 16-23	: LED 2 (Blue)
	 *
	 */
	virtual void Level(uint32_t Val) { Level(&Val, 1); }

	/**
	 * @brief	Set LED level for strip LED.
	 *
	 * This function sets the levels strip RGB strip LED.  These LEDs are normally
	 * controlled via a serial interface.
	 *
	 * @param	pLevel : pointer to array of RGB LED to set
	 * @param 	NbLeds : Number of LED to set.
	 * @param	Repeat : Repeat count.
	 */
	virtual void Level(uint32_t * const pVal, int NbLeds, int Repeat = 0);

private:
	int vNbLed;			//!< Total number of Led in strip
	uint8_t vCIPortNo;	//!< Clock pin i/o port number
	uint8_t vCIPinNo;	//!< Clock pin i/o pin number
	uint8_t vDIPortNo;	//!< Data pin i/o port number
	uint8_t vDIPinNo;	//!< Data pin i/o pin number
	uint8_t vBrightness;//!< Brightness value 0-31
};

#endif // __LED_APA102_H__
