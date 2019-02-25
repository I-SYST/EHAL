/**-------------------------------------------------------------------------
@file	led_gpio.cpp

@brief	Implementation of basic LED control via gpio

Usage example

Standard LED controlled by GPIO pin (port 0, pin 30), LED turns on when pin is at level 0

Led g_Led1;

g_Led1.Init(0, 30, LED_LOGIC_LOW);

g_Led.On();

@author	Hoang Nguyen Hoan
@date	Feb. 23, 2019

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
#include "iopinctrl.h"
#include "miscdev/led.h"

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
bool Led::Init(int Port, int Pin, LED_LOGIC ActLevel)
{
	Type(LED_TYPE_GPIO);

	vPort = Port;
	vPin = Pin;
	vActLevel = ActLevel;

	IOPinConfig(vPort, vPin, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	if (vActLevel == LED_LOGIC_LOW)
	{
		IOPinSet(vPort, vPin);
	}
	else
	{
		IOPinClear(vPort, vPin);
	}

	return true;
}

void Led::On()
{
	if (vActLevel)
	{
		IOPinSet(vPort, vPin);
	}
	else
	{
		IOPinClear(vPort, vPin);
	}
}

void Led::Off()
{
	if (vActLevel)
	{
		IOPinClear(vPort, vPin);
	}
	else
	{
		IOPinSet(vPort, vPin);
	}
}

void Led::Toggle()
{
	IOPinToggle(vPort, vPin);
}
