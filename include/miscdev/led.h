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

typedef enum {
	LED_LOGIC_LOW = 0,
	LED_LOGIC_HIGH = 1
} LED_LOGIC;

typedef struct __LED_Dev {
	uint8_t Port;
	uint8_t Pin;
	LED_LOGIC Act;
} LED_DEV;

class Led : public Device {
public:
	bool Init(int Port, int Pin, LED_LOGIC Active, Pwm * const pPwm = NULL);
	bool Init(LED_DEV * const pLedArray, int Count, Pwm * const pPwm = NULL);
	bool Init(DeviceIntrf * const pIntrf);

	virtual void Level(uint32_t Level);
	virtual void On();
	virtual void Off();
	virtual void Toggle();

private:
	LED_DEV vLeds[LED_PIN_MAX];
	int	vNbLeds;
	Pwm *vpPwm;
};

#endif // __LED_H__

