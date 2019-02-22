/**-------------------------------------------------------------------------
@file	led.cpp

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
#include "iopinctrl.h"
#include "miscdev/led.h"

bool Led::Init(int Port, int Pin, LED_LOGIC Active, Pwm * const pPwm)
{
	vNbLeds = 1;
	vLeds[0].Port = Port;
	vLeds[0].Pin = Pin;
	vLeds[0].Act = Active;

	vpPwm = pPwm;

	IOPinConfig(vLeds[0].Port, vLeds[0].Pin, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	return true;
}

bool Led::Init(LED_DEV * const pLedArray, int Count, Pwm * const pPwm)
{
	vNbLeds = Count;

	for (int i = 0; i < vNbLeds; i++)
	{
		vLeds[i].Port = pLedArray[i].Port;
		vLeds[i].Pin = pLedArray[i].Pin;
		vLeds[i].Act = pLedArray[i].Act;

		IOPinConfig(vLeds[i].Port, vLeds[i].Pin, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	}
	vpPwm = pPwm;

	return true;
}

bool Led::Init(DeviceIntrf * const pIntrf)
{
	Interface(pIntrf);

	vNbLeds = 0;

	return true;
}

void Led::Level(uint32_t Level)
{
	if (vpPwm == nullptr)
		return;

	uint8_t *p = (uint8_t*)Level;
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(i, p[i]);
	}
}

void Led::On()
{
	for (int i = 0; i < vNbLeds; i++)
	{
		if (vLeds[i].Act)
		{
			IOPinSet(vLeds[i].Port, vLeds[i].Pin);
		}
		else
		{
			IOPinClear(vLeds[i].Port, vLeds[i].Pin);
		}
	}
}

void Led::Off()
{
	for (int i = 0; i < vNbLeds; i++)
	{
		if (vLeds[i].Act)
		{
			IOPinClear(vLeds[i].Port, vLeds[i].Pin);
		}
		else
		{
			IOPinSet(vLeds[i].Port, vLeds[i].Pin);
		}
	}
}

void Led::Toggle()
{
	for (int i = 0; i < vNbLeds; i++)
	{
		IOPinToggle(vLeds[i].Port, vLeds[i].Pin);
	}
}



