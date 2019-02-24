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

bool Led::Init(int Port, int Pin, LED_LOGIC Active)//, Pwm * const pPwm)
{
	vNbLeds = 1;
	vLeds[0].Chan = 0;
	vLeds[0].Port = Port;
	vLeds[0].Pin = Pin;
	vLeds[0].Act = Active;
	vLeds[0].Type = LED_TYPE_GPIO;

	vpPwm = nullptr;

	IOPinConfig(vLeds[0].Port, vLeds[0].Pin, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	if (Active == LED_LOGIC_LOW)
	{
		IOPinSet(vLeds[0].Port, vLeds[0].Pin);
	}
	else
	{
		IOPinClear(vLeds[0].Port, vLeds[0].Pin);
	}

	return true;
}

bool Led::Init(Pwm * const pPwm, PWM_CHAN_CFG * const pChanCfg, int NbChan)
{
	if (pPwm == nullptr || pChanCfg == nullptr || NbChan == 0)
	{
		return false;
	}

	vpPwm = pPwm;
	vNbLeds = NbChan;

	memcpy(vPwmChanCfg, pChanCfg, vNbLeds * sizeof(PWM_CHAN_CFG));

	vpPwm->OpenChannel(vPwmChanCfg, vNbLeds);
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(vPwmChanCfg[i].Chan, 0);
		vLeds[i].Chan = pChanCfg[i].Chan;
		vLeds[i].Port = pChanCfg[i].Port;
		vLeds[i].Pin = pChanCfg[i].Pin;
		vLeds[i].Act = pChanCfg[i].Pol == PWM_POL_LOW ? LED_LOGIC_LOW : LED_LOGIC_HIGH;
		vLeds[i].Type = LED_TYPE_PWM;
	}
	vpPwm->Start();

	return true;
}

bool Led::Init(uint8_t DevAddr, DeviceIntrf * const pIntrf, int NbLeds)
{
	Interface(pIntrf);

	vNbLeds = NbLeds;

	vLeds[0].Chan = 0;
	vLeds[0].Port = -1;
	vLeds[0].Pin = -1;
	vLeds[0].Act = LED_LOGIC_HIGH;
	vLeds[0].Type = LED_TYPE_STRIP;

	return true;
}

void Led::Level(uint8_t * const pLedData, int Len)
{

}

void Led::Level(uint32_t Level)
{
	if (vpPwm == nullptr)
		return;

	uint8_t *p = (uint8_t*)&Level;
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(vPwmChanCfg[i].Chan, ((uint32_t)p[i] * 100UL) >> 8UL);
	}
}

void Led::On()
{
	switch (vLeds[0].Type)
	{
		case LED_TYPE_GPIO:
			if (vLeds[0].Act)
			{
				IOPinSet(vLeds[0].Port, vLeds[0].Pin);
			}
			else
			{
				IOPinClear(vLeds[0].Port, vLeds[0].Pin);
			}
			break;
		case LED_TYPE_PWM:
			for (int i = 0; i < vNbLeds; i++)
			{
				vpPwm->DutyCycle(vPwmChanCfg[i].Chan, 100);
			}
			break;
		case LED_TYPE_STRIP:
			break;
	}
}

void Led::Off()
{
	switch (vLeds[0].Type)
	{
		case LED_TYPE_GPIO:
			if (vLeds[0].Act)
			{
				IOPinClear(vLeds[0].Port, vLeds[0].Pin);
			}
			else
			{
				IOPinSet(vLeds[0].Port, vLeds[0].Pin);
			}
			break;
		case LED_TYPE_PWM:
			for (int i = 0; i < vNbLeds; i++)
			{
				vpPwm->DutyCycle(vPwmChanCfg[i].Chan, 0);
			}
			break;
		case LED_TYPE_STRIP:
			break;
	}
}

void Led::Toggle()
{
	if (vLeds[0].Type == LED_TYPE_GPIO)
	{
		IOPinToggle(vLeds[0].Port, vLeds[0].Pin);
	}
}

bool Led::Enable()
{
	if (vpPwm)
	{
		vpPwm->Enable();
		vpPwm->Start();
	}
	return true;
}

void Led::Disable()
{
	if (vpPwm)
	{
		vpPwm->Stop();
		vpPwm->Disable();
	}
}

void Led::Reset()
{
	if (vpPwm)
	{
		for (int i = 0; i < vNbLeds; i++)
		{
			vpPwm->DutyCycle(i, 0);
		}
		vpPwm->Start();
	}
	else
	{
		for (int i = 0; i < vNbLeds; i++)
		{
			if (vLeds[i].Act == LED_LOGIC_LOW)
			{
				IOPinSet(vLeds[i].Port, vLeds[i].Pin);
			}
			else
			{
				IOPinClear(vLeds[i].Port, vLeds[i].Pin);
			}
		}
	}
}
