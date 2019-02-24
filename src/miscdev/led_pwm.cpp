/**-------------------------------------------------------------------------
@file	led_pwm.cpp

@brief	Generic implementation of LED control via PWM


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

bool LedPwm::Init(Pwm * const pPwm, PWM_CHAN_CFG * const pChanCfg, int NbChan)
{
	if (pPwm == nullptr || pChanCfg == nullptr || NbChan == 0)
	{
		return false;
	}

	Type(LED_TYPE_PWM);

	vpPwm = pPwm;
	vNbLeds = NbChan;

	memcpy(vPwmChanCfg, pChanCfg, vNbLeds * sizeof(PWM_CHAN_CFG));

	vpPwm->OpenChannel(vPwmChanCfg, vNbLeds);
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(vPwmChanCfg[i].Chan, 0);
	}
	vpPwm->Start();

	return true;
}

void LedPwm::Level(uint32_t Level)
{
	if (vpPwm == nullptr)
		return;

	uint8_t *p = (uint8_t*)&Level;
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(vPwmChanCfg[i].Chan, ((uint32_t)p[i] * 100UL) >> 8UL);
	}

	vpPwm->Start();

	vLevel = Level;
}

void LedPwm::On()
{
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(vPwmChanCfg[i].Chan, 100);
	}

	vpPwm->Start();

	vLevel = 0xFFFFFFFF;
}

void LedPwm::Off()
{
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(vPwmChanCfg[i].Chan, 0);
	}

	vpPwm->Stop();

	vLevel = 0;
}

void LedPwm::Toggle()
{
	Level(~vLevel);
}

bool LedPwm::Enable()
{
	if (vpPwm == NULL)
	{
		return false;
	}

	vpPwm->Enable();
	vpPwm->Start();

	return true;
}

void LedPwm::Disable()
{
	if (vpPwm)
	{
		vpPwm->Stop();
		vpPwm->Disable();
	}
}

void LedPwm::Reset()
{
	for (int i = 0; i < vNbLeds; i++)
	{
		vpPwm->DutyCycle(i, 0);
	}
	vpPwm->Start();
}

