/**-------------------------------------------------------------------------
@file	buzzer.cpp

@brief	Generic implementation of buzzer driver

@author	Hoang Nguyen Hoan
@date	May 22, 2018

@license

Copyright (c) 2018, I-SYST, all rights reserved

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
//#include <string.h>
#include "idelay.h"
#include "miscdev/buzzer.h"

bool Buzzer::Init(Pwm *pPwm, int Chan)
{
	if (pPwm == nullptr)
	{
		return false;
	}

	vpPwm = pPwm;
	vChan = Chan;

	return true;
}

void Buzzer::Volume(int Volume)
{
	if (Volume >= 0 && Volume <= 100)
	{
		// max volume is at 50% duty cycle
		vDutyCycle = Volume >> 1;
	}
}

/**
 * @brief	Play frequency
 *
 * @param	Freq :
 * @param	msDuration	: Play duration in msec.
 *							if != 0, wait for it then stop
 *							else let pwm running and return (no stop)
 */
void Buzzer::Play(uint32_t Freq, uint32_t msDuration)
{
	vpPwm->Frequency(Freq);
	//vpPwm->DutyCycle(vChan, vDutyCycle);
	vpPwm->Start(msDuration);

	if (msDuration)
	{
		// Wait for duration completion
		usDelay(msDuration * 1000);
		//vpPwm->Stop();
	}
}

void Buzzer::Stop()
{
	vpPwm->Stop();
}



