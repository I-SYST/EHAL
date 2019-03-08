/**-------------------------------------------------------------------------
@example	pwm_demo.cpp

@brief	Example code using PWM (Pulse Width Modulation)

@author	Hoang Nguyen Hoan
@date	May 15, 2018

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
#include <string.h>

#include "pwm.h"
#include "idelay.h"

static const PWM_CFG s_PwmCfg = {
	.DevNo = 0,
	.Freq = 1000,
	.Mode = PWM_MODE_EDGE,
	.bIntEn = false,
	.IntPrio = 6,
	.pEvtHandler = NULL
};

static const PWM_CHAN_CFG s_PwmChanCfg[] = {
	{
		.Chan = 0,
		.Pol = PWM_POL_HIGH,
		.Port = 0,
		.Pin = 25,
	},
	{
		.Chan = 1,
		.Pol = PWM_POL_HIGH,
		.Port = 0,
		.Pin = 22,
	},
};

const int s_NbPwmChan = sizeof(s_PwmChanCfg) / sizeof(PWM_CHAN_CFG);

Pwm g_Pwm;


//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	g_Pwm.Init(s_PwmCfg);
	g_Pwm.OpenChannel(s_PwmChanCfg, s_NbPwmChan);

	// Set duty cycle 20% on channel 0
	g_Pwm.DutyCycle(0, 50);
	g_Pwm.DutyCycle(1, 25);

	g_Pwm.Start();

	// Let it runs for 1 sec
	usDelay(1000000);

	g_Pwm.Stop();

	// Change PWM frequency
	g_Pwm.Frequency(3333333);
	g_Pwm.DutyCycle(0, 50);
	g_Pwm.DutyCycle(1, 25);

	g_Pwm.Start();

	int x = 0;

	while (1)
	{
#if 0
		// Change duty cycle
		g_Pwm.DutyCycle(0, x);
		x+=1;
		if (x > 100)
			x = 0;
#else
		// Change PWM frequency
		//g_Pwm.Stop();
		g_Pwm.Frequency(x);
		g_Pwm.DutyCycle(0, 50);
		g_Pwm.DutyCycle(1, 25);
		//g_Pwm.Start();
		x += 10;
		if (x > 8000000)
			x = 0;
#endif
		usDelay(5000);


		//__WFE();
	}
	return 0;
}
