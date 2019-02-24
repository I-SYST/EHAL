/**-------------------------------------------------------------------------
@example	led_demo.cpp

@brief	Example code using LED object.


@author	Hoang Nguyen Hoan
@date	Feb. 23, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

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
#include "blueio_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "miscdev/led.h"

static const PWM_CFG s_PwmCfg = {
	.DevNo = 0,
	.Freq = 100,
	.Mode = PWM_MODE_EDGE,
	.bIntEn = false,
	.IntPrio = 6,
	.pEvtHandler = NULL
};

static const PWM_CHAN_CFG s_PwmChanCfg[] = {
	{
		.Chan = 0,
		.Pol = PWM_POL_LOW,
		.Port = BLUEIO_LED2_PORT,
		.Pin = BLUEIO_LED2_PIN,
	},
	{
		.Chan = 1,
		.Pol = PWM_POL_LOW,
		.Port = BLUEIO_LED3_PORT,
		.Pin = BLUEIO_LED3_PIN,
	},
	{
		.Chan = 2,
		.Pol = PWM_POL_LOW,
		.Port = BLUEIO_LED4_PORT,
		.Pin = BLUEIO_LED4_PIN,
	},
};

const int s_NbPwmChan = sizeof(s_PwmChanCfg) / sizeof(PWM_CHAN_CFG);

Pwm g_Pwm;

Led g_Led1;
LedPwm g_Led2;

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

	g_Led1.Init(BLUEIO_LED1_PORT, BLUEIO_LED1_PIN, LED_LOGIC_LOW);

	g_Led2.Init(&g_Pwm, (PWM_CHAN_CFG*)s_PwmChanCfg, s_NbPwmChan);
	g_Led1.On();
	g_Led2.Level(100);

	g_Led1.Off();

	uint32_t x = 0xFFFFFFFF;

	while (1)
	{
		g_Led1.Toggle();
		g_Led2.Level(x);

		msDelay(100);

		g_Led2.Toggle();

		x >>= 1;

		if (x == 0)
		{
			x = 0xFFFFFFFF;
		}
		msDelay(20);
	}
}
