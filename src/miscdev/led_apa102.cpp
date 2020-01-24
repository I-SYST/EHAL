/**-------------------------------------------------------------------------
@file	led_apa102.cpp

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
#include "idelay.h"
#include "iopinctrl.h"
#include "miscdev/led_apa102.h"

bool LedApa102::Init(APA102_CFG &Cfg)
{
	IOPinConfig(Cfg.CIPortNo, Cfg.CIPinNo, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(Cfg.DIPortNo, Cfg.DIPinNo, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	vNbLed = Cfg.NbLed;
	vCIPortNo = Cfg.CIPortNo;
	vCIPinNo = Cfg.CIPinNo;
	vDIPortNo = Cfg.DIPortNo;
	vDIPinNo = Cfg.DIPinNo;
	vBrightness = Cfg.Brightness;

	return true;
}

/**
 * Turns all LED 100% on
 */
void LedApa102::On()
{
	uint32_t Val = APA102_ON;

	Level(&Val, 1);
}

/**
 * Turns all LED off
 */
void LedApa102::Off()
{
	uint32_t Val = APA102_OFF;

	Level(&Val, 1);
}

/**
 * @brief	Set LED level for strip LED.
 *
 * This function sets the levels strip RGB strip LED.  These LEDs are normally
 * controlled via a serial interface.
 *
 * @param	pLevel : pointer to array of RGB LED to set
 * @param 	NbLeds : Number of LED to set.
 */
void LedApa102::Level(uint32_t * const pVal, int NbLeds, int Repeat)
{
	uint32_t bit = 0x80000000;

	// Start frame
	IOPinClear(vDIPortNo, vDIPinNo);
	while (bit != 0)
	{
		IOPinClear(vCIPortNo, vCIPinNo);
		//usDelay(1);
		IOPinSet(vCIPortNo, vCIPinNo);

		bit >>= 1;
	}

	do {
		for (int i = 0; i < NbLeds; i++)
		{
			uint32_t p = pVal[i];
			bit = 0x80000000;
			p = (p & 0xFFFFFF) | ((0xe0 | vBrightness) << 24);

			while (bit != 0)
			{
				IOPinClear(vCIPortNo, vCIPinNo);

				if (p & bit)
				{
					IOPinSet(vDIPortNo, vDIPinNo);
				}
				else
				{
					IOPinClear(vDIPortNo, vDIPinNo);
				}
				IOPinSet(vCIPortNo, vCIPinNo);
				bit >>= 1;
			}
		}
	} while (Repeat-- > 0);

	// Stop frame
	bit = 0x80000000;
	IOPinSet(vDIPortNo, vDIPinNo);
	while (bit != 0)
	{
		IOPinClear(vCIPortNo, vCIPinNo);
		//usDelay(1);
		IOPinSet(vCIPortNo, vCIPinNo);

		bit >>= 1;
	}
}
