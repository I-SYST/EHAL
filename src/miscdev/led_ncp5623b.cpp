/**-------------------------------------------------------------------------
@file	led_ncp5623b.cpp

@brief	Implementation of NCP5623B LED device class

@author	Hoang Nguyen Hoan
@date	Feb. 27, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include "idelay.h"
#include "iopinctrl.h"
#include "miscdev/led_ncp5623b.h"

bool LedNcp5623b::Init(DeviceIntrf * const pIntrf)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	Device::Interface(pIntrf);

	Device::DeviceAddress(NCP5623B_I2C_DEV_ADDR);

	uint8_t regaddr = 0;
	Write(&regaddr, 1, NULL, 0);

	regaddr = NCP5623B_ILED_REG | 0x1F;
	Write(&regaddr, 1, NULL, 0);

	return true;
}

void LedNcp5623b::Reset()
{
	uint8_t regaddr = 0;

	Write(&regaddr, 1, NULL, 0);
}

void LedNcp5623b::Level(uint32_t Val)
{
	uint8_t v = (Val >> 3) & NCP5623B_DATA_MASK;
	uint8_t regaddr = NCP5623B_PWM1_REG | v;

	Write(&regaddr, 1, NULL, 0);

	v = (Val >> 11) & NCP5623B_DATA_MASK;
	regaddr = NCP5623B_PWM2_REG | v;
	Write(&regaddr, 1, NULL, 0);

	v = (Val >> 19) & NCP5623B_DATA_MASK;
	regaddr = NCP5623B_PWM3_REG | v;
	Write(&regaddr, 1, NULL, 0);
}
