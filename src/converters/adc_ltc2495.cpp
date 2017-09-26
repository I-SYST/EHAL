/*--------------------------------------------------------------------------
File   : adc_ltc2495.cpp

Author : Hoang Nguyen Hoan          June 16, 2017

Desc   : ADC implementation for LTC2495

Copyright (c) 2017, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include "converters/adc_ltc2495.h"

/**
 * @brief	Execute auto calibration
 *
 * @return	true - success
 */
bool ADCLTC2495::Calibrate()
{
	return true;
}

bool ADCLTC2495::Init(const ADC_CFG &Cfg, DeviceIntrf *pIntrf)
{
	SetInterface(pIntrf);
	SetDeviceAddess(Cfg.DevAddr);

	if (Cfg.NbRefVolt < 1 || Cfg.pRefVolt == NULL)
		return false;

	SetRefVoltage(Cfg.pRefVolt, Cfg.NbRefVolt);

	vResolution = Cfg.Resolution;
	vRate = Cfg.Rate;
	vMode = Cfg.Mode;
	vbInterrupt = false; // Interrupt not available
	SetEvtHandler(NULL);

	return true;
}

bool ADCLTC2495::OpenChannel(const ADC_CHAN_CFG *pChanCfg, int NbChan)
{
	uint8_t d[2];

	d[0] = 0xA0;
	d[1] = 0x80;

	for (int i = 0; i < NbChan; i++)
	{
		if (pChanCfg->Type == ADC_CHAN_TYPE_DIFFERENTIAL)
		{
			d[0] |= 0x40 | pChanCfg[i].Chan;
		}
		else
		{
			d[0] |= pChanCfg[i].Chan;
		}

		uint32_t gain = pChanCfg[i].Gain >> 8;	// Fractional gain not available

		d[1] |= 0x80 | ((31 - __builtin_clzl(gain)) & 0xFF);

		Write(&d[0], 1, &d[1], 1);

		gain = (gain & 0x7) << 2;

		vVFullScale[pChanCfg[i].Chan] = 0.5 * vRefVoltage;
		if (gain > 0)
			vVFullScale[pChanCfg[i].Chan] /= gain;

	}
}

void ADCLTC2495::CloseChannel(int Chan)
{

}

bool ADCLTC2495::StartConversion()
{
	uint8_t d[4];
/*
	d[0] = Chan;
	d[1] = Gain;

	Write(&d[0], 1, &d[1], 1);

	int gain = (Gain & 0x7) << 2;

	vVFullScale = 0.5 * vRefVoltage;
	if (gain > 0)
		vVFullScale /= gain;
*/
	return true;
}
/*
float AdcLtc2495::Read(int Chan)
{
	uint8_t d[4];
	int val;
	uint8_t status;

	d[0] = 0;
	d[1] = 0;
	d[2] = 0;
	d[3] = 0;

	Device::Read((uint8_t*)&Chan, 1, (uint8_t*)d, 3);

	val = (d[2] | ((int32_t)d[1] << 8) | ((int32_t)(d[0] & 0x3f) << 16)) >> 6;
	status = d[0] >> 6;

	if (status == 0 || status == 3)
	{
		return 0.0;
	}
	if (status == 1)
	{
		// negative, make complement 2
		val |= 0xFFFF0000;
	}

	float V = val * vVFullScale / 0xFFFF;

	//V /= Divider;

	return V;
}

int AdcLtc2495::Read(float *pBuff, int Len)
{
	return 0;
}
*/
int ADCLTC2495::Read(ADC_DATA *pBuff, int Len)
{

}
