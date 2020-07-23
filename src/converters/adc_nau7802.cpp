/**-------------------------------------------------------------------------
@file	adc_nau7802.cpp

@brief	Nuvoton NAU7802 24-Bit Dual-Channel ADC For Bridge Sensors

This device has 2 channels multiplexed. Same settings for both.
For single channel operation, channel 0 (VIN1P/N) must used and filter caps must be
enabled on channel 1 (VIN2P/N)

@author	Hoang Nguyen Hoan
@date	Nov. 3, 2019

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
#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "istddef.h"
#include "idelay.h"
#include "convutil.h"
#include "converters/adc_nau7802.h"

bool AdcNau7802::Init(const ADC_CFG &Cfg, Timer * const pTimer, DeviceIntrf * const pIntrf)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(NAU7802_I2C_ADDR);

	Reset();

	msDelay(200);

	uint8_t regaddr = NAU7802_PU_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~NAU7802_PU_CTRL_AVDDS_LDO;

	Write8(&regaddr, 1, d | NAU7802_PU_CTRL_PUD_PWRUPD | NAU7802_PU_CTRL_PUA_PWRUPA | NAU7802_PU_CTRL_AVDDS_LDO);

	do {
		d = Read8(&regaddr, 1);
	} while ((d & NAU7802_PU_CTRL_PUR_PWRUP_RDY) == 0);

	regaddr = NAU7802_DEV_REV_REG;
	d = Read8(&regaddr, 1);

	if (d != NAU7802_DEV_REV_ID)
	{
		return false;
	}

	vChanOpened = 0;

	Rate(Cfg.Rate);
	Mode(Cfg.Mode);
	Resolution(Cfg.Resolution);

	SetRefVoltage(Cfg.pRefVolt, Cfg.NbRefVolt);

	vpTimer = pTimer;

	if (Cfg.EvtHandler)
	{
		SetEvtHandler(Cfg.EvtHandler);
	}

	vbInterrupt = Cfg.bInterrupt;
	vIntPrio = Cfg.IntPrio;

	return true;
}

bool AdcNau7802::Enable()
{
	uint8_t regaddr = NAU7802_PU_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) | NAU7802_PU_CTRL_PUD_PWRUPD | NAU7802_PU_CTRL_PUA_PWRUPA;

	Write8(&regaddr, 1, d);

	return true;
}

void AdcNau7802::Disable()
{
	uint8_t regaddr = NAU7802_PU_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(NAU7802_PU_CTRL_PUD_PWRUPD | NAU7802_PU_CTRL_PUA_PWRUPA);

	Write8(&regaddr, 1, d);
}

void AdcNau7802::Reset()
{
	uint8_t regaddr = NAU7802_PU_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~NAU7802_PU_CTRL_RR_RESET;

	Write8(&regaddr, 1, d | NAU7802_PU_CTRL_RR_RESET);
	Write8(&regaddr, 1, d);
}

uint32_t AdcNau7802::Rate(uint32_t Val)
{
	uint8_t regaddr = NAU7802_CTRL2_REG;
	uint8_t d = Read8(&regaddr, 1) & ~NAU7802_CTRL2_CRS_MASK;

	if (Val < 15000)
	{
		d |= NAU7802_CTRL2_CRS_10SPS;
		Val = 10000;
	}
	else if (Val < 30000)
	{
		d |= NAU7802_CTRL2_CRS_20SPS;
		Val = 20000;
	}
	else if (Val < 60000)
	{
		d |= NAU7802_CTRL2_CRS_40SPS;
		Val = 40000;
	}
	else if (Val < 200000)
	{
		d |= NAU7802_CTRL2_CRS_80SPS;
		Val = 80000;
	}
	else
	{
		d |= NAU7802_CTRL2_CRS_320SPS;
		Val = 320000;
	}

	return AdcDevice::Rate(Val);
}

uint16_t AdcNau7802::Resolution(uint16_t Val)
{
	return AdcDevice::Resolution(24);
}

bool AdcNau7802::OpenChannel(const ADC_CHAN_CFG * const pChanCfg, int NbChan)
{
	if (pChanCfg == NULL || NbChan == 0)
		return false;

	if (pChanCfg->RefVoltIdx > 1)
		return false;

	uint8_t regaddr = NAU7802_CTRL1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(NAU7802_CTRL1_GAINS_MASK | NAU7802_CTRL1_VLDO_MASK);

	int l = min(NbChan, 2);	// Max 2 chan

	for (int i = 0; i < l; i++)
	{
		vChanOpened |= 1 << pChanCfg->Chan;
	}

	if (pChanCfg->Gain < 2)
	{
		d |= NAU7802_CTRL1_GAINS_X1;
	}
	else if (pChanCfg->Gain < 3)
	{
		d |= NAU7802_CTRL1_GAINS_X2;
	}
	else if (pChanCfg->Gain < 3)
	{
		d |= NAU7802_CTRL1_GAINS_X4;
	}
	else if (pChanCfg->Gain < 6)
	{
		d |= NAU7802_CTRL1_GAINS_X8;
	}
	else if (pChanCfg->Gain < 12)
	{
		d |= NAU7802_CTRL1_GAINS_X16;
	}
	else if (pChanCfg->Gain < 48)
	{
		d |= NAU7802_CTRL1_GAINS_X32;
	}
	else if (pChanCfg->Gain < 96)
	{
		d |= NAU7802_CTRL1_GAINS_X64;
	}
	else
	{
		d |= NAU7802_CTRL1_GAINS_X128;
	}

	if (vpRefVolt[pChanCfg->RefVoltIdx].Type == ADC_REFVOLT_TYPE_INTERNAL)
	{
		if (vpRefVolt[pChanCfg->RefVoltIdx].Voltage < 2.55)
		{
			d |= NAU7802_CTRL1_VLDO_2_4;
		}
		else if (vpRefVolt[pChanCfg->RefVoltIdx].Voltage < 2.85)
		{
			d |= NAU7802_CTRL1_VLDO_2_7;
		}
		else if (vpRefVolt[pChanCfg->RefVoltIdx].Voltage < 3.15)
		{
			d |= NAU7802_CTRL1_VLDO_3_0;
		}
		else if (vpRefVolt[pChanCfg->RefVoltIdx].Voltage < 3.45)
		{
			d |= NAU7802_CTRL1_VLDO_3_3;
		}
		else if (vpRefVolt[pChanCfg->RefVoltIdx].Voltage < 3.75)
		{
			d |= NAU7802_CTRL1_VLDO_3_6;
		}
		else if (vpRefVolt[pChanCfg->RefVoltIdx].Voltage < 4.05)
		{
			d |= NAU7802_CTRL1_VLDO_3_9;
		}
		else if (vpRefVolt[pChanCfg->RefVoltIdx].Voltage < 3.35)
		{
			d |= NAU7802_CTRL1_VLDO_4_2;
		}
		else
		{
			d |= NAU7802_CTRL1_VLDO_4_5;
		}
	}

	Write8(&regaddr,1, d);

	regaddr = NAU7802_ADC_REG;
	Write8(&regaddr, 1, NAU7802_ADC_REG_CHPS_MASK);

	if (NbChan == 1)
	{
		regaddr = NAU7802_POWER_CONTROL_REG;
		d = Read8(&regaddr, 1) | NAU7802_POWER_CONTROL_PGA_CAP_EN;
		Write8(&regaddr, 1, d);
	}
	Calibrate();

	return true;
}

void AdcNau7802::CloseChannel(int Chan)
{
	uint8_t regaddr = NAU7802_CTRL2_REG;
	uint8_t d = Read8(&regaddr, 1);

	vChanOpened &= ~(1 << Chan);
}

bool AdcNau7802::StartConversion()
{
	uint8_t regaddr = NAU7802_PU_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) | NAU7802_PU_CTRL_CS;

	Write8(&regaddr, 1, d);

	return true;
}

void AdcNau7802::StopConversion()
{
	uint8_t regaddr = NAU7802_PU_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1) & ~NAU7802_PU_CTRL_CS;

	Write8(&regaddr, 1, d);
}

int AdcNau7802::Read(ADC_DATA *pBuff, int Len)
{
	return 0;
}

bool AdcNau7802::Read(int Chan, ADC_DATA *pBuff)
{
	pBuff->Chan = 0;
	pBuff->Data = (float)vAdcVal[0];

	return true;
}

bool AdcNau7802::Calibrate()
{
	uint8_t regaddr = NAU7802_I2C_CONTROL_REG;
	uint8_t ictrl = Read8(&regaddr, 1) & ~NAU7802_I2C_CONTROL_SI;
	bool retval = true;

	// Offset calibration by shorting input
	Write8(&regaddr, 1, ictrl | NAU7802_I2C_CONTROL_SI);

	regaddr = NAU7802_CTRL2_REG;
	uint8_t d = Read8(&regaddr, 1) & ~(NAU7802_CTRL2_CALMOD_MASK | NAU7802_CTRL2_CHS_2);

	// Calibrate chan 1
	Write8(&regaddr, 1, d | NAU7802_CTRL2_CALMOD_INTERNAL | NAU7802_CTRL2_CALS_TRIG_CAL);

	do {
		d = Read8(&regaddr, 1);
		if (d & NAU7802_CTRL2_CAL_ERR)
		{
			retval = false;
			break;
		}
	} while (d & NAU7802_CTRL2_CALS_TRIG_CAL);

	// Remove shorts
	regaddr = NAU7802_I2C_CONTROL_REG;
	Write8(&regaddr, 1, ictrl);

	return retval;
}

bool AdcNau7802::UpdateData()
{
	uint8_t regaddr = NAU7802_PU_CTRL_REG;
	uint8_t d = Read8(&regaddr, 1);

	if (d & NAU7802_PU_CTRL_CR)
	{
		regaddr = NAU7802_CTRL2_REG;
		d = (Read8(&regaddr, 1) & NAU7802_CTRL2_CHS_2) >> NAU7802_CTRL2_CHS_BITPOS;

		regaddr = NAU7802_ADCO_B2_REG;
		int32_t x = 0;

		Device::Read(&regaddr, 1, (uint8_t*)&x, 3);

		vAdcVal[d] = (int32_t)EndianCvt32(x) >> 8;

		//StartConversion();
		regaddr = NAU7802_PU_CTRL_REG;
		d = Read8(&regaddr, 1);

		//printf("%x : ", d & NAU7802_PU_CTRL_CR);

		EvtHandler(DEV_EVT_DATA_RDY);

		return true;
	}

	return false;
}

