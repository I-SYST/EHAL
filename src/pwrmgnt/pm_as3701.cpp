/**-------------------------------------------------------------------------
@file	pm_as3701.cpp

@brief	Power management implementation of the AS3701


@author	Hoang Nguyen Hoan
@date	July 25, 2019

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
#include "istddef.h"
#include "pwrmgnt/pm_as3701.h"

bool PmAs3701::Init(const PWRCFG &Cfg, DeviceIntrf * const pIntrf)
{
	if (pIntrf == NULL)
	{
		return false;
	}

	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);

	uint8_t regaddr = AS3701_ASIC_ID1_REG;
	uint16_t d = Read16(&regaddr, 1);

	if (d != AS3701_DEVID_16BITS )
	{
		return false;
	}

	if (Cfg.pVout != NULL && Cfg.NbVout > 0)
	{
		int l = min(Cfg.NbVout, AS3701_VOUT_MAXCNT);

		for (int i = 0; i < l; i ++)
		{
			SetVout(i, Cfg.pVout[i].mVout, Cfg.pVout[i].mAlimit);
		}
	}

	SetCharge(PWR_CHARGE_TYPE_AUTO, Cfg.VEndChrg, Cfg.ChrgCurr);

	if (Cfg.pBatProf)
	{
		regaddr = AS3701_CHARGER_SUPERVISION_REG;
		d = 0;

		if (Cfg.pBatProf->ThermBetaConst < 3500)
		{
			d |= AS3701_CHARGER_SUPERVISION_NTC_BETA_3000;
		}
		else if (Cfg.pBatProf->ThermBetaConst < 4000)
		{
			d |= AS3701_CHARGER_SUPERVISION_NTC_BETA_3500;
		}
		else if (Cfg.pBatProf->ThermBetaConst < 4500)
		{
			d |= AS3701_CHARGER_SUPERVISION_NTC_BETA_4000;
		}
		else if (Cfg.pBatProf->ThermBetaConst < 4000)
		{
			d |= AS3701_CHARGER_SUPERVISION_NTC_BETA_3500;
		}
		else
		{
			d |= AS3701_CHARGER_SUPERVISION_NTC_BETA_4500;
		}

		if (Cfg.pBatProf->ThermResistor < 100)
		{
			d |= AS3701_CHARGER_SUPERVISION_NTC_10K;
		}

		Write8(&regaddr, 1, d | AS3701_CHARGER_SUPERVISION_NTC_HIGH_ON | AS3701_CHARGER_SUPERVISION_NTC_HIGH_ON |
			   AS3701_CHARGER_SUPERVISION_NTC_INPUT_NONE);
	}

	vNbLed = min(Cfg.NbLed, AS3701_LED_MAXCNT);

	for (int i = 0; i < vNbLed; i++)
	{
		if (Cfg.pLed[i].Pin >= 0)
		{
			regaddr = AS3701_GPIO1_CTRL_REG + Cfg.pLed[i].Pin;
			Write8(&regaddr, 1, AS3701_GPIO_MODE_OUTPUT);
			vLed[i] = Cfg.pLed[i];
		}
	}

	regaddr = AS3701_REF_CTRL_REG;
	d = Read8(&regaddr, 1) & ~(AS3701_REF_CTRL_LPRESS_DELAY_8 | AS3701_REF_CTRL_TAST_SW_SWITCH);
	if (Cfg.OffSwHold < 8)
	{
		d |= AS3701_REF_CTRL_LPRESS_DELAY_4;
	}

	Write8(&regaddr, 1, d);

	if (Cfg.bIntEn)
	{
		regaddr = AS3701_INTERRUPT_MASK1_REG;
		Write8(&regaddr, 1, AS3701_INTERRUPT_MASK1_EOC | AS3701_INTERRUPT_MASK1_CHDET |
				AS3701_INTERRUPT_MASK1_ONKEY | AS3701_INTERRUPT_MASK1_OVTMP | AS3701_INTERRUPT_MASK1_LOWBAT);

		regaddr = AS3701_INTERRUPT_MASK2_REG;
		Write8(&regaddr, 1, AS3701_INTERRUPT_MASK2_BAT_TEMP);
	}

	return true;
}

int32_t PmAs3701::SetVout(size_t VoutIdx, int32_t mVolt, uint32_t mALimit)
{
	int v = 0;

	switch (VoutIdx)
	{
		case 0:
			{
				uint32_t vsel = 0;
				if (mVolt > 0)
				{
					vsel = (mVolt * 10 - 6000) / 125;
					v = (6000 + vsel * 125) / 10;
					if (vsel > 0x40)
					{
						vsel = ((mVolt - 1400) / 25) & 0xFF;
						v = 1400 + vsel * 25;
						vsel += 0x41;
						if (vsel > 0x70)
						{
							vsel = (mVolt - 2600) / 50;
							v = 2600 + vsel * 50;
							vsel += 0x71;
						}
					}
				}
				uint8_t regaddr = AS3701_SD1_VOLTAGE_REG;
				Write8(&regaddr, 1, vsel & 0xFF);
			}
			break;
		case 1:
			{
				uint32_t vsel = 0;
				vsel = (mVolt - 1200) / 50;
				if (mVolt > 0 && vsel < 0x2B)
				{
					v = 1200 + vsel * 50;
					vsel |= AS3701_LDO1_VOLTAGE_ON;
					if (mALimit > 100 || mALimit == 0)
					{
						vsel |= AS3701_LDO1_VOLTAGE_ILIMIT_200;
					}
				}
				uint8_t regaddr = AS3701_LDO1_VOLTAGE_REG;
				Write8(&regaddr, 1, vsel & 0xFF);
			}
			break;
		case 2:
			{
				uint32_t vsel = 0;
				vsel = (mVolt - 1200) / 50;
				if (mVolt > 0 && vsel < 0x2B)
				{
					v = 1200 + vsel * 50;
					vsel |= AS3701_LDO2_VOLTAGE_ON;
					if (mALimit > 100 || mALimit == 0)
					{
						vsel |= AS3701_LDO2_VOLTAGE_ILIMIT_200;
					}
				}
				uint8_t regaddr = AS3701_LDO2_VOLTAGE_REG;
				Write8(&regaddr, 1, vsel & 0xFF);
			}
			break;
		default:
			;
	}

	return v;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool PmAs3701::Enable()
{
	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void PmAs3701::Disable()
{

}

/**
 * @brief	Reset device to it initial default state
 */
void PmAs3701::Reset()
{

}

void PmAs3701::PowerOff()
{
	uint8_t regaddr = AS3701_SD_CTRL1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~AS3701_SD_CTRL1_SD1_ENBABLE;

	Write8(&regaddr, 1, d);
}

uint32_t PmAs3701::SetCharge(PWR_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr)
{
	uint8_t regaddr = AS3701_CHARGER_VOLTAGE_CTRL_REG;
	uint8_t d = 0;

	if (mVoltEoC > 3820)
	{
		d = (mVoltEoC - 3820) / 20;
	}

	if (mVoltEoC < 3900)
	{
		d |= AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_3900;
	}
	else if (mVoltEoC < 4200)
	{
		d |= AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_4200;
	}
	else if (mVoltEoC < 4500)
	{
		d |= AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_4500;
	}
	else
	{
		d |= AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_4700;
	}

	Write8(&regaddr, 1, d);

	regaddr = AS3701_CHARGER_CURRENT_CTRL_REG;

	d = ((mACurr - 11) / 12) << 4;	// Trickle current
	d |= ((mACurr - 44) / 45) & 0xFF;

	Write8(&regaddr, 1, d);

	regaddr = AS3701_CHARGER_CTRL_REG;
	d = Read8(&regaddr, 1) & ~AS3701_CHARGER_CTRL_USB_CURRENT_MASK;
	d |= AS3701_CHARGER_CTRL_USB_CHGEN | AS3701_CHARGER_CTRL_BAT_CHARGING_ENABLE |
		 AS3701_CHARGER_CTRL_AUTO_RESUME | 8;
	Write8(&regaddr, 1, d);

	return true;
}

/**
 * Turns all LED 100% on
 */
void PmAs3701::On()
{
	uint8_t regaddr = AS3701_GPIO_SIGNAL_OUT_REG;//AS3701_PWM_CONTROL_LOW_REG;//AS3701_CURR1_VALUE_REG;
	uint8_t mask = 0;
	uint8_t d =	Read8(&regaddr, 1);

	for (int i = 0; i < vNbLed; i++)
	{
		if (vLed[i].Act)
		{
			d |= (1 << vLed[i].Pin);
		}
		else
		{
			d &= ~(1 << vLed[i].Pin);
		}
	}
	Write8(&regaddr, 1, d);
}

/**
 * Turns all LED off
 */
void PmAs3701::Off()
{
	uint8_t regaddr = AS3701_GPIO_SIGNAL_OUT_REG;//AS3701_PWM_CONTROL_LOW_REG;//AS3701_CURR1_VALUE_REG;
	uint8_t mask = 0;
	uint8_t d =	Read8(&regaddr, 1);

	for (int i = 0; i < vNbLed; i++)
	{
		if (vLed[i].Act)
		{
			d &= ~(1 << vLed[i].Pin);
		}
		else
		{
			d |= (1 << vLed[i].Pin);
		}
	}
	Write8(&regaddr, 1, d);
}

/**
 * Toggle or invert all LED dimming level
 */
void PmAs3701::Toggle()
{

}

void PmAs3701::Level(uint32_t Level)
{
	uint8_t regaddr = AS3701_GPIO_SIGNAL_OUT_REG;
	uint32_t mask = 0xff;
	uint8_t d =	Read8(&regaddr, 1);

	for (int i = 0; i < vNbLed; i++)
	{
		if (Level & mask)
		{
			if (vLed[i].Act)
			{
				d |= (1 << vLed[i].Pin);
			}
			else
			{
				d &= ~(1 << vLed[i].Pin);
			}
		}
		else
		{
			if (vLed[i].Act)
			{
				d &= ~(1 << vLed[i].Pin);
			}
			else
			{
				d |= (1 << vLed[i].Pin);
			}
		}
		mask <<= 8;
	}
	Write8(&regaddr, 1, d);
}

bool PmAs3701::Charging()
{
	uint8_t regaddr = AS3701_CHARGER_STATUS1_REG;
	uint8_t flag = Read8(&regaddr, 1);

	regaddr = AS3701_CHARGER_STATUS2_REG;
	uint8_t flag2 = Read8(&regaddr, 1);

	if (flag2 & AS3701_CHARGER_STATUS2_CHDET)
	{
		if (flag & AS3701_CHARGER_STATUS1_EOC)
		{
			return false;
		}

		return true;
	}
	return false;
}

bool PmAs3701::Battery()
{
	uint8_t regaddr = AS3701_CHARGER_STATUS1_REG;
	uint8_t flag = Read8(&regaddr, 1);

	return !(flag & AS3701_CHARGER_STATUS1_NO_BAT);
}

void PmAs3701::IrqHandler()
{
	uint8_t regaddr = AS3701_INTERRUPT_STATUS1_REG;
	uint8_t flag = Read8(&regaddr, 1);

	regaddr = AS3701_INTERRUPT_STATUS2_REG;
	uint8_t flag1 = Read8(&regaddr, 1);

	if (vpEvtHandler)
	{
		if (flag & AS3701_INTERRUPT_STATUS1_EOC_INT)
		{
			vpEvtHandler(this, PWREVT_CHARGE_FULL);
		}
		if (flag & AS3701_INTERRUPT_STATUS1_OVTMP_INT)
		{
			vpEvtHandler(this, PWREVT_OVER_HEAT);
		}
		if (flag & AS3701_INTERRUPT_STATUS1_LOWBAT_INT)
		{
			vpEvtHandler(this, PWREVT_LOW_BAT);
		}
		if (flag & AS3701_INTERRUPT_STATUS1_CHDET_INT)
		{
			vpEvtHandler(this, PWREVT_CHARGE_DETECTED);
		}
	}
}
