/**-------------------------------------------------------------------------
@file	pwrmgnt_as3701.cpp

@brief	Power management implementation of the AS3701


@author	Hoang Nguyen Hoan
@date	July 25, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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
#include "istddef.h"
#include "pwrmgnt/pwrmgnt_as3701.h"

bool PowerMgntAS3701::Init(const PWRCFG &Cfg, DeviceIntrf * const pIntrf)
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

int32_t PowerMgntAS3701::SetVout(size_t VoutIdx, int32_t mVolt, uint32_t mALimit)
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
bool PowerMgntAS3701::Enable()
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
void PowerMgntAS3701::Disable()
{

}

/**
 * @brief	Reset device to it initial default state
 */
void PowerMgntAS3701::Reset()
{

}

void PowerMgntAS3701::PowerOff()
{
	uint8_t regaddr = AS3701_SD_CTRL1_REG;
	uint8_t d = Read8(&regaddr, 1) & ~AS3701_SD_CTRL1_SD1_ENBABLE;

	Write8(&regaddr, 1, d);
}

uint32_t PowerMgntAS3701::SetCharge(PWR_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr)
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

	if (Type == PWR_CHARGE_TYPE_TRICKLE)
	{
		d = (mACurr - 11) / 12;
	}
	else
	{
		d = (mACurr - 88);
	}

	Write8(&regaddr, 1, d);

	regaddr = AS3701_CHARGER_CTRL_REG;
	d = Read8(&regaddr, 1) | AS3701_CHARGER_CTRL_USB_CHGEN | AS3701_CHARGER_CTRL_BAT_CHARGING_ENABLE;
	Write8(&regaddr, 1, d);

	return true;
}

/**
 * Turns all LED 100% on
 */
void PowerMgntAS3701::On()
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
void PowerMgntAS3701::Off()
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
void PowerMgntAS3701::Toggle()
{

}

void PowerMgntAS3701::Level(uint32_t Level)
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

bool PowerMgntAS3701::Charging()
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

void PowerMgntAS3701::IrqHandler()
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
