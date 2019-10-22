/**-------------------------------------------------------------------------
@file	pm_as3701.h

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
#ifndef __PM_AS3701_H__
#define __PM_AS3701_H__

#include <stdint.h>

#include "pwrmgnt/pwrmgnt.h"
#include "miscdev/led.h"

/** @addtogroup Power
  * @{
  */

#define AS3701_I2C_7BITS_DEV_ADDR				0x40

#define AS3701_DEVID_16BITS						0x11

#define AS3701_SD1_VOLTAGE_REG			1

#define AS3701_SD1_VOLTAGE_SD1_VSEL_MASK					(0x7F)
#define AS3701_SD1_VOLTAGE_SD1_VSEL_PWRDOWN					(0)

#define AS3701_SD1_VOLTAGE_SD1_FREQ_MASK                	(1<<8)
#define AS3701_SD1_VOLTAGE_SD1_FREQ_LOW						(0<<8)
#define AS3701_SD1_VOLTAGE_SD1_FREQ_HIGH					(1<<8)

#define AS3701_LDO1_VOLTAGE_REG			2

#define AS3701_LDO1_VOLTAGE_VSEL_MASK          				(0x3F)

#define AS3701_LDO1_VOLTAGE_ILIMIT_100         				(0<<6)
#define AS3701_LDO1_VOLTAGE_ILIMIT_200         				(1<<6)

#define AS3701_LDO1_VOLTAGE_OFF								(0<<7)
#define AS3701_LDO1_VOLTAGE_ON								(1<<7)

#define AS3701_LDO2_VOLTAGE_REG			3

#define AS3701_LDO2_VOLTAGE_VSEL_MASK          				(0x3F)
#define AS3701_LDO2_VOLTAGE_ILIMIT_100         				(0<<6)
#define AS3701_LDO2_VOLTAGE_ILIMIT_200         				(1<<6)
#define AS3701_LDO2_VOLTAGE_OFF		         				(0<<7)
#define AS3701_LDO2_VOLTAGE_ON                 				(1<<7)

#define AS3701_GPIO1_CTRL_REG			9
#define AS3701_GPIO2_CTRL_REG			0xA
#define AS3701_GPIO3_CTRL_REG			0xB
#define AS3701_GPIO4_CTRL_REG			0xC
#define AS3701_GPIO5_CTRL_REG			0xD

#define AS3701_GPIO_IOSF_MASK              				(0xF)
#define AS3701_GPIO_IOSF_NORMAL            				(0x0<<0)
#define AS3701_GPIO_IOSF_INTERRUPT_OUT     				(0x1<<0)
#define AS3701_GPIO_IOSF_VSUP_LOW          				(0x2<<0)
#define AS3701_GPIO_IOSF_INTERRUPT_IN      				(0x3<<0)
#define AS3701_GPIO_IOSF_SINK              				(0x4<<0)
#define AS3701_GPIO_IOSF_VSELECT           				(0x5<<0)
#define AS3701_GPIO_IOSF_STANDBY           				(0x6<<0)
#define AS3701_GPIO_IOSF_POWER_GOOD        				(0x7<<0)
#define AS3701_GPIO_IOSF_CURRENT1          				(0x8<<0)
#define AS3701_GPIO_IOSF_CURRENT2          				(0x9<<0)
#define AS3701_GPIO_IOSF_CHARGER_OUT       				(0xA<<0)
#define AS3701_GPIO_IOSF_EOC               				(0xB<<0)
#define AS3701_GPIO_IOSF_CURRENT3          				(0xC<<0)
#define AS3701_GPIO_IOSF_CURRENT4          				(0xD<<0)
#define AS3701_GPIO_IOSF_PWM               				(0xE<<0)
#define AS3701_GPIO_IOSF_CHARGER_IN        				(0xF<<0)

#define AS3701_GPIO_MODE_MASK              				(7<<4)
#define AS3701_GPIO_MODE_INPUT             				(0<<4)
#define AS3701_GPIO_MODE_OUTPUT            				(1<<4)
#define AS3701_GPIO_MODE_IO1               				(2<<4)
#define AS3701_GPIO_MODE_INPUT_TRI         				(3<<4)
#define AS3701_GPIO_MODE_INPUT_UP          				(4<<4)
#define AS3701_GPIO_MODE_INPUT_DOWN        				(5<<4)
#define AS3701_GPIO_MODE_IO2               				(6<<4)
#define AS3701_GPIO_MODE_CURR1             				(7<<4)

#define AS3701_GPIO_INVERT_NORMAL          				(0<<7)
#define AS3701_GPIO_INVERT_INVERT          				(1<<7)


#define AS3701_GPIO_SIGNAL_OUT_REG		0x20

#define AS3701_GPIO_SIGNAL_OUT_1            				(1<<0)
#define AS3701_GPIO_SIGNAL_OUT_2            				(1<<1)
#define AS3701_GPIO_SIGNAL_OUT_3            				(1<<2)
#define AS3701_GPIO_SIGNAL_OUT_4            				(1<<3)
#define AS3701_GPIO_SIGNAL_OUT_5            				(1<<4)

#define AS3701_GPIO_SIGNAL_IN_REG		0x21

#define AS3701_REG1_VOLTAGE_REG			0x22

#define AS3701_REG2_VOLTAGE_REG			0x23

#define AS3701_REG_CTRL_REG				0x24

#define AS3701_REG_CTRL_REG1_SELECT_MASK					(3)
#define AS3701_REG_CTRL_REG1_SELECT_SD1						(1<<0)
#define AS3701_REG_CTRL_REG1_SELECT_LDO1					(2<<0)
#define AS3701_REG_CTRL_REG1_SELECT_LDO2					(3<<0)

#define AS3701_REG_CTRL_REG2_SELECT_MASK					(3<<4)
#define AS3701_REG_CTRL_REG2_SELECT_SD1						(1<<4)
#define AS3701_REG_CTRL_REG2_SELECT_LDO1					(2<<4)
#define AS3701_REG_CTRL_REG2_SELECT_LDO2					(3<<4)

#define AS3701_GPIO_CTRL1_REG			0x25

#define AS3701_GPIO_CTRL1_LDO1_MASK							(7<<0)
#define AS3701_GPIO_CTRL1_LDO2_MASK							(7<<4)

#define AS3701_GPIO_CTRL2_REG			0x26

#define AS3701_GPIO_CTRL1_SD1_MASK							(7<<0)

#define AS3701_SD_CTRL1_REG				0x30

#define AS3701_SD_CTRL1_DVM_TIME_8US						(0<<0)
#define AS3701_SD_CTRL1_DVM_TIME_16US						(1<<0)
#define AS3701_SD_CTRL1_DVM_ENABLE							(1<<1)
#define AS3701_SD_CTRL1_SD1_FSEL_LOW						(0<<4)
#define AS3701_SD_CTRL1_SD1_FSEL_HIGH						(1<<4)
#define AS3701_SD_CTRL1_SD1_FAST_NORMAL						(0<<5)
#define AS3701_SD_CTRL1_SD1_FAST_FAST						(1<<5)
#define AS3701_SD_CTRL1_SD1_LOW_NOISE_NORMAL				(0<<6)
#define AS3701_SD_CTRL1_SD1_LOW_NOISE_LOW_NOISE				(1<<6)
#define AS3701_SD_CTRL1_SD1_DISABLE							(0<<7)
#define AS3701_SD_CTRL1_SD1_ENBABLE							(1<<7)


#define AS3701_BAT_VOLTAGE_MONITOR_REG	0x32

#define AS3701_BAT_VOLTAGE_RESVOLTRISE_MASK					(7<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_2_7  				(0<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_2_9  				(1<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_3_1  				(2<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_3_2  				(3<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_3_3  				(4<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_3_4  				(5<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_3_5  				(6<<0)
#define AS3701_BAT_VOLTAGE_RESVOLTRISE_3_6  				(7<<0)

#define AS3701_BAT_VOLTAGE_RESVOLTFALL_MASK					(7<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_2_7  				(0<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_2_9  				(1<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_3_1  				(2<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_3_2  				(3<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_3_3  				(4<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_3_4  				(5<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_3_5  				(6<<3)
#define AS3701_BAT_VOLTAGE_RESVOLTFALL_3_6  				(7<<3)

#define AS3701_BAT_VOLTAGE_SUPRESEN_2_7						(0<<6)
#define AS3701_BAT_VOLTAGE_SUPRESEN_RESVOLTFALL				(1<<6)

#define AS3701_BAT_VOLTAGE_FASTRESEN_3MS					(0<<7)
#define AS3701_BAT_VOLTAGE_FASTRESEN_64US					(0<<7)

#define AS3701_STARTUP_CTRL_REG				0x33

#define AS3701_STARTUP_CTRL_RES_TIMER_MASK					(3<<0)
#define AS3701_STARTUP_CTRL_RES_TIMER_10					(0<<0)
#define AS3701_STARTUP_CTRL_RES_TIMER_50					(1<<0)
#define AS3701_STARTUP_CTRL_RES_TIMER_100					(2<<0)
#define AS3701_STARTUP_CTRL_RES_TIMER_150					(3<<0)

#define AS3701_STARTUP_CTRL_OFF_DELAY_MASK					(3<<2)
#define AS3701_STARTUP_CTRL_OFF_DELAY_0						(0<<2)
#define AS3701_STARTUP_CTRL_OFF_DELAY_8						(1<<2)
#define AS3701_STARTUP_CTRL_OFF_DELAY_16					(2<<2)
#define AS3701_STARTUP_CTRL_OFF_DELAY_32					(3<<2)

#define AS3701_STARTUP_CTRL_AUTO_OFF_STARTUP				(0<<4)
#define AS3701_STARTUP_CTRL_AUTO_OFF_PWROFF					(1<<4)

#define AS3701_STARTUP_CTRL_STBY_RESET_NORMAL				(0<<5)
#define AS3701_STARTUP_CTRL_STBY_RESET_DISABLE				(1<<5)

#define AS3701_STARTUP_CTRL_POWER_OFF_AT_VSUPLOW_STARTUP	(0<<6)
#define AS3701_STARTUP_CTRL_POWER_OFF_AT_VSUPLOW_PWROFF		(1<<6)

#define AS3701_STARTUP_CTRL_CHG_PWR_OFF_EN_LEVEL			(0<<7)
#define AS3701_STARTUP_CTRL_CHG_PWR_OFF_EN_EDGE				(1<<7)

#define AS3701_REF_CTRL_REG					0x35

#define AS3701_REF_CTRL_PWM_DIV_MASK						(3<<0)
#define AS3701_REF_CTRL_PWM_DIV_1           				(0<<0)
#define AS3701_REF_CTRL_PWM_DIV_2           				(1<<0)
#define AS3701_REF_CTRL_PWM_DIV_4           				(2<<0)
#define AS3701_REF_CTRL_PWM_DIV_16          				(3<<0)

#define AS3701_REF_CTRL_CLK_MASK							(3<<2)
#define AS3701_REF_CTRL_CLK_4_0             				(0<<2)
#define AS3701_REF_CTRL_CLK_3_8             				(1<<2)
#define AS3701_REF_CTRL_CLK_3_6             				(2<<2)
#define AS3701_REF_CTRL_CLK_3_4             				(3<<2)

#define AS3701_REF_CTRL_STANDBY_MODE_ON						(1<<4)
#define AS3701_REF_CTRL_LPRESS_RESET						(1<<5)
#define AS3701_REF_CTRL_TAST_SW_PUSHBUT						(0<<6)
#define AS3701_REF_CTRL_TAST_SW_SWITCH						(1<<6)
#define AS3701_REF_CTRL_LPRESS_DELAY_4						(1<<7)
#define AS3701_REF_CTRL_LPRESS_DELAY_8						(0<<7)

#define AS3701_RESET_CTRL_REG               0x36

#define AS3701_RESET_CTRL_FORCE_RESET       				(1<<0)
#define AS3701_RESET_CTRL_POWER_OFF         				(1<<1)
#define AS3701_RESET_CTRL_ON_INPUT          				(1<<2)
#define AS3701_RESET_CTRL_ONKEY_ENABLE      				(1<<3)
#define AS3701_RESET_CTRL_RESET_REASON_MASK					(0xF<<4)

#define AS3701_OVR_TEMP_CTRL_REG			0x37

#define AS3701_OVR_TEMP_CTRL_TEMP_PMC_ON					(1<<0)
#define AS3701_OVR_TEMP_CTRL_OV_TEMP_110					(1<<1)
#define AS3701_OVR_TEMP_CTRL_OV_TEMP_140					(1<<2)
#define AS3701_OVR_TEMP_CTRL_RST_OV_TEMP_140				(1<<3)
#define AS3701_OVR_TEMP_CTRL_TEMP_TEST_MASK					(3<<4)
#define AS3701_OVR_TEMP_CTRL_TCO_110_A						(1<<6)
#define AS3701_OVR_TEMP_CTRL_TCO_140_A						(1<<7)

#define AS3701_REG_STANDBY_MOD1_REG			0x39

#define AS3701_REG_STANDBY_MOD1_LDO1_STBY_ON				(1<<0)
#define AS3701_REG_STANDBY_MOD1_LDO2_STBY_ON				(1<<1)
#define AS3701_REG_STANDBY_MOD1_SD1_STBY_ON					(1<<2)
#define AS3701_REG_STANDBY_MOD1_DISABLE_REGPD				(1<<3)

#define AS3701_PWM_CONTROL_LOW_REG          0x41

#define AS3701_PWM_CONTROL_HIGH_REG         0x42

#define AS3701_CURR1_VALUE_REG				0x43

#define AS3701_CURR2_VALUE_REG				0x44

#define AS3701_REG_STATUS_REG				0x73

#define AS3701_REG_STATUS_SD1_LV							(1<<0)
#define AS3701_REG_STATUS_CURR1_LV							(1<<5)
#define AS3701_REG_STATUS_CURR2_LV							(1<<6)

#define AS3701_INTERRUPT_MASK1_REG			0x74

#define AS3701_INTERRUPT_MASK1_TRICKLE						(1<<0)
#define AS3701_INTERRUPT_MASK1_NOBAT						(1<<1)
#define AS3701_INTERRUPT_MASK1_RESUME						(1<<2)
#define AS3701_INTERRUPT_MASK1_EOC							(1<<3)
#define AS3701_INTERRUPT_MASK1_CHDET						(1<<4)
#define AS3701_INTERRUPT_MASK1_ONKEY						(1<<5)
#define AS3701_INTERRUPT_MASK1_OVTMP						(1<<6)
#define AS3701_INTERRUPT_MASK1_LOWBAT						(1<<7)

#define AS3701_INTERRUPT_MASK2_REG			0x75

#define AS3701_INTERRUPT_MASK2_BAT_TEMP						(1<<0)
#define AS3701_INTERRUPT_MASK2_SD1_LV						(1<<1)
#define AS3701_INTERRUPT_MASK2_GPIO_RESTART					(1<<2)
#define AS3701_INTERRUPT_MASK2_GPIO1						(1<<3)
#define AS3701_INTERRUPT_MASK2_GPIO2						(1<<4)
#define AS3701_INTERRUPT_MASK2_GPIO3						(1<<5)
#define AS3701_INTERRUPT_MASK2_GPIO4						(1<<6)
#define AS3701_INTERRUPT_MASK2_GPIO5						(1<<7)

#define AS3701_INTERRUPT_STATUS1_REG		0x77

#define AS3701_INTERRUPT_STATUS1_TRICKLE_INT				(1<<0)
#define AS3701_INTERRUPT_STATUS1_NOBAT_INT					(1<<1)
#define AS3701_INTERRUPT_STATUS1_RESUME_IN					(1<<2)
#define AS3701_INTERRUPT_STATUS1_EOC_INT					(1<<3)
#define AS3701_INTERRUPT_STATUS1_CHDET_INT					(1<<4)
#define AS3701_INTERRUPT_STATUS1_ONKEY_INT					(1<<5)
#define AS3701_INTERRUPT_STATUS1_OVTMP_INT					(1<<6)
#define AS3701_INTERRUPT_STATUS1_LOWBAT_INT					(1<<7)

#define AS3701_INTERRUPT_STATUS2_REG		0x78

#define AS3701_INTERRUPT_STATUS2_BAT_TEMP_INT				(1<<0)
#define AS3701_INTERRUPT_STATUS2_SD1_LV_INT					(1<<1)
#define AS3701_INTERRUPT_STATUS2_GPIO_RESTART_INT			(1<<2)
#define AS3701_INTERRUPT_STATUS2_GPIO1_INT					(1<<3)
#define AS3701_INTERRUPT_STATUS2_GPIO2_INT					(1<<4)
#define AS3701_INTERRUPT_STATUS2_GPIO3_INT					(1<<5)
#define AS3701_INTERRUPT_STATUS2_GPIO4_INT					(1<<6)
#define AS3701_INTERRUPT_STATUS2_GPIO5_INT					(1<<7)

#define AS3701_CHARGER_CTRL_REG             0x80

#define AS3701_CHARGER_CTRL_USB_CURRENT_MASK				(0xF<<0)
#define AS3701_CHARGER_CTRL_USB_CHGEN						(1<<4)
#define AS3701_CHARGER_CTRL_BAT_CHARGING_ENABLE				(1<<5)
#define AS3701_CHARGER_CTRL_AUTO_RESUME     				(1<<6)
#define AS3701_CHARGER_CTRL_CC_RANGE_SELECT_LOW    			(1<<7)

#define AS3701_CHARGER_VOLTAGE_CTRL_REG	0x81

#define AS3701_CHARGER_VOLTAGE_CTRL_CHVOL_EOC_MASK			(0x1F<<0)
#define AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_MASK			(3<<6)
#define AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_3900			(0<<6)
#define AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_4200			(1<<6)
#define AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_4500			(2<<6)
#define AS3701_CHARGER_VOLTAGE_CTRL_VSUP_MIN_4700			(3<<6)

#define AS3701_CHARGER_CURRENT_CTRL_REG		0x82

#define AS3701_CHARGER_CURRENT_CTRL_CONST_CURR_MASK			(0xF<<0)
#define AS3701_CHARGER_CURRENT_CTRL_TRICKLE_CURR_MASK		(0xF<<4)

#define AS3701_CHARGER_CFG1_REG             0x83

#define AS3701_CHARGER_CFG1_VSUP_VOLTAGE_MASK				(7<<1)
#define AS3701_CHARGER_CFG1_TEMP_SEL_MASK					(3<<4)
#define AS3701_CHARGER_CFG1_CHVOLT_RESUME					(1<<6)
#define AS3701_CHARGER_CFG1_CHARGING_1HZ_CLK				(1<<7)

#define AS3701_CHARGER_CFG2_REG             0x84

#define AS3701_CHARGER_CFG2_CH_TIMEOUT_MASK					(0xF<0)
#define AS3701_CHARGER_CFG2_CHARGIN_TMAX					(1<<4)
#define AS3701_CHARGER_CFG2_EOC_CURRENT_MASK				(7<<5)

#define AS3701_CHARGER_SUPERVISION_REG		0x85

#define AS3701_CHARGER_SUPERVISION_NTC_BETA_MASK			(3<<0)
#define AS3701_CHARGER_SUPERVISION_NTC_BETA_3000			(0<<0)
#define AS3701_CHARGER_SUPERVISION_NTC_BETA_3500			(1<<0)
#define AS3701_CHARGER_SUPERVISION_NTC_BETA_4000			(2<<0)
#define AS3701_CHARGER_SUPERVISION_NTC_BETA_4500			(3<<0)
#define AS3701_CHARGER_SUPERVISION_NTC_INPUT_MASK			(3<<2)
#define AS3701_CHARGER_SUPERVISION_NTC_INPUT_NONE			(0<<2)
#define AS3701_CHARGER_SUPERVISION_NTC_INPUT_XIRQ_PIN		(1<<2)
#define AS3701_CHARGER_SUPERVISION_NTC_INPUT_GPIO3_PIN		(2<<2)
#define AS3701_CHARGER_SUPERVISION_NTC_INPUT_GPIO4_PIN		(3<<0)
#define AS3701_CHARGER_SUPERVISION_NTC_MODE_MASK			(1<<4)
#define AS3701_CHARGER_SUPERVISION_NTC_MODE_45				(0<<4)
#define AS3701_CHARGER_SUPERVISION_NTC_MODE_60				(1<<4)
#define AS3701_CHARGER_SUPERVISION_NTC_10K					(1<<5)
#define AS3701_CHARGER_SUPERVISION_NTC_LOW_ON				(1<<6)
#define AS3701_CHARGER_SUPERVISION_NTC_HIGH_ON				(1<<7)

#define AS3701_CHARGER_STATUS1_REG          0x86

#define AS3701_CHARGER_STATUS1_CCM          				(1<<0)
#define AS3701_CHARGER_STATUS1_RESUME       				(1<<1)
#define AS3701_CHARGER_STATUS1_TRICKLE      				(1<<2)
#define AS3701_CHARGER_STATUS1_CVM          				(1<<3)
#define AS3701_CHARGER_STATUS1_EOC							(1<<4)
#define AS3701_CHARGER_STATUS1_TEMP_COND_MASK				(3<<5)
#define AS3701_CHARGER_STATUS1_TEMP_COND_TYPICAL			(0<<5)
#define AS3701_CHARGER_STATUS1_TEMP_COND_COLD				(1<<5)
#define AS3701_CHARGER_STATUS1_TEMP_COND_HOT				(2<<5)
#define AS3701_CHARGER_STATUS1_NO_BAT       				(1<<7)

#define AS3701_CHARGER_STATUS2_REG          0x87

#define AS3701_CHARGER_STATUS2_BATSW_MODE_MASK    			(3<<0)
#define AS3701_CHARGER_STATUS2_CHDET						(1<<2)
#define AS3701_CHARGER_STATUS2_CHDET_OFF					(1<<3)

#define AS3701_LOCK_REGISTER_REG			0x8E

#define AS3701_LOCK_REGISTER_REG_LOCK_MASK					(3<<0)
#define AS3701_LOCK_REGISTER_CHARGER_LOCK					(1<<2)

#define AS3701_ASIC_ID1_REG                 0x90

#define AS3701_ASIC_ID1_VALUE								(0x11)

#define AS3701_ASIC_ID2_REG					0x91

#define AS3701_ASIC_ID2_VALUE								(0)

#define AS3701_VOUT_MAXCNT					3	//!< Max number of output
#define AS3701_LED_MAXCNT					2

class PmAs3701 : public PowerMgnt, public LedDevice {
public:
	bool Init(const PWRCFG &Cfg, DeviceIntrf * const pIntrf);
	int32_t SetVout(size_t VoutIdx, int32_t mVolt, uint32_t CurrLimit);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * This function is used to put the device in lowest power mode
	 * possible so that the Enable function can wake up without full
	 * initialization.
	 */
	void Disable();

	/**
	 * @brief	Reset device to it initial default state
	 */
	void Reset();

	void PowerOff();

	uint32_t SetCharge(PWR_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr);

	bool Charging();
	bool Battery();


	/**
	 * Turns all LED 100% on
	 */
	void On();

	/**
	 * Turns all LED off
	 */
	void Off();

	/**
	 * Toggle or invert all LED dimming level
	 */
	void Toggle();

	/**
	 * @brief	Set LED level
	 *
	 * This function set the dimming level of the LED 0-255.  On multi-color LED can be
	 * used to mix color.  Usually used for PWM analog led
	 *
	 * @param Level	: LED dimming Level 0-255.  0 = Off, 255 = 100% On. Up to 4 LEDs can be dimmed.
	 * 					Bits 0-7  	: LED 0
	 * 					Bits 8-15 	: LED 1
	 * 					Bits 16-23	: LED 2
	 * 					Bits 24-31	: LED 3
	 *
	 */
	void Level(uint32_t Level);

	void IrqHandler();

private:
	LED_DEV vLed[AS3701_LED_MAXCNT];
	int vNbLed;
};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif //__PM_AS3701_H__
