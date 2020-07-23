/**-------------------------------------------------------------------------
@file	adc_nau7802.h

@brief	Nuvoton NAU7802 24-Bit Dual-Channel ADC For Bridge Sensors

This device has 2 channels multiplexed. Same settings for both. In order
to capture both channel.
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
#ifndef __ADC_NAU7802_H__
#define __ADC_NAU7802_H__

#include <stdint.h>

#include "adc_device.h"

/** @addtogroup Converters
  * @{
  */

#define NAU7802_I2C_ADDR					0x2A	// NAU7802 I2C 7 bits slave address

#define NAU7802_DEV_REV_REG					0x1F
#define NAU7802_DEV_REV_ID_MASK						(0xF<<0)
#define NAU7802_DEV_REV_ID							(0xF<<0)

#define NAU7802_PU_CTRL_REG					0x0		//

#define NAU7802_PU_CTRL_RR_RESET					(1<<0)	// Reset
#define NAU7802_PU_CTRL_PUD_PWRUPD					(1<<1)	// Power up digital circuit
#define NAU7802_PU_CTRL_PUA_PWRUPA					(1<<2)	// Power up analog circuit
#define NAU7802_PU_CTRL_PUR_PWRUP_RDY				(1<<3)	// Power up ready
#define NAU7802_PU_CTRL_CS							(1<<4)	// Start conversion
#define NAU7802_PU_CTRL_CR							(1<<5)	// Data ready
#define NAU7802_PU_CTRL_OSCS_EXT					(1<<6)	// Select external crystal
#define NAU7802_PU_CTRL_AVDDS_LDO					(1<<7)	// Select internal LDO

#define NAU7802_CTRL1_REG					0x1		//

#define NAU7802_CTRL1_GAINS_MASK					(7<<0)	// Gain select mask
#define NAU7802_CTRL1_GAINS_X1						(0<<0)	// Gains x1
#define NAU7802_CTRL1_GAINS_X2						(1<<0)	// Gains x2
#define NAU7802_CTRL1_GAINS_X4						(2<<0)	// Gains x4
#define NAU7802_CTRL1_GAINS_X8						(3<<0)	// Gains x8
#define NAU7802_CTRL1_GAINS_X16						(4<<0)	// Gains x16
#define NAU7802_CTRL1_GAINS_X32						(5<<0)	// Gains x32
#define NAU7802_CTRL1_GAINS_X64						(6<<0)	// Gains x64
#define NAU7802_CTRL1_GAINS_X128					(7<<0)	// Gains x128

#define NAU7802_CTRL1_VLDO_MASK						(7<<3)	// LDO voltage select mask
#define NAU7802_CTRL1_VLDO_4_5						(0<<3)	// LDO 4.5V
#define NAU7802_CTRL1_VLDO_4_2						(1<<3)	// LDO 4.2V
#define NAU7802_CTRL1_VLDO_3_9						(2<<3)	// LDO 3.9V
#define NAU7802_CTRL1_VLDO_3_6						(3<<3)	// LDO 3.6V
#define NAU7802_CTRL1_VLDO_3_3						(4<<3)	// LDO 3.3V
#define NAU7802_CTRL1_VLDO_3_0						(5<<3)	// LDO 3.0V
#define NAU7802_CTRL1_VLDO_2_7						(6<<3)	// LDO 2.7V
#define NAU7802_CTRL1_VLDO_2_4						(7<<3)	// LDO 2.4V

#define NAU7802_CTRL1_DRDY_SEL_DRDY					(0<<6)	// Select DRDY pin as data ready flag
#define NAU7802_CTRL1_DRDY_SEL_CLOCK				(1<<6)	// Select DRDY pin as clock output

#define NAU7802_CTRL1_CRP_CRDY_LOW					(1<<7)	// Select CRDY pin active low (avail on 16 pins package only)

#define NAU7802_CTRL2_REG					0x2		//

#define NAU7802_CTRL2_CALMOD_MASK					(3<<0)	// Calibration mode mask
#define NAU7802_CTRL2_CALMOD_INTERNAL				(0<<0)	// Offset calibration internal
#define NAU7802_CTRL2_CALMOD_OFFSET					(2<<0)	// Offset calibration system
#define NAU7802_CTRL2_CALMOD_GAIN					(3<<0)	// Gain calibration system
#define NAU7802_CTRL2_CALS_TRIG_CAL					(1<<2)	// Trigger calibration. This bit will reset to 0 when calibration done
#define NAU7802_CTRL2_CAL_ERR						(1<<3)	// Calibration error
#define NAU7802_CTRL2_CRS_MASK						(7<<4)	// Conversion rate
#define NAU7802_CTRL2_CRS_10SPS						(0<<4)	// 10 samples per second
#define NAU7802_CTRL2_CRS_20SPS						(1<<4)	// 20 samples per second
#define NAU7802_CTRL2_CRS_40SPS						(2<<4)	// 40 samples per second
#define NAU7802_CTRL2_CRS_80SPS						(3<<4)	// 80 samples per second
#define NAU7802_CTRL2_CRS_320SPS					(7<<4)	// 320 samples per second
#define NAU7802_CTRL2_CHS_1							(0<<7)	// Select analog input channel 1
#define NAU7802_CTRL2_CHS_2							(1<<7)	// Select analog input channel 2
#define NAU7802_CTRL2_CHS_BITPOS					(7)

#define NAU7802_OCAL1_B2_REG				0x3		//
#define NAU7802_OCAL1_B1_REG				0x4		//
#define NAU7802_OCAL1_B0_REG				0x5		//

#define NAU7802_GCAL1_B3_REG				0x6		//
#define NAU7802_GCAL1_B2_REG				0x7		//
#define NAU7802_GCAL1_B1_REG				0x8		//
#define NAU7802_GCAL1_B0_REG				0x9		//

#define NAU7802_OCAL2_B2_REG				0xA		//
#define NAU7802_OCAL2_B1_REG				0xB		//
#define NAU7802_OCAL2_B0_REG				0xC		//

#define NAU7802_GCAL2_B3_REG				0xD		//
#define NAU7802_GCAL2_B2_REG				0xE		//
#define NAU7802_GCAL2_B1_REG				0xF		//
#define NAU7802_GCAL2_B0_REG				0x10	//

#define NAU7802_I2C_CONTROL_REG				0x11

#define NAU7802_I2C_CONTROL_BGPCP					(1<<0)	// Disable bandgap chopper
#define NAU7802_I2C_CONTROL_TS						(1<<1)	// Switches PGA input to temperature sensor
#define NAU7802_I2C_CONTROL_BOPGA					(1<<2)	// Enable 2.5uA burnout current source to PGA positive
#define NAU7802_I2C_CONTROL_SI						(1<<3)	// Short input together, measure offset
#define NAU7802_I2C_CONTROL_WPD						(1<<4)	// Disable weak pullup for I2C
#define NAU7802_I2C_CONTROL_SPE						(1<<5)	// Enable strong strong pullup for I2C
#define NAU7802_I2C_CONTROL_FRD						(1<<6)	// Enable fast read, register 0x15 bit 7 must also be 1
#define NAU7802_I2C_CONTROL_CRSD					(1<<7)	// Enable bit for pull SDA low

#define NAU7802_ADCO_B2_REG					0x12
#define NAU7802_ADCO_B1_REG					0x13
#define NAU7802_ADCO_B0_REG					0x14

#define NAU7802_ADC_REG						0x15

#define NAU7802_ADC_REG_CHP_MASK			(3<<0)
#define NAU7802_ADC_ADC_VCM_MASK			(3<<2)
#define NAU7802_ADC_REG_CHPS_MASK			(3<<4)


#define NAU7802_OPT_B1_REG					0x15
#define NAU7802_OPT_B0_REG					0x16

#define NAU7802_PGA_REG						0x1B

#define NAU7802_PGA_PGACHPDIS						(1<<0)	// Chopper disable
#define NAU7802_PGA_PGAINV							(1<<3)	// invert PGA input phase
#define NAU7802_PGA_BYPASS_EN						(1<<4)	// PGA bypass enable
#define NAU7802_PGA_BUFFER_EN						(1<<5)	// PGA output buffer enable
#define NAU7802_PGA_LDOMODE							(1<<6)	// Improve stability and lower DC gain
#define NAU7802_PGA_RD_OTP_SEL						(1<<7)	// Select reg 0x15 reads OTP
#define NAU7802_PGA_RD_ADC_SEL						(0<<7)	// Select reg 0x15 reads ADC

#define NAU7802_POWER_CONTROL_REG			0x1C

#define NAU7802_POWER_CONTROL_PGA_CURR_MASK			(3<<0)	// PGA current mask
#define NAU7802_POWER_CONTROL_PGA_CURR_100			(0<<0)	// 100% of master bias
#define NAU7802_POWER_CONTROL_PGA_CURR_95			(1<<0)	// 95% of master bias
#define NAU7802_POWER_CONTROL_PGA_CURR_86			(2<<0)	// 86% of master bias
#define NAU7802_POWER_CONTROL_PGA_CURR_70			(3<<0)	// 70% of master bias

#define NAU7802_POWER_CONTROL_PGA_CAP_EN			(1<<7)

#define NAU7802_MAX_CHAN					2

class AdcNau7802 : public AdcDevice {
public:
	bool Enable();
	void Disable();
	void Reset();
	uint32_t Rate(uint32_t Val);
	uint16_t Resolution(uint16_t Val);
	bool OpenChannel(const ADC_CHAN_CFG * const pChanCfg, int NbChan);
	void CloseChannel(int Chan);
	bool StartConversion();
	void StopConversion();
	int Read(ADC_DATA *pBuff, int Len);
	bool Read(int Chan, ADC_DATA *pBuff);
	bool Calibrate();
	bool Init(const ADC_CFG &Cfg, Timer * const pTimer, DeviceIntrf * const pIntrf);
	void IntHandler() { UpdateData(); }
	bool UpdateData();

private:
	uint8_t vChanOpened;
	int32_t vAdcVal[NAU7802_MAX_CHAN];
	ADC_DATA vData[NAU7802_MAX_CHAN];
};

/** @} End of group Converters */

#endif // __ADC_NAU7802_H__
