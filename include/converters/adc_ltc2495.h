/*--------------------------------------------------------------------------
File   : adc_ltc2495.h

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
#ifndef __ADC_LTC2495_H__
#define __ADC_LTC2495_H__

#include <stdint.h>

#include "converters/adc_device.h"

#define LTC2495_I2C_ADDR					0x14	// LTC2495 I2C slave address

// Differential channels
#define LTC2495_DIFF_CHAN0P					0xA0	// Chan 0+, 1-
#define LTC2495_DIFF_CHAN1P					0xA1	// Chan 2+, 3-
#define LTC2495_DIFF_CHAN2P					0xA2	// Chan 4+, 5-
#define LTC2495_DIFF_CHAN3P					0xA3	// Chan 6+, 7-
#define LTC2495_DIFF_CHAN4P					0xA4	// Chan 8+, 9-
#define LTC2495_DIFF_CHAN5P					0xA5	// Chan 10+, 11-
#define LTC2495_DIFF_CHAN6P					0xA6	// Chan 12+, 13-
#define LTC2495_DIFF_CHAN7P					0xA7	// Chan 14+, 15-

#define LTC2495_DIFF_CHAN0N					0xA8	// Chan 0-, 1+
#define LTC2495_DIFF_CHAN1N					0xA9	// Chan 2-, 3+
#define LTC2495_DIFF_CHAN2N					0xAA	// Chan 4-, 5+
#define LTC2495_DIFF_CHAN3N					0xAB	// Chan 6-, 7+
#define LTC2495_DIFF_CHAN4N					0xAC	// Chan 8-, 9+
#define LTC2495_DIFF_CHAN5N					0xAD	// Chan 10-, 11+
#define LTC2495_DIFF_CHAN6N					0xAE	// Chan 12-, 13+
#define LTC2495_DIFF_CHAN7N					0xAF	// Chan 14-, 15+

// Single ended channels
#define LTC2495_SGL_CHAN0					0xB0	// Chan 0
#define LTC2495_SGL_CHAN1					0xB8	// Chan 1
#define LTC2495_SGL_CHAN2					0xB1	// Chan 2
#define LTC2495_SGL_CHAN3					0xB9	// Chan 3
#define LTC2495_SGL_CHAN4					0xB2	// Chan 4
#define LTC2495_SGL_CHAN5					0xBA	// Chan 5
#define LTC2495_SGL_CHAN6					0xB3	// Chan 6
#define LTC2495_SGL_CHAN7					0xBB	// Chan 7
#define LTC2495_SGL_CHAN8					0xB4	// Chan 8
#define LTC2495_SGL_CHAN9					0xBC	// Chan 9
#define LTC2495_SGL_CHAN10					0xB5	// Chan 10
#define LTC2495_SGL_CHAN11					0xBD	// Chan 11
#define LTC2495_SGL_CHAN12					0xB6	// Chan 12
#define LTC2495_SGL_CHAN13					0xBE	// Chan 13
#define LTC2495_SGL_CHAN14					0xB7	// Chan 14
#define LTC2495_SGL_CHAN15					0xBF	// Chan 15

#define LTC2495_CONVERT_CFG_NONE			0
#define LTC2495_CONVERT_CFG_GAIN1			0x80
#define LTC2495_CONVERT_CFG_GAIN4			0x81
#define LTC2495_CONVERT_CFG_GAIN8			0x82
#define LTC2495_CONVERT_CFG_GAIN16			0x83
#define LTC2495_CONVERT_CFG_GAIN32			0x84
#define LTC2495_CONVERT_CFG_GAIN64			0x85
#define LTC2495_CONVERT_CFG_GAIN128			0x86
#define LTC2495_CONVERT_CFG_GAIN256			0x87

#define LTC2495_MAX_CHAN					16

class AdcLTC2495 : public AdcDevice {
public:
	/**
	 * @brief	ADC device initialization
	 *
	 * @param	Cfg 	: Configuration data
	 * 			pIntrf	: Pointer to device interface instance
	 * 					  NULL, if self interface or internal SoC
	 * 					  such as MCU ADC pins
	 *
	 * @return	True - Success
	 */
	virtual bool Init(const ADC_CFG &Cfg, DeviceIntrf *pIntrf);

	/**
	 * @brief	Set conversion rate for continuous mode only
	 *
	 * @param 	Val : Rate value in Hz
	 *
	 * @return	Real rate value set in Hz
	 */
	virtual uint32_t Rate(uint32_t Val);

	/**
	 * @brief	Set conversion resolution
	 *
	 * @param 	Val : Resolution value in bits
	 *
	 * @return	Real resolution value set in bits
	 */
	virtual uint16_t Resolution(uint16_t Val);

	/**
	 * @brief	Configure channel for ADC conversion
	 *
	 * @param	pChanCfg : Array of channels to configure
	 * 			NbChan	 : Number of channels (array size)
	 *
	 * @return	True - Success
	 */
	virtual bool OpenChannel(const ADC_CHAN_CFG *pChanCfg, int NbChan);

	/**
	 * @brief	Close ADC channel
	 *
	 * @param 	Chan : Channel number
	 */
	virtual void CloseChannel(int Chan);

	/**
	 * @brief	Start ADC conversion process
	 *
	 * @return	True - Success
	 */
	virtual bool StartConversion();

	/**
	 * @brief	Stop ADC conversion
	 */
	virtual void StopConversion();

	/**
	 * @brief	Read converted data
	 *
	 * @param	pBuff : Buffer to receive converted data
	 * 			Len	  : Size of buffer array (total number of elements)
	 *
	 * @return	Number of ADC data in array.
	 */
	virtual int Read(ADC_DATA *pBuff, int Len);

	/**
	 * @brief	Execute auto calibration
	 *
	 * @return	true - success
	 */
	virtual bool Calibrate();

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

private:
	float vVFullScale[LTC2495_MAX_CHAN];
	int vRefVoltage;

};

#endif // __ADC_LTC2495_H__
