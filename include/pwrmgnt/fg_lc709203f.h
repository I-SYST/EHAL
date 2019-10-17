/**-------------------------------------------------------------------------
@file	fg_lc709203f.h

@brief	Fuel gauge implementation of the ON Semi LC709203F

NOTE: Notice for I2C communication shared with another device
When the I2C Bus (on which the Fuel Gauge LSI is connected) is shared with
another device the Fuel Gauge LSI must be in its operation mode before the
other Device starts I2C communication.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2019

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
#ifndef __FG_LC709203F_H__
#define __FG_LC709203F_H__

#include <stdint.h>

#include "pwrmgnt/pwrmgnt.h"

/** @addtogroup Power
  * @{
  */

#define LC709203F_I2C_7BITS_DEV_ADDR			0x0B

#define LC709203F_CMD_BEFORE_RSOC				4		//!< Executes RSOC initialization
														//!< with sampled maximum voltage when 0xAA55 is set.
#define LC709203F_CMD_THERMISTOR_B				6		//!< Sets B-constant of the thermistor to be measured. in 1K
#define LC709203F_CMD_INITIAL_RSOC				7		//!< Executes RSOC initialization when 0xAA55 is set.
#define LC709203F_CMD_CELL_TEMP					8		//!< Cell temperature
#define LC709203F_CMD_CELL_VOLTAGE				9		//!< Read cell voltage
#define LC709203F_CMD_CURRENT_DIR				0xA		//!< Charge mode
#define LC709203F_CMD_APA						0xB		//!< Set parasitic impedance
#define LC709203F_CMD_APT						0xC		//!< Sets a value to adjust temperature measurement delay timing
#define LC709203F_CMD_RSOC						0xD		//!< Displays RSOC value based on a 0-100 scale 1%
#define LC709203F_CMD_ITE						0xF		//!< Displays RSOC value based on a 0-1000 scale 0.1%
#define LC709203F_CMD_IC_VERSION				0x11	//!<
#define LC709203F_CMD_CHANGE_PARAM				0x12	//!< Selects a battery profile
#define LC709203F_CMD_ALARM_LOW_RSOC			0x13	//!< Sets RSOC threshold to generate Alarm signal. 1%
#define LC709203F_CMD_ALARM_LOW_CELL_VOLTAGE	0x14	//!< Sets Voltage threshold to generate Alarm signal. 1mV
#define LC709203F_CMD_IC_POWER_MODE				0x15	//!< Select power mode
#define LC709203F_CMD_STATUS					0x16	//!< Selects Temperature obtaining method
#define LC709203F_CMD_NUMBER_PARAMETER			0x1A	//!< Displays Battery profile code

class FgLc709203f : public FuelGauge {
public:
	bool Init(const FUELGAUGE_CFG &Cfg, DeviceIntrf * const pIntrf, PowerMgnt * const pPwrMnt);

	/**
	 * @brief	Get battery level
	 *
	 * Returns battery level in 1 digit fixed point decimal.
	 *
	 * ex. 123 => 12.3%
	 *
	 * @return	Battery level in (0-100) % in 1 digit fixed point
	 */
	uint16_t Level();
	int32_t Temperature();
	int32_t Voltage();

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

protected:
	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif //__FG_LC709203F_H__
