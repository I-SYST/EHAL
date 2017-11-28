/*--------------------------------------------------------------------------
File   : tph_ms8607.h

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : MS8607 environment sensor implementation
			- Temperature, Pressure, Humidity

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
#ifndef __TPH_MS8607_H__
#define __TPH_MS8607_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "tph_sensor.h"

// Device address
#define MS8607_PTDEV_ADDR				0x76
#define MS8607_RHDEV_ADDR				0x40

#define MS8607_CMD_PT_RESET				0x1E
#define MS8607_CMD_ADC_READ				0
#define MS8607_CMD_P_CONVERT_D1_256		0x40
#define MS8607_CMD_P_CONVERT_D1_512		0x42
#define MS8607_CMD_P_CONVERT_D1_1024	0x44
#define MS8607_CMD_T_CONVERT_D2_256		0x50
#define MS8607_CMD_T_CONVERT_D2_512		0x52
#define MS8607_CMD_T_CONVERT_D2_1024	0x54

#define MS8607_CMD_RH_RESET				0xFE
#define MS8607_CMD_RH_HOLD_MASTER		0xE5
#define MS8607_CMD_RH_WRITE_USER		0xE6
#define MS8607_CMD_RH_READ_USER			0xE7

#define MS8607_PROM_START_ADDR			0xA0

#ifdef __cplusplus

class TphMS8607 : public TPHSensor {
public:
	TphMS8607() {}
	virtual ~TphMS8607() {}
	virtual bool Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer = NULL);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * @return	None
	 */
	virtual void Disable();

	/**
	 * @brief	Reset device to it initial state
	 *
	 * @return	None
	 */
	virtual void Reset();

	/**
	 * @brief Set operating mode
	 *
	 * @param OpMode : Operating mode
	 * 					- SENSOR_OPMODE_SINGLE
	 * 					- SENSOR_OPMODE_CONTINUOUS
	 * @param Freq : Sampling frequency in Hz for continuous mode
	 *
	 * @return true- if success
	 */
	virtual bool SetMode(SENSOR_OPMODE OpMode, uint32_t Freq);

	/**
	 * @brief	Set sampling frequency.
	 * 		The sampling frequency is relevant only in continuous mode.
	 *
	 * @return	Frequency in Hz
	 */
	virtual uint32_t SamplingFrequency(uint32_t FreqHz);

	/**
	 * @brief	Start sampling data
	 *
	 * @return	true - success
	 */
	virtual bool StartSampling();

	/**
	 * @brief	Read TPH data
	 * 			Read TPH data from device if available. If not
	 * 			return previous data.
	 *
	 * @param 	TphData : TPH data to return
	 *
	 * @return	true - new data
	 * 			false - old data
	 */
	bool Read(TPHSENSOR_DATA &TphData);
	float ReadTemperature();
	float ReadPressure();
	float ReadHumidity();

private:

	void ReadPtProm();

	uint16_t vPTProm[8];
	int32_t vCurDT;
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __TPH_MS8607_H__
