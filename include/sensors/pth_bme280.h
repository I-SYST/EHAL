/*--------------------------------------------------------------------------
File   : pth_bme280.h

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : BME280 environment sensor implementation
			- Temperature, Humidity, Barometric pressure

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
#ifndef __PTH_BME280_H__
#define __PTH_BME280_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "pth_sensor.h"

// Device address depending on SDO wiring
#define BME280_DEV_ADDR0				0x76	// SDO to GND
#define BME280_DEV_ADDR1				0x77	// SDO to VCC

#define BME280_REG_HUM_LSB				0xFE
#define BME280_REG_HUM_MSB				0xFD
#define BME280_REG_TEMP_XLSB			0xFC
#define BME280_REG_TEMP_LSB				0xFB
#define BME280_REG_TEMP_MSB				0xFA
#define BME280_REG_PRESS_XLSB			0xF9
#define BME280_REG_PRESS_LSB			0xF8
#define BME280_REG_PRESS_MSB			0xF7
#define BME280_REG_CONFIG				0xF5
#define BME280_REG_CTRL_MEAS			0xF4
#define BME280_REG_STATUS				0xF3
#define BME280_REG_CTRL_HUM				0xF2
#define BME280_REG_CALIB_26_41_START	0xE1
#define BME280_REG_RESET				0xE0
#define BME280_REG_ID					0xD0
#define BME280_REG_CALIB_00_25_START	0x88

#define BME280_ID						0x60

#pragma pack(push, 4)

typedef struct {

} BME280_CFG;

#pragma pack(pop)

#ifdef __cplusplus

class BME280 : public PTHSensor {
public:
	BME280() : vCalibTFine(0) {}
	virtual ~BME280() {}
	virtual bool Init(void *pCfgData, DeviceIntrf *pIntrf);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	bool ReadPTH(PTHSENSOR_DATA &PthData);

private:

	uint32_t CompenPress(int32_t RawPress);
	int32_t CompenTemp(int32_t RawTemp);
	uint32_t CompenHum(int32_t RawHum);

	int32_t vCurTemp;
	int32_t vCurBarPres;
	int32_t vCurRelHum;
	int32_t vCalibTFine;	// For internal calibration use only
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __PHT_BME280_H__
