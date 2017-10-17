/*--------------------------------------------------------------------------
File   : pthg_bme680.cpp

Author : Hoang Nguyen Hoan          			Oct. 15, 2017

Desc   : BME680 environment sensor implementation
			- Temperature, Humidity, Barometric pressure, Gas

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
#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "idelay.h"
#include "device_intrf.h"
#include "iopincfg.h"
#include "sensors/tphg_bme680.h"


bool TphgBme680::Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf)
{
	uint8_t regaddr = BME680_REG_ID;
	uint8_t d;

	SetInterface(pIntrf);
	SetDeviceAddess(CfgData.DevAddr);

	Read((uint8_t*)&regaddr, 1, &d, 1);
	if (d != BME680_ID)
	{
		return false;
	}

	if (CfgData.DevAddr == BME680_I2C_DEV_ADDR0 || CfgData.DevAddr == BME680_I2C_DEV_ADDR1)
	{
		// I2C mode
		vRegWrMask = 0xFF;
	}
	else
	{
		vRegWrMask = 0x7F;

		// Set default SPI page 1
		regaddr = BME680_REG_STATUS & vRegWrMask;
		d = BME680_REG_STATUS_SPI_MEM_PG;
		Write((uint8_t*)&regaddr, 1, &d, 1);
	}


	Reset();

	usDelay(30000);

	// Setup oversampling.  Datasheet recommend write humidity oversampling first
	// follow by temperature & pressure in single write operation
	d = 0;
	if (CfgData.HumOvrs > 0)
	{
		d = (CfgData.HumOvrs & 3);
	}

	regaddr = BME680_REG_CTRL_HUM & vRegWrMask;
	Write((uint8_t*)&regaddr, 1, &d, 1);

	// Need to keep temperatre & pressure oversampling
	// because of shared register with operating mode settings

	vCtrlReg = 0;

	if (CfgData.TempOvrs > 0)
	{
		vCtrlReg |= (CfgData.TempOvrs & 3) << BME680_REG_CTRL_MEAS_OSRS_T_BITPOS;
	}

	if (CfgData.PresOvrs > 0)
	{
		vCtrlReg |= (CfgData.PresOvrs & 3) << BME680_REG_CTRL_MEAS_OSRS_P_BITPOS;
	}

	regaddr = BME680_REG_CTRL_MEAS & vRegWrMask;
	Write((uint8_t*)&regaddr, 1, &vCtrlReg, 1);


	regaddr = BME680_REG_CONFIG;
	Read((uint8_t*)&regaddr, 1, &d, 1);

	regaddr &= vRegWrMask;
	d |= (CfgData.FilterCoeff << BME680_REG_CONFIG_FILTER_BITPOS) & BME680_REG_CONFIG_FILTER_MASK;
	Write((uint8_t*)&regaddr, 1, &d, 1);

	SetMode(CfgData.OpMode, CfgData.Freq);

	usDelay(10000);

	return true;
}

/**
 * @brief Set operating mode
 *
 * @param OpMode : Operating mode
 * 					- TPHSENSOR_OPMODE_SLEEP
 * 					- TPHSENSOR_OPMODE_SINGLE
 * 					- TPHSENSOR_OPMODE_CONTINUOUS
 * @param Freq : Sampling frequency in Hz for continuous mode
 *
 * @return true- if success
 */
bool TphgBme680::SetMode(TPHSENSOR_OPMODE OpMode, uint32_t Freq)
{
	uint8_t regaddr;
	uint8_t d = 0;

	vOpMode = OpMode;
	vSampFreq = Freq;

	// read current ctrl_meas register
	//regaddr = BME680_REG_CTRL_MEAS;
	//Read(&regaddr, 1, &vCtrlReg, 1);

	vCtrlReg &= ~BME680_REG_CTRL_MEAS_MODE_MASK;

	switch (OpMode)
	{
		case TPHSENSOR_OPMODE_SLEEP:
			regaddr = BME680_REG_CTRL_MEAS & vRegWrMask;
			Write(&regaddr, 1, &vCtrlReg, 1);
			break;
		case TPHSENSOR_OPMODE_CONTINUOUS:
			// There is no conituous mode.  Force back to single
		case TPHSENSOR_OPMODE_SINGLE:

			vCtrlReg |= BME680_REG_CTRL_MEAS_MODE_FORCED;
			break;
	}

	StartSampling();

	return true;
}

/**
 * @brief	Start sampling data
 *
 * @return	true - success
 */
bool TphgBme680::StartSampling()
{
	uint8_t regaddr = BME680_REG_CTRL_MEAS & vRegWrMask;
	Write(&regaddr, 1, &vCtrlReg, 1);

	return true;
}

bool TphgBme680::Enable()
{
	SetMode(TPHSENSOR_OPMODE_CONTINUOUS, vSampFreq);

	return true;
}

void TphgBme680::Disable()
{
	SetMode(TPHSENSOR_OPMODE_SLEEP, 0);
}

void TphgBme680::Reset()
{
	uint8_t addr = BME680_REG_RESET & vRegWrMask;
	uint8_t d = BME680_REG_RESET_VAL;

	Write(&addr, 1, &d, 1);
}

bool TphgBme680::ReadTPH(TPHSENSOR_DATA &PthData)
{
	uint8_t addr = BME680_REG_STATUS;
	uint8_t status = 0;
	bool retval = false;
	int timeout = 20;

	if (vOpMode == TPHSENSOR_OPMODE_SINGLE)
	{
		StartSampling();
		usDelay(20000);
	}

	do {
		usDelay(1000);
		Read(&addr, 1, &status, 1);
	} while ((status & (BME680_REG_MEAS_STATUS_0_MEASURING | BME680_REG_MEAS_STATUS_0_GAS_MEASURING)) != 0 && timeout-- > 0);

	if ((status & BME680_REG_MEAS_STATUS_0_NEW_DATA)== 0)
	{
		uint8_t d[8];
		addr = BME680_REG_PRESS_MSB;

		if (Read(&addr, 1, d, 8) == 8)
		{
			int32_t p = (((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | ((uint32_t)d[2] >> 4));
			int32_t t = (((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | ((uint32_t)d[5] >> 4));
			int32_t h = (((uint32_t)d[6] << 8) | d[7]);

			vCurTemp = t;//CompenTemp(t);
			vCurBarPres = p;//CompenPress(p) * 100 / 256;
			vCurRelHum = h;//CompenHum(h) * 100 / 1024;
			retval = true;
		}
	}

	PthData.Humidity = (int16_t)vCurRelHum;
	PthData.Pressure = (uint32_t)vCurBarPres;
	PthData.Temperature = vCurTemp;

	return retval;
}
