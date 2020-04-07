/*--------------------------------------------------------------------------
File   : pth_ms8607.cpp

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : MS8607 environment sensor implementation
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

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "sensors/tph_ms8607.h"

uint8_t crc4_PT(uint16_t *pData)
{
	int cnt;
	unsigned int n_rem=0;
	unsigned char n_bit;

	pData[0]=((pData[0]) & 0x0FFF);
	pData[7]=0;

	for (cnt = 0; cnt < 16; cnt++)
	{
		if (cnt % 2 == 1)
			n_rem ^= (unsigned short) ((pData[cnt>>1]) & 0x00FF);
		else
			n_rem ^= (unsigned short) (pData[cnt>>1]>>8);

		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}
	n_rem= ((n_rem >> 12) & 0x000F);

	return (n_rem ^ 0x00);
}

bool TphMS8607::Init(const TPHSENSOR_CFG &CfgData, DeviceIntrf *pIntrf, Timer *pTimer)
{
	//TPHSENSOR_CFG *cfg = (TPHSENSOR_CFG*)pCfgData;

	Interface(pIntrf);
	DeviceAddress(CfgData.DevAddr);

	vpTimer = pTimer;

	Reset();

	Valid(true);

	ReadPtProm();

	return true;
}

void TphMS8607::ReadPtProm()
{
	uint8_t cmd;

	cmd = MS8607_PROM_START_ADDR;

	for (int i = 0; i < 7; i++)
	{
		uint8_t d[2];
		vpIntrf->Tx(MS8607_PTDEV_ADDR, &cmd, 1);
		vpIntrf->Rx(MS8607_PTDEV_ADDR, d, 2);
		vPTProm[i] = ((uint16_t)d[0] << 8) | d[1];
		cmd += 2;
	}


//	int crc = crc4_PT(vPTProm);
}

/**
 * @brief	Set sampling frequency.
 * 		The sampling frequency is relevant only in continuous mode.
 *
 * @return	Frequency in mHz
 */
uint32_t TphMS8607::SamplingFrequency(uint32_t Freq)
{
	return 0;
}

bool TphMS8607::StartSampling()
{
	return true;
}

/**
 * @brief Set operating mode
 *
 * @param OpMode : Operating mode
 * 					- TPHSENSOR_OPMODE_SINGLE
 * 					- TPHSENSOR_OPMODE_CONTINUOUS
 * @param Freq : Sampling frequency in mHz for continuous mode
 *
 * @return true- if success
 */
bool TphMS8607::Mode(SENSOR_OPMODE OpMode, uint32_t Freq)
{
	vOpMode = OpMode;
	vSampFreq = Freq;

	switch (OpMode)
	{
		case SENSOR_OPMODE_SINGLE:
			break;
		case SENSOR_OPMODE_CONTINUOUS:
			break;
		case SENSOR_OPMODE_TIMER:
			break;
	}

	StartSampling();

	return true;
}

bool TphMS8607::Enable()
{
	State(SENSOR_STATE_IDLE);

	return true;
}

void TphMS8607::Disable()
{
	State(SENSOR_STATE_SLEEP);
}

void TphMS8607::Reset()
{
	uint8_t cmd;

	cmd = MS8607_CMD_PT_RESET;
	vpIntrf->Tx(MS8607_PTDEV_ADDR, &cmd, 1);

	cmd = MS8607_CMD_RH_RESET;
	vpIntrf->Tx(MS8607_RHDEV_ADDR, &cmd, 1);
}

bool TphMS8607::Read(TPHSENSOR_DATA &TphData)
{
//	bool retval = false;

// TODO: Create UpdateData function
// so that we get correct timestamp
	ReadTemperature();
	ReadPressure();
	ReadHumidity();

	if (vpTimer)
	{
		vTphData.Timestamp = vpTimer->mSecond();
	}

	memcpy(&TphData, &vTphData, sizeof(TPHSENSOR_DATA));

	return true;
}

float TphMS8607::ReadTemperature()
{
	uint8_t cmd = MS8607_CMD_T_CONVERT_D2_256;
	uint32_t raw = 0;
	uint8_t d[4];

	vpIntrf->Tx(MS8607_PTDEV_ADDR, &cmd, 1);

	// 560 usec conversion time for res 256
	usDelay(600);

	cmd = MS8607_CMD_ADC_READ;
	vpIntrf->Tx(MS8607_PTDEV_ADDR, &cmd, 1);
	int c = vpIntrf->Rx(MS8607_PTDEV_ADDR, d, 3);
	if ( c > 0)
	{
		uint64_t t2;

		raw = ((uint32_t)d[0] << 16) + ((uint32_t)d[1] << 8) + d[3];
		vCurDT = raw - ((int32_t)vPTProm[5] << 8L);
		vTphData.Temperature = 2000L + (((uint64_t)vCurDT * (int64_t)vPTProm[6]) >> 23LL);

		// Second order conversion
		if (vTphData.Temperature < 2000)
		{
			t2 = (3LL * vCurDT * vCurDT) >> 33LL;
		}
		else
		{
			t2 = (5LL * vCurDT * vCurDT) >> 38LL;
		}
		vTphData.Temperature -= t2;
	}

	return (float)vTphData.Temperature / 100.0;
}

float TphMS8607::ReadPressure()
{
	uint8_t cmd = MS8607_CMD_P_CONVERT_D1_256;
	uint32_t raw = 0;
	uint8_t d[4];

	vpIntrf->Tx(MS8607_PTDEV_ADDR, &cmd, 1);

	// 560 usec coversion time for res 256
	usDelay(600);

	cmd = MS8607_CMD_ADC_READ;
	vpIntrf->Tx(MS8607_PTDEV_ADDR, &cmd, 1);
	int c = vpIntrf->Rx(MS8607_PTDEV_ADDR, d, 3);
	if (c > 0)
	{
		raw = ((uint32_t)d[0] << 16) + ((uint32_t)d[1] << 8) + d[3];

		int64_t off  = ((int64_t)vPTProm[2] << 17LL) + (((int64_t)vPTProm[4] * vCurDT) >> 6LL);
		int64_t sens = ((int64_t)vPTProm[1] << 16LL) + (((int64_t)vPTProm[3] * vCurDT) >> 7LL);
		//int64_t t2;
		int64_t off2, sens2;

		// Second order compensation
		if (vTphData.Temperature < 2000)
		{
			int64_t tx2 = (vTphData.Temperature - 2000) * (vTphData.Temperature - 2000);
			off2 = (61LL * tx2) >> 4LL;
			sens2 = (29LL * tx2) >> 4LL;

			if (vTphData.Temperature < 1500)
			{
				int64_t tx1 = (vTphData.Temperature + 1500) * (vTphData.Temperature + 1500);
				off2 += 16LL * tx1;
				sens2 += 8LL * tx1;
			}
		}
		else
		{
			off2 = 0;
			sens2 = 0;
		}

		off -= off2;
		sens -= sens2;

		// pressure in mBar (1 mBar = 100 Pascal)
		int64_t p = (((int64_t)raw * (sens >> 21LL) - off) >> 15LL);

		vTphData.Pressure = p;
	}

	// pressur in Pascal
	return (float)vTphData.Pressure;
}

float TphMS8607::ReadHumidity()
{
	uint8_t cmd = MS8607_CMD_RH_HOLD_MASTER;
	uint32_t raw = 0;
	uint8_t d[4];

	vpIntrf->Tx(MS8607_RHDEV_ADDR, &cmd, 1);
	vpIntrf->Rx(MS8607_RHDEV_ADDR, (uint8_t*)d, 3);

	raw = ((uint32_t)d[0] << 8) + (d[1] & 0xFC);

	if ((d[1] & 0x3) == 2)
	{
		int32_t rh = ((12500L * raw) >> 16L) - 600L;

		rh = rh + ((2000 - vTphData.Temperature) * (-18)) / 100;

		vTphData.Humidity = rh;
	}

	 return (float)vTphData.Humidity / 100.0;
}

