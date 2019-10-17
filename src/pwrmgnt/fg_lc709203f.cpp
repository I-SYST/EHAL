/**-------------------------------------------------------------------------
@file	fg_lc709203f.cpp

@brief	Fuel gauge implementation of the ON LC709203F


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
#include "istddef.h"
#include "crc.h"
#include "pwrmgnt/fg_lc709203f.h"

#define CRC_ATM_POLYNOMIAL		0x107

bool FgLc709203f::Init(const FUELGAUGE_CFG &Cfg, DeviceIntrf * const pIntrf, PowerMgnt * const pPwrMnt)
{
	if (pIntrf == NULL || Cfg.DevAddr != LC709203F_I2C_7BITS_DEV_ADDR)
	{
		return false;
	}

	vDevAddr = Cfg.DevAddr;
	Interface(pIntrf);

	uint8_t cmd = LC709203F_CMD_IC_VERSION;
	uint16_t d = Read16(&cmd, 1);

	if (d == 0xFF)
	{
		return false;
	}

	vBatProfile = Cfg.BatProf;

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool FgLc709203f::Enable()
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
void FgLc709203f::Disable()
{

}

/**
 * @brief	Reset device to it initial default state
 */
void FgLc709203f::Reset()
{
	uint8_t cmd = LC709203F_CMD_BEFORE_RSOC;

	Write16(&cmd, 1, 0xAA55);
}

uint16_t FgLc709203f::Level()
{
	uint8_t cmd = LC709203F_CMD_ITE;
	uint16_t d = Read16(&cmd, 1);

	return d;
}

int32_t FgLc709203f::Temperature()
{
	uint8_t cmd = LC709203F_CMD_CELL_TEMP;
	uint32_t d = Read16(&cmd, 1) - 0x9e4 - 200;

	return d;
}

int32_t FgLc709203f::Voltage()
{
	uint8_t cmd = LC709203F_CMD_CELL_VOLTAGE;
	uint32_t d = Read16(&cmd, 1);

	return d;
}

int FgLc709203f::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	uint8_t buf[10];
	int l = Device::Read(pCmdAddr, CmdAddrLen, &buf[3], BuffLen + 1);
	if (l > 0)
	{
		buf[0] = LC709203F_I2C_7BITS_DEV_ADDR << 1;
		buf[1] = *pCmdAddr;
		buf[2] = buf[0] | 1;
		uint8_t crc = crc8(CRC_ATM_POLYNOMIAL, buf, l + 2, 0);
		if (crc == buf[l + 2])
		{
			l--;
			memcpy(pBuff, &buf[3], l);

			return l;
		}
	}

	return 0;
}

int FgLc709203f::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	uint8_t buf[10];
	int len = min(8, DataLen);

	buf[0] = LC709203F_I2C_7BITS_DEV_ADDR << 1;
	buf[1] = *pCmdAddr;
	memcpy(&buf[2], pData, len);
	len += 2;
	buf[len] = crc8(CRC_ATM_POLYNOMIAL, buf, len, 0);

	return Device::Write(pCmdAddr, CmdAddrLen, &buf[2], len);
}
