/**-------------------------------------------------------------------------
@file	mag_sensor.cpp

@brief	Generic magnetometer sensor abstraction

@author	Hoang Nguyen Hoan
@date	Sept. 16, 2019

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
#include <string.h>

#include "sensors/mag_sensor.h"

bool MagSensor::Read(MAGSENSOR_DATA &Data)
{
	Data.Timestamp = vData.Timestamp;
	Data.X = (float)vData.X * vCalibGain[0][0] + (float)vData.Y * vCalibGain[1][0] + (float)vData.Z * vCalibGain[2][0] + vCalibOffset[0];
	Data.Y = (float)vData.X * vCalibGain[0][1] + (float)vData.Y * vCalibGain[1][1] + (float)vData.Z * vCalibGain[2][1] + vCalibOffset[1];
	Data.Z = (float)vData.X * vCalibGain[0][2] + (float)vData.Y * vCalibGain[1][2] + (float)vData.Z * vCalibGain[2][2] + vCalibOffset[2];

	return true;
}

void MagSensor::Sensitivity(uint16_t (&Sen)[3])
{
	for (int i = 0; i < 3; i++)
	{
		vCalibGain[0][i] = (vCalibGain[0][i] * Sen[i]) / (vSensitivity[i] * 1000.0);
		vCalibGain[1][i] = (vCalibGain[1][i] * Sen[i]) / (vSensitivity[i] * 1000.0);
		vCalibGain[2][i] = (vCalibGain[2][i] * Sen[i]) / (vSensitivity[i] * 1000.0);
		vCalibOffset[i] = (vCalibOffset[i] * Sen[i]) / (vSensitivity[i] * 1000.0);
		vSensitivity[i] = Sen[i];
	}
}

void MagSensor::SetCalibration(float (&Gain)[3][3], float (&Offset)[3])
{
	for (int i = 0; i < 3; i++)
	{
		vCalibGain[0][i] = Gain[0][i] * vSensitivity[i] / 1000.0;
		vCalibGain[1][i] = Gain[1][i] * vSensitivity[i] / 1000.0;
		vCalibGain[2][i] = Gain[2][i] * vSensitivity[i] / 1000.0;
		vCalibOffset[i] = Offset[i] * vSensitivity[i] / 1000.0;
	}
}

void MagSensor::ClearCalibration()
{
	memset(vCalibOffset, 0, sizeof(float) * 3);
	memset(vCalibGain, 0, sizeof(float) * 9);

	vCalibGain[0][0] = vSensitivity[0] / 1000.0;
	vCalibGain[1][1] = vSensitivity[1] / 1000.0;
	vCalibGain[2][2] = vSensitivity[2] / 1000.0;
}

int MagSensor::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// Most sensor that supports SPI have this for reading registers
		// overload this function if different
		*pCmdAddr |= 0x80;
	}

	return vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int MagSensor::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// Most sensor that supports SPI have this for writing registers
		// overload this function if different
		*pCmdAddr &= 0x7F;
	}

	return vpIntrf->Write(DevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
}

