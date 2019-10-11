/**-------------------------------------------------------------------------
@file	accel_sensor.cpp

@brief	Generic accelerometer sensor abstraction

@author	Hoang Nguyen Hoan
@date	Nov. 18, 2017

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

#include "sensors/accel_sensor.h"

bool AccelSensor::Read(ACCELSENSOR_DATA &Data)
{
	if (vData.Range == 0)
		return false;

	Data.Timestamp = vData.Timestamp;
	Data.X = (float)vData.X * vCalibGain[0][0] + vData.Y * vCalibGain[1][0] + vData.Z * vCalibGain[2][0] + vCalibOffset[0];
	Data.Y = (float)vData.X * vCalibGain[0][1] + vData.Y * vCalibGain[1][1] + vData.Z * vCalibGain[2][1] + vCalibOffset[1];
	Data.Z = (float)vData.X * vCalibGain[0][2] + vData.Y * vCalibGain[1][2] + vData.Z * vCalibGain[2][2] + vCalibOffset[2];

	return true;
}

uint16_t AccelSensor::Scale(uint16_t Value)
{
	float scale = 0.0;

	if (vScale != 0)
	{
		scale = (float)Value / (float)vScale;
	}
	else
	{
		scale = (float)Value / (float)vRange;
	}

	for (int i = 0; i < 3; i++)
	{
		vCalibGain[0][i] *= scale;
		vCalibGain[1][i] *= scale;
		vCalibGain[2][i] *= scale;
		vCalibOffset[i] *= scale;
	}

	vScale = Value;

	return vScale;
}

void AccelSensor::SetCalibration(float (&Gain)[3][3], float (&Offset)[3])
{
	float scale = (float)vScale / (float)vRange;

	for (int i = 0; i < 3; i++)
	{
		vCalibGain[0][i] = Gain[0][i] * scale;
		vCalibGain[1][i] = Gain[1][i] * scale;
		vCalibGain[2][i] = Gain[2][i] * scale;
		vCalibOffset[i] = Offset[i] * scale;
	}
}

void AccelSensor::ClearCalibration()
{
	memset(vCalibOffset, 0, sizeof(float) * 3);
	memset(vCalibGain, 0, sizeof(float) * 9);

	if (vScale == 0 || vRange == 0)
	{
		vCalibGain[0][0] = 1.0;
		vCalibGain[1][1] = vCalibGain[0][0];
		vCalibGain[2][2] = vCalibGain[0][0];
	}
	else
	{
		vCalibGain[0][0] = (float)vScale / (float)vRange;
		vCalibGain[1][1] = vCalibGain[0][0];
		vCalibGain[2][2] = vCalibGain[0][0];
	}
}
