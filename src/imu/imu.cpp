/**-------------------------------------------------------------------------
@file	imu.cpp

@brief	Implementation of an Inertia Measurement Unit

This a generic abstraction layer for IMU sensor fusion.  It is a mean to
provide a common interface to different sensor fusion library out there.

@author	Hoang Nguyen Hoan
@date	Aug. 1, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/

#include "imu/imu.h"
/*
bool Imu::Init(const IMU_CFG &Cfg, uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	SetEvtHandler(Cfg.EvtHandler);
	vpTimer = pTimer;
	Interface(pIntrf);
	DeviceAddess(DevAddr);

	return true;
}
*/
bool Imu::Init(const IMU_CFG &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	vpAccel = pAccel;
	vpGyro = pGyro;
	vpMag = pMag;
	SetEvtHandler(Cfg.EvtHandler);

	return true;
}

IMU_FEATURE Imu::Feature(IMU_FEATURE FeatureBit, bool bEnDis)
{
	if (bEnDis == true)
	{
		// Enable
		vActiveFeature |= FeatureBit;
	}
	else
	{
		// disable
		vActiveFeature &= ~FeatureBit;
	}

	return vActiveFeature;
}

bool Imu::Quaternion(bool bEn, int NbAxis)
{

}


