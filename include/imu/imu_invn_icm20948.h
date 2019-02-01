/**-------------------------------------------------------------------------
@file	imu_invn_icm20948.h

@brief	Implementation of an Inertial Measurement Unit for Invensense ICM-20948

This is an implementation wrapper over Invensense SmartMotion for the ICM-20948
9 axis motion sensor

@author	Hoang Nguyen Hoan
@date	Dec. 26, 2018

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
#ifndef __IMU_INVN_ICM20948_H__
#define __IMU_INVN_ICM20948_H__

#include "device_intrf.h"
#include "imu/imu.h"
#include "sensors/agm_invn_icm20948.h"

class ImuInvnIcm20948 : public Imu {
public:

	//bool Init(const IMU_CFG &Cfg, uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	bool Init(const IMU_CFG &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
	virtual void IntHandler() {inv_icm20948_poll_sensor(vpIcmDevice, (void*)this, SensorEventHandler);}
	virtual IMU_FEATURE Feature(IMU_FEATURE FeatureBit, bool bEnDis);
	virtual bool Calibrate();
	virtual void SetAxisAlignmentMatrix(int8_t * const pMatrix) ;
	virtual bool Compass(bool bEn);
	virtual bool Pedometer(bool bEn);
	virtual bool Quaternion(bool bEn, int NbAxis);
	virtual bool Tap(bool bEn);

	virtual bool Read(IMU_QUAT &Data) { return Imu::Read(Data); }
	virtual bool Read(IMU_EULER &Data) { return Imu::Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(ACCELSENSOR_DATA &Data) { return Imu::Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(GYROSENSOR_DATA &Data) { return Imu::Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(MAGSENSOR_DATA &Data) { return Imu::Read(Data); }

protected:

private:
	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);
	void UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg);
	static void SensorEventHandler(void * context, enum inv_icm20948_sensor sensor, uint64_t timestamp, const void * data, const void *arg);
	static int InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
	static int InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);

	inv_icm20948_t *vpIcmDevice;
	inv_icm20948_t vIcmDevice;
	int32_t vCfgAccFsr; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
	int32_t vCfgGyroFsr; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
};



#endif // __IMU_INVN_ICM20948_H__
