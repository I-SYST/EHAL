/**-------------------------------------------------------------------------
@file	imu_mpl_mpu9250.h

@brief	Implementation of an Inertial Measurement Unit of InvenSense MPU-9250

Implements the DMP (Digital Motion Processor) driver portion of the MPU-9250

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

#ifndef __IMU_MPU9250_H__
#define __IMU_MPU9250_H__

#include "imu/imu.h"
#include "imu/mpu9250_dmpkey.h"
#include "imu/mpu9250_dmpmap.h"
#include "sensors/agm_mpu9250.h"

class ImuMpu9250 : public Imu {
public:
	bool Init(const IMU_CFG &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
	virtual void IntHandler();
	uint32_t Rate(uint32_t DataRate);

protected:
	/**
	 * @brief	Read device's register/memory block
	 *
	 * @param 	pCmdAddr 	: Buffer containing command or address to be written
	 * 						  prior reading data back
	 * @param	CmdAddrLen 	: Command buffer size
	 * @param	pBuff		: Data buffer container
	 * @param	BuffLen		: Data buffer size
	 *
	 * @return	Actual number of bytes read
	 */
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		return vpMpu->Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	/**
	 * @brief	Write to device's register/memory block
	 *
	 * @param 	pCmdAddr 	: Buffer containing command or address to be written
	 * 						  prior writing data back
	 * @param	CmdAddrLen 	: Command buffer size
	 * @param	pData		: Data buffer to be written to the device
	 * @param	DataLen		: Size of data
	 *
	 * @return	Actual number of bytes written
	 */
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		return vpMpu->Write(pCmdAddr, CmdAddrLen, pData, DataLen);
	}


protected:
	virtual int Read(uint16_t Addr, uint8_t *pBuff, int Len);
	virtual int Write(uint16_t Addr, uint8_t *pData, int Len);


private:

	bool UploadDMPImage();

	AgmMpu9250 *vpMpu;
};

#endif // __IMU_MPU9250_H__
