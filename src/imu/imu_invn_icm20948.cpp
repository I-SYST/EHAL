/**-------------------------------------------------------------------------
@file	imu_invn_icm20948.cpp

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
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
#include "Devices/SensorTypes.h"

#include "idelay.h"
#include "imu/imu_invn_icm20948.h"
#include "sensors/agm_invn_icm20948.h"

#define ICM20948_WHO_AM_I_ID		0xEA

#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

#if 0
int ImuInvnIcm20948::InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	ImuInvnIcm20948 *dev = (ImuInvnIcm20948*)context;
//	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
//	reg |= 0x80;
	int cnt = dev->Read(&reg, 1, rbuffer, (int)rlen);

	return cnt > 0 ? 0 : 1;
}

int ImuInvnIcm20948::InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	ImuInvnIcm20948 *dev = (ImuInvnIcm20948*)context;
//	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

	int cnt = dev->Write(&reg, 1, (uint8_t*)wbuffer, (int)wlen);

	return cnt > 0 ? 0 : 1;
}

bool ImuInvnIcm20948::Init(const IMU_CFG &Cfg, uint32_t DevAddr, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Imu::Init(Cfg, DevAddr, pIntrf, pTimer);
	//Interface(pIntrf);
	//DeviceAddess(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	struct inv_icm20948_serif icm20948_serif;

	icm20948_serif.context   = this;
	icm20948_serif.read_reg  = InvnReadReg;
	icm20948_serif.write_reg = InvnWriteReg;
	icm20948_serif.max_read  = 1024*16; /* maximum number of bytes allowed per serial read */
	icm20948_serif.max_write = 1024*16; /* maximum number of bytes allowed per serial write */
	icm20948_serif.is_spi = vpIntrf->Type() == DEVINTRF_TYPE_SPI;

	inv_icm20948_reset_states(&vIcmDevice, &icm20948_serif);

	inv_icm20948_register_aux_compass(&vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, (uint8_t)AK0991x_DEFAULT_I2C_ADDR);

	uint8_t d;

	int rc = 	rc = inv_icm20948_get_whoami(&vIcmDevice, &d);

	if (d != ICM20948_WHO_AM_I_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	usDelay(500000);

	// Setup accel and gyro mounting matrix and associated angle for current board
	inv_icm20948_init_matrix(&vIcmDevice);

	for (int i = 0; i < INV_ICM20948_SENSOR_MAX; i++) {
		inv_icm20948_set_matrix(&vIcmDevice, s_CfgMountingMatrix, (inv_icm20948_sensor)i);
	}

	rc = inv_icm20948_initialize(&vIcmDevice, s_Dmp3Image, sizeof(s_Dmp3Image));
	/* Initialize auxiliary sensors */
	inv_icm20948_register_aux_compass( &vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
	rc = inv_icm20948_initialize_auxiliary(&vIcmDevice);

	// re-initialize base state structure
	inv_icm20948_init_structure(&vIcmDevice);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&vCfgAccFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&vCfgAccFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&vCfgGyroFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&vCfgGyroFsr);
	inv_icm20948_set_fsr(&vIcmDevice, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&vCfgGyroFsr);

	rc = inv_icm20948_load(&vIcmDevice, s_Dmp3Image, sizeof(s_Dmp3Image));

	vpIcmDevice= &vIcmDevice;

	return true;
}
#endif

bool ImuInvnIcm20948::Init(const IMU_CFG &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == NULL)
	{
		return false;
	}

	Imu::Init(Cfg, pAccel, pGyro, pMag);

	//vpSensorDev = (AgmInvnIcm20948*)pAccel;
	vEvtHandler = Cfg.EvtHandler;
	vpIcmDevice = *(AgmInvnIcm20948*)pAccel;

	return true;
}

bool ImuInvnIcm20948::Enable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
	while(i-- > 0) {
		inv_icm20948_enable_sensor(vpIcmDevice, (inv_icm20948_sensor)i, 1);
	}

	return true;
}

void ImuInvnIcm20948::Disable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
	while(i-- > 0) {
		inv_icm20948_enable_sensor(vpIcmDevice, (inv_icm20948_sensor)i, 0);
	}
	inv_icm20948_set_chip_power_state(vpIcmDevice, CHIP_AWAKE, 0);
}

void ImuInvnIcm20948::Reset()
{
	inv_icm20948_soft_reset(vpIcmDevice);
}

IMU_FEATURE ImuInvnIcm20948::Feature(IMU_FEATURE FeatureBit, bool bEnDis)
{
	return Imu::Feature();
}

bool ImuInvnIcm20948::Calibrate()
{
	return true;
}

void ImuInvnIcm20948::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool ImuInvnIcm20948::Compass(bool bEn)
{
	return true;
}

bool ImuInvnIcm20948::Pedometer(bool bEn)
{
	return true;
}

bool ImuInvnIcm20948::Quaternion(bool bEn, int NbAxis)
{
	return true;
}

bool ImuInvnIcm20948::Tap(bool bEn)
{
	return true;
}

bool ImuInvnIcm20948::UpdateData()
{
	return true;
}
/*
void ImuInvnIcm20948::IntHandler()
{
	//if (vpIcmDev)
	{

//		inv_icm20948_poll_sensor(*vpIcmDev, (void*)this, SensorEventHandler);
	}
	//else
	{
		inv_icm20948_poll_sensor(vpIcmDevice, (void*)this, SensorEventHandler);
	}
}
*/
void ImuInvnIcm20948::SensorEventHandler(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	ImuInvnIcm20948 *dev = (ImuInvnIcm20948*)context;

	dev->UpdateData(sensortype, timestamp, data, arg);
}

void ImuInvnIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	//float raw_bias_data[6];
	inv_sensor_event_t event;
	//uint8_t sensor_id = convert_to_generic_ids[sensortype];

	memset((void *)&event, 0, sizeof(event));
	event.sensor = sensortype;
	event.timestamp = timestamp;
	switch (sensortype)
	{
/*	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
		memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
		memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GYROSCOPE:
		memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GRAVITY:
		memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
		event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
		break;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
	case INV_ICM20948_SENSOR_ACCELEROMETER:
		memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
		memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
		vAccData.X = event.data.acc.vect[0] * 256.0;
		vAccData.Y = event.data.acc.vect[1] * 256.0;
		vAccData.Z = event.data.acc.vect[2] * 256.0;
		vAccData.Timestamp = timestamp;
		printf("a %d : %d %d %d\r\n", vAccData.Timestamp, vAccData.X, vAccData.Y, vAccData.Z);
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
		memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
		break;*/
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:
		memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
//		vQuat.Q1 = event.data.quaternion.quat[0] * 16384.0;
//		vQuat.Q2 = event.data.quaternion.quat[1] * 16384.0;
//		vQuat.Q3 = event.data.quaternion.quat[2] * 16384.0;
//		vQuat.Q4 = event.data.quaternion.quat[3] * 16384.0;
//		vQuat.Timestamp = timestamp;
		break;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
	{
		uint8_t accel_accuracy;
		uint8_t gyro_accuracy;

		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

		accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
		gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();

		event.data.quaternion.accuracy_flag = min(accel_accuracy, gyro_accuracy);
		vQuat.Q1 = event.data.quaternion.quat[0];// * 32768.0;
		vQuat.Q2 = event.data.quaternion.quat[1];// * 32768.0;
		vQuat.Q3 = event.data.quaternion.quat[2];// * 32768.0;
		vQuat.Q4 = event.data.quaternion.quat[3];// * 32768.0;
		vQuat.Timestamp = timestamp;
	}
		break;
	case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:
		memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
		break;
	case INV_ICM20948_SENSOR_FLIP_PICKUP:
	case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:
	case INV_ICM20948_SENSOR_STEP_DETECTOR:
	case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
		event.data.event = true;
		break;
	case INV_ICM20948_SENSOR_B2S:
		event.data.event = true;
		memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
		break;
	case INV_ICM20948_SENSOR_STEP_COUNTER:
		memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
		break;
	case INV_ICM20948_SENSOR_ORIENTATION:
		//we just want to copy x,y,z from orientation data
		memcpy(&(event.data.orientation), data, 3*sizeof(float));
		break;
/*	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
		memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
		break;*/
	default:
		((AgmInvnIcm20948*)vpAccel)->UpdateData(sensortype, timestamp, data, arg);
		return;
	}

	if (vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}

int ImuInvnIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr |= 0x80;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int ImuInvnIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		*pCmdAddr &= 0x7F;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

