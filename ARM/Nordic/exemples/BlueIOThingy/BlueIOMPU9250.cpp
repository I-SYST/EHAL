/*
 * BLueIOMPU9250.h
 *
 *  Created on: Jul 25, 2018
 *      Author: hoan
 */
#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"
#include "device_intrf.h"
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "sensors/agm_mpu9250.h"
#include "imu/imu_mpu9250.h"
#include "idelay.h"
#include "board.h"
#include "BlueIOThingy.h"
#include "BlueIOMPU9250.h"

static const ACCELSENSOR_CFG s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Scale = 2,
	.LPFreq = 0,
	.bInter = true,
	.IntPol = DEVINTR_POL_LOW,
};

static const GYROSENSOR_CFG s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 10,
	.LPFreq = 200,
};

static const MAGSENSOR_CFG s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,//SENSOR_OPMODE_SINGLE,
	.Freq = 50000,
	.Precision = 14,
};

AgmMpu9250 g_Mpu9250;

static void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const IMU_CFG s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

static ImuMpu9250 s_Imu;

static Timer * s_pTimer;
static uint32_t s_MotionFeature = 0;

int8_t g_AlignMatrix[9] = {
	0,  1,  0,
    -1,  0,  0,
    0,  0, -1
};
#if 0
/**@brief Acclerometer rotation matrix.
 *
 * @note Accellerometer inverted to get positive readings when axis is aligned with g (down).
 */
static struct platform_data_s s_accel_pdata =
{
    .orientation = {  0,  1,  0,
                      -1,  0,  0,
                      0,  0, -1}
};


/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
#if 1
static struct platform_data_s gyro_pdata = {
    .orientation = { 0, 1, 0,
                     -1, 0, 0,
                     0, 0, -1}
};
#else
// BLUEIO-TAG-EVIM
static struct platform_data_s gyro_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, 1}
};
#endif

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
#if 1
	.orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, -1}
#else
    .orientation = { 1, 0, 0,
                     0, -1, 0,
                     0, 0, -1}
#endif
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif
#endif

void ImuRawDataSend(ACCELSENSOR_DATA &AccData, GYROSENSOR_DATA GyroData, MAGSENSOR_DATA &MagData);
void ImuQuatDataSend(long Quat[4]);

static void ImuDataChedHandler(void * p_event_data, uint16_t event_size)
{
	ACCELSENSOR_DATA accdata;
	GYROSENSOR_DATA gyrodata;
	MAGSENSOR_DATA magdata;
	IMU_QUAT quat;
	long q[4];

	s_Imu.Read(accdata);
	s_Imu.Read(gyrodata);
	s_Imu.Read(magdata);
	ImuRawDataSend(accdata, gyrodata, magdata);
	s_Imu.Read(quat);
	//q[0] = ((float)quat.Q[0] / 32768.0) * (float)(1<<30);
	//q[1] = ((float)quat.Q[1] / 32768.0) * (float)(1<<30);
	//q[2] = ((float)quat.Q[2] / 32768.0) * (float)(1<<30);
	//q[3] = ((float)quat.Q[3] / 32768.0) * (float)(1<<30);
	//q[0] = quat.Q[0] << 15;
	//q[1] = quat.Q[1] << 15;
	//q[2] = quat.Q[2] << 15;
	//q[3] = quat.Q[3] << 15;
	q[0] = quat.Q[0] * (1 << 30);
	q[1] = quat.Q[1] * (1 << 30);
	q[2] = quat.Q[2] * (1 << 30);
	q[3] = quat.Q[3] * (1 << 30);
	//printf("Quat %d: %d %d %d %d\r\n", quat.Timestamp, q[0], q[1], q[2], q[3]);
	ImuQuatDataSend(q);
}

static void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{

	switch (Evt)
	{
		case DEV_EVT_DATA_RDY:
			app_sched_event_put(NULL, 0, ImuDataChedHandler);
			//ImuDataChedHandler(NULL, 0);
			//g_MotSensor.Read(accdata);
			break;
	}
}

void MPU9250IntHandler(int IntNo)
{
	s_Imu.IntHandler();

	return;
}

void mpulib_data_handler_cb()
{

}

void MPU9250EnableFeature(uint32_t Feature)
{
	s_MotionFeature |= Feature;
}

static void mpulib_tap_cb(unsigned char direction, unsigned char count)
{
//	ble_tms_tap_t tap;

//	tap.dir = direction;
//	tap.cnt = count;

//	BleSrvcCharNotify(GetImuSrvcInstance(), 2, (uint8_t*)&tap, sizeof(ble_tms_tap_t));

/*    if (m_motion.features & DRV_MOTION_FEATURE_MASK_TAP)
    {
        drv_motion_evt_t evt     = DRV_MOTION_EVT_TAP;
        uint8_t          data[2] = {direction, count};

        m_motion.evt_handler(&evt, data, sizeof(data));
    }
*/
    #ifdef MOTION_DEBUG
        switch (direction)
        {
            case TAP_X_UP:
                NRF_LOG_DEBUG("drv_motion: tap x+ ");
                break;
            case TAP_X_DOWN:
                NRF_LOG_DEBUG("drv_motion: tap x- ");
                break;
            case TAP_Y_UP:
                NRF_LOG_DEBUG("drv_motion: tap y+ ");
                break;
            case TAP_Y_DOWN:
                NRF_LOG_DEBUG("drv_motion: tap y- ");
                break;
            case TAP_Z_UP:
                NRF_LOG_DEBUG("drv_motion: tap z+ ");
                break;
            case TAP_Z_DOWN:
                NRF_LOG_DEBUG("drv_motion: tap z- ");
                break;
            default:
                return;
        }

        NRF_LOG_DEBUG("x%d\r\n", count);
    #endif
}


static void mpulib_orient_cb(unsigned char orientation)
{
	BleSrvcCharNotify(GetImuSrvcInstance(), 2, &orientation, 1);

/*    if (m_motion.features & DRV_MOTION_FEATURE_MASK_ORIENTATION)
    {
        drv_motion_evt_t evt     = DRV_MOTION_EVT_ORIENTATION;

        m_motion.evt_handler(&evt, &orientation, 1);
    }
*/
    #ifdef MOTION_DEBUG
        switch (orientation)
        {
            case ANDROID_ORIENT_PORTRAIT:
                NRF_LOG_DEBUG("Portrait\r\n");
                break;
            case ANDROID_ORIENT_LANDSCAPE:
                NRF_LOG_DEBUG("Landscape\r\n");
                break;
            case ANDROID_ORIENT_REVERSE_PORTRAIT:
                NRF_LOG_DEBUG("Reverse Portrait\r\n");
                break;
            case ANDROID_ORIENT_REVERSE_LANDSCAPE:
                NRF_LOG_DEBUG("Reverse Landscape\r\n");
                break;
            default:
                return;
        }
    #endif
}

SPI *g_pSpi = NULL;

bool MPU9250Init(DeviceIntrf * const pIntrF, Timer * const pTimer)
{
	g_pSpi = (SPI*)pIntrF;
	s_pTimer = pTimer;


	bool res = g_Mpu9250.Init(s_AccelCfg, pIntrF, pTimer);
	if (res == false)
		return res;

	g_Mpu9250.Init(s_GyroCfg, NULL);
	g_Mpu9250.Init(s_MagCfg, NULL);

	IOPinConfig(BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP,
			IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	IOPinEnableInterrupt(BLUEIO_TAG_EVIM_IMU_INT_NO, 6, BLUEIO_TAG_EVIM_IMU_INT_PORT,
						 BLUEIO_TAG_EVIM_IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION,
						 MPU9250IntHandler);



//	g_Mpu9250.Enable();

#if 0
	while (1)
	{
		long l[3];
		g_Mpu9250.UpdateData();
		ACCELSENSOR_DATA accdata;
		g_Mpu9250.Read(accdata);
		l[0] = accdata.X;
		l[1] = accdata.Y;
		l[2] = accdata.Z;
		inv_build_accel(l, 0, accdata.Timestamp);
	}
#endif

    s_Imu.Init(s_ImuCfg, &g_Mpu9250, &g_Mpu9250, &g_Mpu9250);
    s_Imu.SetAxisAlignmentMatrix(g_AlignMatrix);
    s_Imu.Quaternion(true, 6);
    //s_Imu.Compass(true);

    return true;
}

int Mpu9250AuxRead(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	uint8_t regaddr;
	uint8_t d[8];

	d[0] = MPU9250_AG_I2C_SLV0_ADDR;
	d[1] = DevAddr | MPU9250_AG_I2C_SLV0_ADDR_I2C_SLVO_RD;
	d[2] = *pCmdAddr;

	while (BuffLen > 0)
	{
		int cnt = min(15, BuffLen);

		d[3] = MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_EN |cnt;

		g_pSpi->Write(0, d, 4, NULL, 0);

		// Delay require for transfer to complete
		usDelay(300 + (cnt << 4));

		regaddr = MPU9250_AG_EXT_SENS_DATA_00;

		cnt = g_pSpi->Read(0, &regaddr, 1, pBuff, cnt);
		if (cnt <=0)
			break;

		pBuff += cnt;
		BuffLen -= cnt;
		retval += cnt;
	}

	return retval;
}

int Mpu9250AuxWrite(uint8_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	uint8_t regaddr;
	uint8_t d[8];

	d[0] = MPU9250_AG_I2C_SLV0_ADDR;
	d[1] = DevAddr;
	d[2] = *pCmdAddr;
	d[3] = MPU9250_AG_I2C_SLV0_CTRL_I2C_SLV0_EN;

	while (DataLen > 0)
	{
		regaddr = MPU9250_AG_I2C_SLV0_DO;
		g_pSpi->Write(0, &regaddr, 1, pData, 1);

		g_pSpi->Write(0, d, 4, NULL, 0);

		d[2]++;
		pData++;
		DataLen--;
		retval++;
	}

	return retval;
}

/**@brief Function for writing to a MPU-9250 register.
 *
 * @param[in] slave_addr    Slave address on the TWI bus.
 * @param[in] reg_addr      Register address to write.
 * @param[in] length        Length of the data to write.
 * @param[in] p_data        Pointer to the data to write.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const * p_data)
{
	if (slave_addr != MPU9250_I2C_DEV_ADDR0 && slave_addr != MPU9250_I2C_DEV_ADDR1)
	{
		return Mpu9250AuxWrite(slave_addr, &reg_addr, 1, (uint8_t*)p_data, length) <= 0;
	}
/*	else
	{
		reg_addr &= 0x7F;
		return g_pSpi->Write(0, &reg_addr, 1, (uint8_t*)p_data, length) <= 0;
	}*/
	return g_Mpu9250.Write(&reg_addr, 1, (uint8_t*)p_data, length) <= 0;
}

/**@brief Function for reading a MPU-9250 register.
 *
 * @param[in]  slave_addr   Slave address on the TWI bus.
 * @param[in]  reg_addr     Register address to read.
 * @param[in]  length       Length of the data to read.
 * @param[out] p_data       Pointer to where the data should be stored.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * p_data)
{
	if (slave_addr != MPU9250_I2C_DEV_ADDR0 && slave_addr != MPU9250_I2C_DEV_ADDR1)
	{
		return Mpu9250AuxRead(slave_addr, &reg_addr, 1, (uint8_t*)p_data, length) <= 0;
	}
/*	else
	{
		reg_addr |= 0x80;
		return g_pSpi->Read(0, &reg_addr, 1, p_data, length) <= 0;
	}*/
	return g_Mpu9250.Read(&reg_addr, 1, p_data, length) <= 0;
}

/**@brief Function for getting a timestamp in milliseconds.
 *
 * @param[out] p_count      Pointer to the timestamp.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_ms_get(unsigned long * p_count)
{
	*p_count = s_pTimer->uSecond() / 1000;

	return 0;
}

/**@brief Function for enabling and registering the MPU-9250 interrupt callback.
 *
 * @param[in] p_int_param   Pointer to the interrupt parameter structure.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_int_register(struct int_param_s * p_int_param)
{
	printf("drv_mpu9250_int_register\r\n");
	IOPinConfig(BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP,
			IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL);
	IOPinEnableInterrupt(BLUEIO_TAG_EVIM_IMU_INT_NO, 6, BLUEIO_TAG_EVIM_IMU_INT_PORT,
						 BLUEIO_TAG_EVIM_IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION,
						 MPU9250IntHandler);

	return 0;
}



