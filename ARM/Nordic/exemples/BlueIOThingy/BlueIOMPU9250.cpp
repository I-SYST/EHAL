/*
 * BLueIOMPU9250.h
 *
 *  Created on: Jul 25, 2018
 *      Author: hoan
 */

#include "ble_app.h"
#include "ble_service.h"
#include "device_intrf.h"
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "BlueIOMPU9250.h"
#include "sensors/agm_mpu9250.h"
#include "imu/imu_mpu9250.h"
#include "idelay.h"
#include "board.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "BlueIOThingy.h"
#include "eMPL/packet.h"

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

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const IMU_CFG s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

static ImuMpu9250 g_Imu;

static Timer * s_pTimer;
static uint32_t s_MotionFeature = 0;

#if 1
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

int8_t g_AlignMatrix[9] = {
	0,  1,  0,
    -1,  0,  0,
    0,  0, -1
};

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

extern "C" void mpulib_init();
void ImuRawDataSend(ACCELSENSOR_DATA &AccData, GYROSENSOR_DATA GyroData, MAGSENSOR_DATA &MagData);
void ImuQuatDataSend(long Quat[4]);

#define eMPL_send_quat	ImuQuatDataSend
void eMPL_send_data(unsigned char type, long *data)
{

}

/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
       /* Sends a quaternion packet to the PC. Since this is used by the Python
        * test app to visually represent a 3D quaternion, it's sent each time
        * the MPL has new data.
        */
        eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
        if (hal.report & PRINT_QUAT)
            eMPL_send_data(PACKET_DATA_QUAT, data);
    }

    if (hal.report & PRINT_ACCEL) {
        if (inv_get_sensor_type_accel(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    if (hal.report & PRINT_GYRO) {
        if (inv_get_sensor_type_gyro(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_GYRO, data);
    }
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS) {
        if (inv_get_sensor_type_compass(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_COMPASS, data);
    }
#endif
    if (hal.report & PRINT_EULER) {
        if (inv_get_sensor_type_euler(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_EULER, data);
    }
    if (hal.report & PRINT_ROT_MAT) {
        if (inv_get_sensor_type_rot_mat(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ROT, data);
    }
    if (hal.report & PRINT_HEADING) {
        if (inv_get_sensor_type_heading(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_HEADING, data);
    }
    if (hal.report & PRINT_LINEAR_ACCEL) {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
        	MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
        			float_data[0], float_data[1], float_data[2]);
         }
    }
    if (hal.report & PRINT_GRAVITY_VECTOR) {
            if (inv_get_sensor_type_gravity(float_data, &accuracy,
                (inv_time_t*)&timestamp))
            	MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
            			float_data[0], float_data[1], float_data[2]);
    }
    if (hal.report & PRINT_PEDO) {
        unsigned long timestamp;
        drv_mpu9250_ms_get(&timestamp);
        if (timestamp > hal.next_pedo_ms) {
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            unsigned long step_count, walk_time;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);
            MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
            walk_time);
        }
    }

    /* Whenever the MPL detects a change in motion state, the application can
     * be notified. For this example, we use an LED to represent the current
     * motion state.
     */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
            INV_MSG_NO_MOTION_EVENT);
    if (msg) {
        if (msg & INV_MSG_MOTION_EVENT) {
            MPL_LOGI("Motion!\n");
        } else if (msg & INV_MSG_NO_MOTION_EVENT) {
            MPL_LOGI("No motion!\n");
        }
    }
}

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{
	IMU_QUAT iquat;
	int32_t q[4];

	g_Imu.IntHandler();
	g_Imu.Read(iquat);

	q[0] = iquat.Q1 * (1<<30);
	q[1] = iquat.Q2 * (1<<30);
	q[2] = iquat.Q3 * (1<<30);
	q[3] = iquat.Q4 * (1<<30);
	ImuQuatDataSend(q);
}


void MPU9250IntHandler(int IntNo)
{
	IMU_QUAT iquat;
	int32_t q[4];

	g_Imu.IntHandler();
	g_Imu.Read(iquat);

	q[0] = (int32_t)(iquat.Q1 * (float)(1<<30));
	q[1] = (int32_t)(iquat.Q2 * (float)(1<<30));
	q[2] = (int32_t)(iquat.Q3 * (float)(1<<30));
	q[3] = (int32_t)(iquat.Q4 * (float)(1<<30));
	ImuQuatDataSend(q);

	return;

    unsigned char accel_fsr,  new_temp = 0;
    unsigned long sensor_timestamp;
    unsigned char new_compass = 0;
    int new_data = 0;
    hal.new_gyro = 1;
    if (hal.new_gyro && hal.lp_accel_mode) {
         short accel_short[3];
         long accel[3];
         mpu_get_accel_reg(accel_short, &sensor_timestamp);
         accel[0] = (long)accel_short[0];
         accel[1] = (long)accel_short[1];
         accel[2] = (long)accel_short[2];
         inv_build_accel(accel, 0, sensor_timestamp);
         new_data = 1;
         hal.new_gyro = 0;
     } else if (hal.new_gyro && hal.dmp_on) {
         short gyro[3], accel_short[3], sensors;
         unsigned char more;
         long accel[3], quat[4], temperature;
         /* This function gets new data from the FIFO when the DMP is in
          * use. The FIFO can contain any combination of gyro, accel,
          * quaternion, and gesture data. The sensors parameter tells the
          * caller which data fields were actually populated with new data.
          * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
          * the FIFO isn't being filled with accel data.
          * The driver parses the gesture data to determine if a gesture
          * event has occurred; on an event, the application will be notified
          * via a callback (assuming that a callback function was properly
          * registered). The more parameter is non-zero if there are
          * leftover packets in the FIFO.
          */
         dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
         if (!more)
             hal.new_gyro = 0;
         if (sensors & INV_XYZ_GYRO) {
             /* Push the new data to the MPL. */
             inv_build_gyro(gyro, sensor_timestamp);
             new_data = 1;
             if (new_temp) {
                 new_temp = 0;
                 /* Temperature only used for gyro temp comp. */
                 mpu_get_temperature(&temperature, &sensor_timestamp);
                 inv_build_temp(temperature, sensor_timestamp);
             }
         }
         if (sensors & INV_XYZ_ACCEL) {
             accel[0] = (long)accel_short[0];
             accel[1] = (long)accel_short[1];
             accel[2] = (long)accel_short[2];
             inv_build_accel(accel, 0, sensor_timestamp);
             new_data = 1;
         }
         if (sensors & INV_WXYZ_QUAT) {
             inv_build_quat(quat, 0, sensor_timestamp);
             new_data = 1;
         }
     } else if (hal.new_gyro) {
         short gyro[3], accel_short[3];
         unsigned char sensors, more;
         long accel[3], temperature;
         /* This function gets new data from the FIFO. The FIFO can contain
          * gyro, accel, both, or neither. The sensors parameter tells the
          * caller which data fields were actually populated with new data.
          * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
          * being filled with accel data. The more parameter is non-zero if
          * there are leftover packets in the FIFO. The HAL can use this
          * information to increase the frequency at which this function is
          * called.
          */
         hal.new_gyro = 0;
         mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
             &sensors, &more);
         if (more)
             hal.new_gyro = 1;
         if (sensors & INV_XYZ_GYRO) {
             /* Push the new data to the MPL. */
             inv_build_gyro(gyro, sensor_timestamp);
             new_data = 1;
             if (new_temp) {
                 new_temp = 0;
                 /* Temperature only used for gyro temp comp. */
                 mpu_get_temperature(&temperature, &sensor_timestamp);
                 inv_build_temp(temperature, sensor_timestamp);
             }
         }
         if (sensors & INV_XYZ_ACCEL) {
             accel[0] = (long)accel_short[0];
             accel[1] = (long)accel_short[1];
             accel[2] = (long)accel_short[2];
             inv_build_accel(accel, 0, sensor_timestamp);
             new_data = 1;
         }
     }
#ifdef COMPASS_ENABLED
     if (new_compass) {
         short compass_short[3];
         long compass[3];
         new_compass = 0;
         /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
          * magnetometer registers are copied to special gyro registers.
          */
         if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
             compass[0] = (long)compass_short[0];
             compass[1] = (long)compass_short[1];
             compass[2] = (long)compass_short[2];
             /* NOTE: If using a third-party compass calibration library,
              * pass in the compass data in uT * 2^16 and set the second
              * parameter to INV_CALIBRATED | acc, where acc is the
              * accuracy from 0 to 3.
              */
             inv_build_compass(compass, 0, sensor_timestamp);
         }
         new_data = 1;
     }
#endif
     if (new_data) {
         inv_execute_on_data();
         /* This function reads bias-compensated sensor data and sensor
          * fusion outputs from the MPL. The outputs are formatted as seen
          * in eMPL_outputs.c. This function only needs to be called at the
          * rate requested by the host.
          */
         read_from_mpl();
     }

	return;

	ACCELSENSOR_DATA accdata;
	GYROSENSOR_DATA gyrodata;
	MAGSENSOR_DATA magdata;
	long l[3];
	short s[3];
	long quat[4];

    short gyro[3], accel_short[3], sensors;
    unsigned char more;

    //unsigned long sensor_timestamp;
	//dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
   // inv_build_gyro(gyro, sensor_timestamp);

    l[0] = accel_short[0];
	l[1] = accel_short[1];
	l[2] = accel_short[2];


	g_Mpu9250.IntHandler();

	g_Mpu9250.Read(accdata);

	l[0] = accdata.X / 256;
	l[1] = accdata.Y / 256;
	l[2] = accdata.Z / 256;
	inv_build_accel(l, 0, accdata.Timestamp);

	g_Mpu9250.Read(gyrodata);

	s[0] = gyrodata.X / 256;
	s[1] = gyrodata.Y / 256;
	s[2] = gyrodata.Z / 256;
	inv_build_gyro(s, accdata.Timestamp);

	g_Mpu9250.Read(magdata);

	l[0] = accdata.X;
	l[1] = accdata.Y;
	l[2] = accdata.Z;

	inv_build_compass(l, 0, accdata.Timestamp);

	ImuRawDataSend(accdata, gyrodata, magdata);

	inv_build_quat(quat, 0, accdata.Timestamp);

	inv_execute_on_data();
    inv_time_t       timestamp;
    int8_t           accuracy;

	inv_get_sensor_type_quat((long *)quat, &accuracy, &timestamp);
//	read_from_mpl();
//	long data[9];
//	int8_t accuracy;
//	inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&sensor_timestamp);

	ImuQuatDataSend(quat);

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


	g_Mpu9250.Init(s_AccelCfg, pIntrF, pTimer);
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

    g_Imu.Init(s_ImuCfg, &g_Mpu9250, &g_Mpu9250, &g_Mpu9250);
    g_Imu.SetAxisAlignmentMatrix(g_AlignMatrix);
    g_Imu.Quaternion(true, 6);
    //g_Imu.Compass(true);

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



