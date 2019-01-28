/*
 * BlueIOMPU9250.h
 *
 *  Created on: Jul 25, 2018
 *      Author: hoan
 */

#ifndef __BLUEIOMPU9250_H__
#define __BLUEIOMPU9250_H__

#define MPU9250_MOTION_FEATURE_RAW				(1<<0)
#define MPU9250_MOTION_FEATURE_QUAT				(1<<1)
#define MPU9250_MOTION_FEATURE_EULER			(1<<2)
#define MPU9250_MOTION_FEATURE_ROT_MAT			(1<<3)
#define MPU9250_MOTION_FEATURE_HEADING			(1<<4)
#define MPU9250_MOTION_FEATURE_GRAVITY_VECTOR	(1<<5)
#define MPU9250_MOTION_FEATURE_TAP				(1<<6)
#define MPU9250_MOTION_FEATURE_ORIENTATION		(1<<7)
#define MPU9250_MOTION_FEATURE_PEDOMETER		(1<<8)
#define MPU9250_MOTION_FEATURE_WAKE_ON_MOTION	(1<<9)


#ifdef __cplusplus

#include "device_intrf.h"
#include "coredev/timer.h"

bool MPU9250Init(DeviceIntrf * const pIntrF, Timer * const pTimer);

extern "C" {
#endif

void MPU9250EnableFeature(uint32_t Feature);

/**@brief Function for writing to a MPU-9250 register.
 *
 * @param[in] slave_addr    Slave address on the TWI bus.
 * @param[in] reg_addr      Register address to write.
 * @param[in] length        Length of the data to write.
 * @param[in] p_data        Pointer to the data to write.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const * p_data);

/**@brief Function for reading a MPU-9250 register.
 *
 * @param[in]  slave_addr   Slave address on the TWI bus.
 * @param[in]  reg_addr     Register address to read.
 * @param[in]  length       Length of the data to read.
 * @param[out] p_data       Pointer to where the data should be stored.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char * p_data);

/**@brief Function for getting a timestamp in milliseconds.
 *
 * @param[out] p_count      Pointer to the timestamp.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_ms_get(unsigned long * p_count);

/**@brief Function for enabling and registering the MPU-9250 interrupt callback.
 *
 * @param[in] p_int_param   Pointer to the interrupt parameter structure.
 *
 * @retval 0 if success. Else -1.
 */
int drv_mpu9250_int_register(struct int_param_s * p_int_param);

#ifdef __cplusplus
}
#endif

#endif // __BLUEIOMPU9250_H__
