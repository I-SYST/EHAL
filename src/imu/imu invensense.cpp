/**-------------------------------------------------------------------------
@file	imu.cpp

@brief	Implementation of an Inertial Measurement Unit

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

#include "accel_auto_cal.h"
#include "quaternion_supervisor.h"
#include "fusion_9axis.h"
#include "fast_no_motion.h"
#include "compass_vec_cal.h"
#include "gyro_tc.h"
#include "heading_from_gyro.h"
#include "inv_math.h"
#include "invensense_adv.h"
#include "mag_disturb.h"
#include "motion_no_motion.h"
#include "no_gyro_fusion.h"
#include "storage_manager.h"
#include "start_manager.h"
#include "data_builder.h"
#include "results_holder.h"
#include "mlinclude.h"
#include "mpl.h"
#include "log.h"

#include "imu/imu_invensense.h"
#include "sensors/agm_mpu9250.h"

bool ImuInvenSense::Init(AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	inv_error_t err;

	if (pAccel == NULL)
	{
		return false;
	}

	vpMpu = (AgmMpu9250 *)pAccel;

    inv_init_storage_manager();

    /* initialize the start callback manager */
    err = inv_init_start_manager();

    /* initialize the data builder */
    err = inv_init_data_builder();

    err = inv_enable_results_holder();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test. */
    err = inv_enable_in_use_auto_calibration();

    /* Compute 6-axis and 9-axis quaternions. */
    err = inv_enable_quaternion();

    err = inv_enable_9x_sensor_fusion();

    /* Update gyro biases when not in motion. */
    err = inv_enable_fast_nomot();

    /* Update gyro biases when temperature changes. */
    err = inv_enable_gyro_tc();

    /* Compass calibration algorithms. */
    err = inv_enable_vector_compass_cal();

    err = inv_enable_magnetic_disturbance();

    //err = inv_enable_eMPL_outputs();

  //  err = inv_enable_heading_from_gyro();
    //RETURN_IF_ERROR(err_code);


    return Imu::Init(pAccel, pGyro, pMag);
}

bool ImuInvenSense::UpdateData()
{

	return true;
}




