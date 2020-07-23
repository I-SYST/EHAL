/**-------------------------------------------------------------------------
@example	mot_sensor_demo.cpp


@brief	Motion sensor demo

	This application demo shows how to use Motion sensors.

@author	Hoang Nguyen Hoan
@date	Dec. 21, 2018

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
#include <stdio.h>
#include <atomic>

#include "nrf.h"

#include "idelay.h"
#include "coredev/spi.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "timer_nrfx.h"
#include "sensors/agm_icm20948.h"
#include "sensors/agm_invn_icm20948.h"
#include "sensors/agm_mpu9250.h"
#include "sensors/agm_lsm9ds1.h"
#include "sensors/ag_bmi160.h"
#include "sensors/accel_h3lis331dl.h"
#include "imu/imu_invn_icm20948.h"
#include "imu/imu_icm20948.h"
#include "imu/imu_mpu9250.h"

#include "board.h"

std::atomic<bool> g_bTest(false);

//#include "sensor.h"
//#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
//#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

//static const uint8_t dmp3_image[] = {
//#include "imu/icm20948_img.dmp3a.h"
//};

//int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
//int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
//inv_bool_t interface_is_SPI(void);

static const IOPINCFG s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#ifdef NEBLINA
	{NEBLINA_SPI_BMI160_CS_PORT, NEBLINA_SPI_BMI160_CS_PIN, NEBLINA_SPI_BMI160_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{NEBLINA_SPI_H3LIS_CS_PORT, NEBLINA_SPI_H3LIS_CS_PIN, NEBLINA_SPI_H3LIS_CS_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#else
	{BLUEIO_TAG_EVIM_IMU_CS_PORT, BLUEIO_TAG_EVIM_IMU_CS_PIN, BLUEIO_TAG_EVIM_IMU_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#endif
};

static const SPICFG s_SpiCfg = {
    SPI_DEVNO,
	SPIPHY_NORMAL,
    SPIMODE_MASTER,
    s_SpiPins,
    sizeof(s_SpiPins) / sizeof(IOPINCFG),
    2000000,   // Speed in Hz
    8,      // Data Size
    5,      // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,         // clock polarity
    SPICSEL_AUTO,
	true,
	false,
    6,      // Interrupt priority
    NULL
};

SPI g_Spi;

static const IOPINCFG s_GpioPins[] = {
	{BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, BLUEIO_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 1
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 2
	{BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, // IMU int pin
	{0, 24, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// But 2
};

static const int s_NbGpioPins = sizeof(s_GpioPins) / sizeof(IOPINCFG);

void TimerHandler(Timer *pTimer, uint32_t Evt);

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 1,
	.EvtHandler = TimerHandler,
};

TimerLFnRFx g_Timer;

static const ACCELSENSOR_CFG s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 1000,
	.Scale = 2,
	.FltrFreq = 0,
	.bInter = true,
	.IntPol = DEVINTR_POL_LOW,
};

static const GYROSENSOR_CFG s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 10,
	.FltrFreq = 200,
};

static const MAGSENSOR_CFG s_MagCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,//SENSOR_OPMODE_SINGLE,
	.Freq = 50000,
	.Precision = MAGSENSOR_PRECISION_HIGH,
};

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const IMU_CFG s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

//#define ICM20948
#define MPU9250
//#define BMI160
//#define H3LIS331DL

#ifdef ICM20948
ImuIcm20948 g_Imu;
AgmIcm20948 g_MotSensor;
#elif 0
AgmIcm20948 g_MotSensor;
#elif defined(MPU9250)
ImuMpu9250 g_Imu;
AgmMpu9250 g_MotSensor;
#elif defined(BMI160)
AgBmi160 g_MotSensor;
#elif defined(H3LIS331DL)
AccelH3lis331dl g_MotSensor;
#else
AgmLsm9ds1 g_MotSensor;
#endif

AccelSensor *g_pAccel = NULL;
GyroSensor *g_pGyro = NULL;
MagSensor *g_pMag = NULL;

uint32_t g_DT = 0;
static uint32_t g_TPrev = 0;

void TimerHandler(Timer *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    }
}

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt)
{
	ACCELSENSOR_DATA accdata;
	IMU_QUAT quat;

	switch (Evt)
	{
		case DEV_EVT_DATA_RDY:
			g_MotSensor.Read(accdata);
			//g_Imu.Read(accdata);
			printf("Accel %d: %d %d %d\r\n", accdata.Timestamp, accdata.X, accdata.Y, accdata.Z);
			//g_Imu.Read(quat);
			//printf("Quat %d: %d %d %d %d\r\n", quat.Timestamp, quat.Q1, quat.Q2, quat.Q2, quat.Q3);

			break;
	}
}


void ImuIntHandler(int IntNo)
{
	if (IntNo == 0)
	{
//		IOPinSet(0, 24);
		uint32_t t = g_Timer.uSecond();
		g_DT = t - g_TPrev;
		g_TPrev = t;

		//g_Imu.IntHandler();
		//g_MotSensor.IntHandler();
		//IOPinClear(0, 24);
	}
}

uint64_t inv_icm20948_get_time_us(void)
{
	return g_Timer.uSecond();
}

void inv_icm20948_sleep(int ms)
{
	msDelay(ms);
}

void inv_icm20948_sleep_us(int us)
{
	usDelay(us);
}

bool HardwareInit()
{
	bool res;

	g_Timer.Init(s_TimerCfg);

	res = g_Spi.Init(s_SpiCfg);

	if (res == true)
	{
		res = g_MotSensor.Init(s_AccelCfg, &g_Spi, &g_Timer);
		if (res == true)
		{
			g_pAccel = &g_MotSensor;
		}
#if !defined(H3LIS331DL)
		res = g_MotSensor.Init(s_GyroCfg, &g_Spi);
		if (res == true)
		{
			g_pGyro = &g_MotSensor;
		}

		res = g_MotSensor.Init(s_MagCfg, &g_Spi);
		if (res == true)
		{
			g_pMag = &g_MotSensor;
		}
#endif

#if defined(ICM20948) || defined(MPU9250)
		res = g_Imu.Init(s_ImuCfg, &g_MotSensor, &g_MotSensor, &g_MotSensor);
#endif
	}

	if (res == true)
	{
		//IOPinCfg(s_GpioPins, s_NbGpioPins);
		//IOPinEnableInterrupt(0, 6, BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, IOPINSENSE_LOW_TRANSITION, ImuIntHandler);

		int8_t m[9] = { 1, 0, 0,
						0, 1, 0,
						0, 0, 1 };

		//g_Imu.SetAxisAlignmentMatrix(m);
		//g_Imu.Quaternion(true, 6);
	}

	//uint64_t period = g_Timer.EnableTimerTrigger(0, 1UL, TIMER_TRIG_TYPE_CONTINUOUS);

	//printf("period %u\r\n", (uint32_t)period);

	return res;
}
//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	bool res = HardwareInit();


	if (res == false)
	{
		printf("Failed initializing hardware\r\n");

		return 1;
	}

	printf("MotionSensorDemo\r\n");

	ACCELSENSOR_RAWDATA arawdata;
	ACCELSENSOR_DATA accdata;
	GYROSENSOR_RAWDATA grawdata;
	GYROSENSOR_DATA gyrodata;
	MAGSENSOR_RAWDATA mrawdata;
	IMU_QUAT quat;

	memset(&arawdata, 0, sizeof(ACCELSENSOR_RAWDATA));
	memset(&accdata, 0, sizeof(ACCELSENSOR_DATA));
	memset(&gyrodata, 0, sizeof(GYROSENSOR_DATA));


	uint32_t prevt = 0;
	int cnt = 100;
	while (1)
	{
//		uint32_t t = g_Timer.uSecond();

		//NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
		//__WFE();
		g_MotSensor.UpdateData();

		uint32_t dt = arawdata.Timestamp - prevt;
		prevt = arawdata.Timestamp;

		g_MotSensor.Read(arawdata);

		if (g_pGyro)
		{
			g_pGyro->Read(grawdata);
		}

		if (g_pMag)
		{
			g_pMag->Read(mrawdata);
		}

		//g_Imu.Read(accdata);
		//g_Imu.Read(quat);

		if (cnt-- < 0)
		{
			cnt = 100;
			printf("Accel %d %d: %d %d %d\r\n", (uint32_t)g_DT, (uint32_t)dt, arawdata.X, arawdata.Y, arawdata.Z);
			//printf("Accel %d %d: %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, accdata.X, accdata.Y, accdata.Z);
			//printf("Quat %d %d: %f %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, quat.Q1, quat.Q2, quat.Q3, quat.Q4);
		}
	}
}
