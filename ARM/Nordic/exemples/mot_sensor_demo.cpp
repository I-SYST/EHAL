/**-------------------------------------------------------------------------
@file	mot_sensor_demo.cpp

@brief	Motion sensor demo

	This application demo shows UART Rx/Tx over BLE custom service using EHAL library.

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

/*
#include "Devices/Drivers/Icm20948/Icm20948.h"
//#include "Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Devices/SensorTypes.h"
#include "Devices/SensorConfig.h"
#include "EmbUtils/InvScheduler.h"
#include "EmbUtils/RingByteBuffer.h"
#include "EmbUtils/Message.h"
#include "EmbUtils/ErrorHelper.h"
#include "EmbUtils/DataConverter.h"
#include "EmbUtils/RingBuffer.h"
#include "DynamicProtocol/DynProtocol.h"
#include "DynamicProtocol/DynProtocolTransportUart.h"
#include "Icm20948.h"
#include "Icm20948Defs.h"
#include "Icm20948DataBaseDriver.h"

#include "Icm20948AuxTransport.h"
*/

#include "idelay.h"
#include "coredev/spi.h"
#include "coredev/iopincfg.h"
#include "timer_nrf5x.h"
#include "sensors/agm_icm20948.h"
#include "sensors/agm_invn_icm20948.h"
#include "sensors/agm_mpu9250.h"
#include "sensors/agm_lsm9ds1.h"
#include "sensors/ag_bmi160.h"
#include "imu/imu_invn_icm20948.h"
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
    {SPI2_SCK_PORT, SPI2_SCK_PIN, SPI2_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI2_MISO_PORT, SPI2_MISO_PIN, SPI2_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI2_MOSI_PORT, SPI2_MOSI_PIN, SPI2_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {BLUEIO_TAG_EVIM_IMU_CS_PORT, BLUEIO_TAG_EVIM_IMU_CS_PIN, BLUEIO_TAG_EVIM_IMU_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const SPICFG s_SpiCfg = {
    SPI2_DEVNO,
    SPIMODE_MASTER,
    s_SpiPins,
    sizeof(s_SpiPins) / sizeof(IOPINCFG),
    4000000,   // Speed in Hz
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
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 1
	{BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, // IMU int pin
};

static const int s_NbGpioPins = sizeof(s_GpioPins) / sizeof(IOPINCFG);

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 7,
	.EvtHandler = NULL
};

TimerLFnRF5x g_Timer;

static const ACCELSENSOR_CFG s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_SINGLE,
	.Scale = 2,
	.Freq = 50,
};

static const GYROSENSOR_CFG s_GyroCfg = {
	.DevAddr = 0,
	.Sensitivity = 10,
};

static const MAGSENSOR_CFG s_MagCfg = {
	.DevAddr = 0,
	.Precision = 10,
};

void ImuEvtHandler(Device * const pDev, DEV_EVT Evt);

static const IMU_CFG s_ImuCfg = {
	.EvtHandler = ImuEvtHandler
};

#if 0
ImuInvnIcm20948 g_Imu;
AgmInvnIcm20948 g_MotSensor;
#elif 0
AgmIcm20948 g_MotSensor;
#elif 1
ImuMpu9250 g_Imu;
AgmMpu9250 g_MotSensor;
#else
AgmLsm9ds1 g_MotSensor;
#endif

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
		//g_Imu.IntHandler();
		g_MotSensor.IntHandler();
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
#if 0
		res = g_Imu.Init(s_ImuCfg, 0, &g_Spi, &g_Timer);
#else
		res = g_MotSensor.Init(s_AccelCfg, &g_Spi);
		if (res == true)
		{
			res |= g_MotSensor.Init(s_GyroCfg, &g_Spi);
			if (res == true)
			{
				res |= g_MotSensor.Init(s_MagCfg, &g_Spi);
			}
		}
		if (res == true)
		{
			//res |= g_Imu.Init(s_ImuCfg, &g_MotSensor, &g_MotSensor, &g_MotSensor);
		}
#endif
	}

	if (res == true)
	{
		IOPinCfg(s_GpioPins, s_NbGpioPins);
		IOPinEnableInterrupt(0, 6, BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, IOPINSENSE_HIGH_TRANSITION, ImuIntHandler);

		int8_t m[9] = { 1, 0, 0,
						0, 1, 0,
						0, 0, 1 };

		//g_Imu.RotationMatrix(m);
	}

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

	//g_MotSensor.Enable();
	//g_Imu.Enable();
//	g_MotSensor.Disable();
//	g_Spi.Disable();
//	g_Spi.PowerOff();
	ACCELSENSOR_RAWDATA rawdata;
	ACCELSENSOR_DATA accdata;
	GYROSENSOR_RAWDATA grawdata;
	GYROSENSOR_DATA gyrodata;

	memset(&rawdata, 0, sizeof(ACCELSENSOR_RAWDATA));
	memset(&accdata, 0, sizeof(ACCELSENSOR_DATA));
	memset(&gyrodata, 0, sizeof(GYROSENSOR_DATA));

	while (1)
	{
		//NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
		__WFE();
		g_MotSensor.Read(grawdata);
		//g_Imu.Read(accdata);
		printf("Accel %d: %d %d %d\r\n", grawdata.Timestamp, grawdata.X, grawdata.Y, grawdata.Z);
	}
}
