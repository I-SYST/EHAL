/**-------------------------------------------------------------------------
@file	a_adxl362.h

@brief	Implementation of Analog ADXL362 accelerometer

		Interface supported SPI CPHA = CPOL = 0

@author	Hoang Nguyen Hoan
@date	July 13, 2018

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

#ifndef __A_ADXL362_H__
#define __A_ADXL362_H__

#include <inttypes.h>

#include "sensors/accel_sensor.h"

#define ADXL362_DEVID_AD_REG		0x00

#define ADXL362_DEVID_AD							0xAD

#define ADXL362_DEVID_MST_REG		0x01

#define ADXL362_DEVID_MST							0x1D

#define ADXL362_PARTID_REG			0x02

#define ADXL362_PARTID								0xF2

#define ADXL362_REVID_REG			0x03

#define ADXL362_REVID								0x1

#define ADXL362_XDATA_REG			0x08

#define ADXL362_YDATA_REG			0x09

#define ADXL362_ZDATA_REG			0x0A


#define ADXL362_STATUS_REG			0x0B

#define ADXL362_STATUS_DATA_READY					(1<<0)
#define ADXL362_STATUS_FIFO_READY					(1<<1)
#define ADXL362_STATUS_FIFO_WATERMARK				(1<<2)
#define ADXL362_STATUS_FIFO_OVER_RUN				(1<<3)
#define ADXL362_STATUS_ACT							(1<<4)
#define ADXL362_STATUS_INACT						(1<<5)
#define ADXL362_STATUS_AWAKE						(1<<6)
#define ADXL362_STATUS_ERR_USER_REGS				(1<<7)

#define ADXL362_FIFO_ENTRIES_L_REG	0x0C

#define ADXL362_FIFO_ENTRIES_H_REG	0x0D

#define ADXL362_FIFO_ENTRIES_H_MASK					(3)

#define ADXL362_XDATA_L_REG			0x0E

#define ADXL362_XDATA_H_REG			0x0F

#define ADXL362_YDATA_L_REG			0x10

#define ADXL362_YDATA_H_REG			0x11

#define ADXL362_ZDATA_L_REG			0x12

#define ADXL362_ZDATA_H_REG			0x13

#define ADXL362_TEMP_L_REG			0x14

#define ADXL362_TEMP_H_REG			0x15

#define ADXL362_SOFT_RESET_REG		0x1F

#define ADXL362_SOFT_RESET							(0x52)

#define ADXL362_THRESH_ACT_L_REG	0x20

#define ADXL362_THRESH_ACT_H_REG	0x21

#define ADXL362_THRESH_ACT_H_MASK					(7)

#define ADXL362_TIME_ACT_REG		0x22

#define ADXL362_THRESH_INACT_L_REG	0x23

#define ADXL362_THRESH_INACT_H_REG	0x24

#define ADXL362_THRESH_INACT_H_MASK					(7)

#define ADXL362_TIME_INACT_L_REG	0x25

#define ADXL362_TIME_INACT_H_REG	0x26

#define ADXL362_ACT_INACT_CTL_REG	0x27

#define ADXL362_ACT_INACT_CTL_ACT_EN				(1<<0)
#define ADXL362_ACT_INACT_CTL_ACT_REF				(1<<1)
#define ADXL362_ACT_INACT_CTL_INACT_EN				(1<<2)
#define ADXL362_ACT_INACT_CTL_INACT_REF				(1<<3)
#define ADXL362_ACT_INACT_CTL_LINKLOOP_MASK			(3<<4)
#define ADXL362_ACT_INACT_CTL_LINKLOOP_BITPOS		(4)
#define ADXL362_ACT_INACT_CTL_RES_MASK				(3<<6)
#define ADXL362_ACT_INACT_CTL_RES_BITPOST			(6)

#define ADXL362_FIFO_CONTROL_REG	0x28

#define ADXL362_FIFO_CONTROL_MASK					(0xF)
#define ADXL362_FIFO_CONTROL_FIFO_MODE_MASK			(3)
#define ADXL362_FIFO_CONTROL_FIFO_MODE_BITPOS		(0)
#define ADXL362_FIFO_CONTROL_FIFO_TEMP				(1<<2)
#define ADXL362_FIFO_CONTROL_AH						(1<<3)

#define ADXL362_FIFO_SAMPLES_REG	0x29

#define ADXL362_INTMAP1_REG			0x2A

#define ADXL362_INTMAP1_DATA_READY					(1<<0)
#define ADXL362_INTMAP1_FIFO_READY					(1<<1)
#define ADXL362_INTMAP1_FIFO_WATERMARK				(1<<2)
#define ADXL362_INTMAP1_FIFO_OVERRUN				(1<<3)
#define ADXL362_INTMAP1_ACT							(1<<4)
#define ADXL362_INTMAP1_INACT						(1<<5)
#define ADXL362_INTMAP1_AWAKE						(1<<6)
#define ADXL362_INTMAP1_INT_LOW						(1<<7)

#define ADXL362_INTMAP2_REG			0x2B

#define ADXL362_INTMAP2_DATA_READY					(1<<0)
#define ADXL362_INTMAP2_FIFO_READY					(1<<1)
#define ADXL362_INTMAP2_FIFO_WATERMARK				(1<<2)
#define ADXL362_INTMAP2_FIFO_OVERRUN				(1<<3)
#define ADXL362_INTMAP2_ACT							(1<<4)
#define ADXL362_INTMAP2_INACT						(1<<5)
#define ADXL362_INTMAP2_AWAKE						(1<<6)
#define ADXL362_INTMAP2_INT_LOW						(1<<7)

#define ADXL362_FILTER_CTL_REG		0x2C

#define ADXL362_FILTER_CTL_ODR_MASK					(0x7)
#define ADXL362_FILTER_CTL_ODR_BITPOS				(0)
#define ADXL362_FILTER_CTL_ODR_12_5HZ				(0)
#define ADXL362_FILTER_CTL_ODR_25HZ					(1<<0)
#define ADXL362_FILTER_CTL_ODR_50HZ					(2<<0)
#define ADXL362_FILTER_CTL_ODR_100HZ				(3<<0)
#define ADXL362_FILTER_CTL_ODR_200HZ				(4<<0)
#define ADXL362_FILTER_CTL_ODR_400HZ				(5<<0)

#define ADXL362_FILTER_CTL_EXT_SAMPLE				(1<<3)
#define ADXL362_FILTER_CTL_HALF_BW					(1<<4)
#define ADXL362_FILTER_CTL_RES						(1<<5)
#define ADXL362_FILTER_CTL_RANGE_MASK				(3<<6)
#define ADXL362_FILTER_CTL_RANGE_BITPOS				(6)
#define ADXL362_FILTER_CTL_RANGE_2G					(0)
#define ADXL362_FILTER_CTL_RANGE_4G					(1<<6)
#define ADXL362_FILTER_CTL_RANGE_8G					(1<<7)

#define ADXL362_POWER_CTL_REG		0x2D

#define ADXL362_POWER_CTL_MEASURE_MASK				(3)
#define ADXL362_POWER_CTL_MEASURE_BITPOS			(0)
#define ADXL362_POWER_CTL_MEASURE_STANDBY			(0)
#define ADXL362_POWER_CTL_MEASURE_START				(2)

#define ADXL362_POWER_CTL_AUTOSLEEP					(1<<2)

#define ADXL362_POWER_CTL_WAKEUP					(1<<3)
#define ADXL362_POWER_CTL_LOW_NOISE_MASK			(3<<4)
#define ADXL362_POWER_CTL_LOW_NOISE_BITPOS			(4)
#define ADXL362_POWER_CTL_LOW_NOISE_NORMAL			(0)
#define ADXL362_POWER_CTL_LOW_NOISE_LOW_NOISE		(1<<4)
#define ADXL362_POWER_CTL_LOW_NOISE_ULTRA_LOW_NOISE	(2<<4)

#define ADXL362_POWER_CTL_EXT_CLK					(1<<6)

#define ADXL362_SELF_TEST_REG		0x2E

#define ADXL362_SELF_TEST_ST						(1<<0)

#define ADXL362_CMD_READ			0x0B
#define ADXL362_CMD_WRITE			0x0A
#define ADXL362_CMD_READFIFO		0x0D

#define ADXL362_DEVID	((ADXL362_DEVID_MST << 8) | ADXL362_DEVID_AD)

class AccelAdxl362 : public AccelSensor {
public:
	virtual bool Init(const ACCELSENSOR_CFG &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = NULL);
	virtual bool Read(ACCELSENSOR_DATA &Data);
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool StartSampling();
	virtual uint16_t Scale(uint16_t Value);
	virtual bool WakeOnEvent(bool bEnable, int Threshold);

private:
	bool UpdateData();

	bool vbInitialized;
};


#endif //__A_ADXL362_H__

