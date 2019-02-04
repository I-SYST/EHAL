/**-------------------------------------------------------------------------
@file	bleadv_mandata.h

@brief	Contains definitions for data types to be advertised as manufacturer specific data


@author	Hoang Nguyen Hoan
@date	Nov 11, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#ifndef __BLEADV_MANDATA_H__
#define	__BLEADV_MANDATA_H__

#include "blueio_types.h"

/** @addtogroup Bluetooth
  * @{
  */

/// Manufacture specific advertisement data type
#define BLEADV_MANDATA_TYPE_SN			0xFF						//!< Device Serial Number or UID (8 bytes)
#define BLEADV_MANDATA_TYPE_TPH			BLUEIO_DATA_TYPE_TPH		//!< Environmental sensor data (Temperature, Pressure, Humidity)
#define BLEADV_MANDATA_TYPE_GAS			BLUEIO_DATA_TYPE_GAS		//!< Gas sensor data
#define BLEADV_MANDATA_TYPE_ACCEL		BLUEIO_DATA_TYPE_ACCEL		//!< Accelerometer sensor data
#define BLEADV_MANDATA_TYPE_GYRO		BLUEIO_DATA_TYPE_GYRO		//!< Gyroscope sensor data
#define BLEADV_MANDATA_TYPE_MAG			BLUEIO_DATA_TYPE_MAG		//!< Magnetometer sensor data
#define BLEADV_MANDATA_TYPE_PROXY		BLUEIO_DATA_TYPE_PROXY		//!< Proximity sensor data
#define BLEADV_MANDATA_TYPE_ADC			BLUEIO_DATA_TYPE_ADC		//!< Analog converter data
#define BLEADV_MANDATA_TYPE_GPIO		BLUEIO_DATA_TYPE_GPIO		//!< GPIO pins state
#define BLEADV_MANDATA_TYPE_BUT			BLUEIO_DATA_TYPE_BUT		//!< Button/Switch state
#define BLEADV_MANDATA_TYPE_MOT			BLUEIO_DATA_TYPE_MOT		//!< Motion detection
#define BLEADV_MANDATA_TYPE_AUDIO		BLUEIO_DATA_TYPE_AUDIO		//!< Audio data
#define BLEADV_MANDATA_TYPE_BAT			BLUEIO_DATA_TYPE_BAT		//!< Battery level data

#define BLEADV_MANDATA_LEN_MAX			8		//!< Max number of bytes of data

#pragma pack(push, 1)

/// Generic manufacture specific data format in advertisement
typedef struct __BleApp_Advertising_ManData {
	uint8_t Type;							//!< Data types (see defined code above)
	uint8_t Data[BLEADV_MANDATA_LEN_MAX];	//!< Type specific data follows can be more than 1 bytes
} BLEADV_MANDATA;

/// Environmental sensor data
typedef struct __EnvTPHData {
	uint32_t Pressure;		//!< Barometric pressure in Pa no decimal
	int16_t  Temperature;	//!< Temperature in degree C, 2 decimals fixed point
	uint16_t Humidity;		//!< Relative humidity in %, 2 decimals fixed point
} BLEADV_MANDATA_TPHSENSOR;

/// Gas sensor data
typedef struct __EnvGasData {
	uint32_t GasRes;		//!< Gas resistance
	uint16_t AirQIdx;		//!< Air quality index
} BLEADV_MANDATA_GASSENSOR;

/// IMU data
typedef struct __AGM_Raw_Data {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} BLEADV_MANDATA_IMUSENSOR;

/// I/O pin state
typedef struct __GPIO_Pin_State {
	uint32_t State;		//!< Bit field pins state
} BLEADV_MANDATA_GPIO;

/// Button/Switch state
typedef struct __ButSwitch_State {
	uint32_t State;		//!< Bit field state state 0 = close, 1 = open
} BLEADV_MANDATA_BUT;

#pragma pack(pop)

/** @} End of group Bluetooth */

#endif // __BLEADV_MANDATA_H__
