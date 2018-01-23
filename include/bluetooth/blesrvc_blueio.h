/**-------------------------------------------------------------------------
@file	blesrvc_blueio.h

@brief	BLUEIO BLE custom service


@author	Hoang Nguyen Hoan
@date	Dec. 4, 2017

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

#ifndef __BLESRVC_BLUEIO__
#define __BLESRVC_BLUEIO__

#include <inttypes.h>
#include "blueio_types.h"

#define BLUEIOSRVC_DATA_ID_BLECFG			0							//!< BLE configuration settings
#define BLUEIOSRVC_DATA_ID_TPH				BLUEIO_DATA_TYPE_TPH		//!< Environmental sensor data (Temperature, Pressure, Humidity)
#define BLUEIOSRVC_DATA_ID_GAS				BLUEIO_DATA_TYPE_GAS		//!< Gas sensor data
#define BLUEIOSRVC_DATA_ID_ACCEL			BLUEIO_DATA_TYPE_ACCEL		//!< Accelerometer sensor data
#define BLUEIOSRVC_DATA_ID_GYRO				BLUEIO_DATA_TYPE_GYRO		//!< Gyroscope sensor data
#define BLUEIOSRVC_DATA_ID_MAG				BLUEIO_DATA_TYPE_MAG		//!< Magnetometer sensor data
#define BLUEIOSRVC_DATA_ID_PROXY			BLUEIO_DATA_TYPE_PROXY		//!< Proximity sensor data
#define BLUEIOSRVC_DATA_ID_ADC				BLUEIO_DATA_TYPE_ADC		//!< Analog converter data
#define BLUEIOSRVC_DATA_ID_GPIO				BLUEIO_DATA_TYPE_GPIO		//!< GPIO pins data
#define BLUEIOSRVC_DATA_ID_BUT				BLUEIO_DATA_TYPE_BUT		//!< Button state
#define BLUEIOSRVC_DATA_ID_MOT              BLUEIO_DATA_TYPE_MOT		//!< Motion detection
#define BLUEIOSRVC_DATA_ID_AUDIO			BLUEIO_DATA_TYPE_AUDIO		//!< Audio data
#define BLUEIOSRVC_DATA_ID_DFU				0xFF						//!< Re-boot to DFU mode

#define BLUEIOSRVC_DATA_LEN_MAX				48

#pragma pack(push, 1)
typedef struct __BlueioSrvc_Data {
	uint8_t Id;		// Data types (see defined code above)
	uint8_t Data[BLUEIOSRVC_DATA_LEN_MAX];	// Type specific data follows can be more than
} BLUEIO_SRVC_DATA;

#define BLUEIOSRVC_RDCHAR_IDX				0
#define BLUEIOSRVC_WDCHAR_IDX				1

#define DEVICE_NAME_LEN_MAX					10

// BLE configuration

typedef enum {
	APP_MODE_NORMAL,
	APP_MODE_MOTDET		// motion detect
} APP_MODE;

typedef struct __App_Parameter {
	APP_MODE Mode;			// Operating mode 0 - Normal advertising, 1 - Deep sleep Wake on int event
	uint8_t WomThresh;		// Wake on motion threshold
	char DevName[DEVICE_NAME_LEN_MAX]; // zero terminated string device name
	uint32_t AdvTimeout;	// Advertisement timeout in sec for mode 1
} APP_PARAM;


// GPIO
// Write data format

typedef enum __Gpio_Write_Cmd {
	GPIOWR_CMD_RD,		// Read pin
	GPIOWR_CMD_WR,		// Write pin
	GPIOWR_CMD_CONFIG	// Configure pin
} GPIOWR_CMD;

typedef struct __Gpio_Write {
	GPIOWR_CMD 	Cmd;
	uint8_t 	PinNo;
	uint8_t 	Value;
} GPIO_WR;

// Read data format
typedef struct __Gpio_Read {
	uint8_t PinNo;
	uint8_t Value;
} GPIO_RD;

// Button data format

typedef struct __Button_Read {
	uint8_t ButNo;
	uint8_t Value;
} BUT_RD;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

BLESRVC *GetBlueIOSrvcInstance();
bool IsBlueIOSrvcConnected();
bool IsBlueIOSrvcCharNotifyEnabled();

#ifdef __cplusplus
}
#endif

#endif // __BLESRVC_BLUEIO__

