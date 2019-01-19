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

/// Nordic custom UUID
/// UART UUID :
#define NUS_BASE_UUID	{ 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
						  0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E } /**< Used vendor specific UUID. */
#define BLE_UUID_NUS_SERVICE			0x0001	/**< The UUID of the Nordic UART Service. */
#define BLE_UUID_NUS_TX_CHARACTERISTIC	0x0003	/**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_RX_CHARACTERISTIC	0x0002	 /**< The UUID of the RX Characteristic. */

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_NUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_NUS_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#define BLE_NUS_MAX_RX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN	/**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_NUS_MAX_TX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN	/**< Maximum length of the TX Characteristic (in bytes). */


/// Default BlueIO custom UUID.  User should use privately generated UUID
/// UUID : 00000000-287c-11e4-ab74-0002a5d5c51b
#define BLUEIO_UUID_BASE { 	0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x74, 0xab, \
							0xe4, 0x11, 0x7c, 0x28, 0x00, 0x00, 0x00, 0x00 }

/// Default BlueIO control/data service UUID
#define BLUEIO_UUID_SERVICE 		0x1		//!< BlueIO default service
#define BLUEIO_UUID_RDCHAR 			0x2		//!< Data characteristic
#define BLUEIO_UUID_RDCHAR_PROP		(BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY)
#define BLUEIO_UUID_WRCHAR 			0x3		//!< Command control characteristic
#define BLUEIO_UUID_WRCHAR_PROP		(BLESVC_CHAR_PROP_WRITE)

/// Default BLueIO UART service UUID
#define BLUEIO_UUID_UART_SERVICE 		0x101		//!< BlueIO Uart service
#define BLUEIO_UUID_UART_RX_CHAR		0x102		//!< UART Rx characteristic
#define BLUEIO_UUID_UART_TX_CHAR		0x103		//!< UART Tx characteristic

/// BlueIO data type
#define BLUEIO_PACKET_ID_BLECFG			0							//!< BLE configuration settings
#define BLUEIO_PACKET_ID_TPH			BLUEIO_DATA_TYPE_TPH		//!< Environmental sensor data (Temperature, Pressure, Humidity)
#define BLUEIO_PACKET_ID_GAS			BLUEIO_DATA_TYPE_GAS		//!< Gas sensor data
#define BLUEIO_PACKET_ID_ACCEL			BLUEIO_DATA_TYPE_ACCEL		//!< Accelerometer sensor data
#define BLUEIO_PACKET_ID_GYRO			BLUEIO_DATA_TYPE_GYRO		//!< Gyroscope sensor data
#define BLUEIO_PACKET_ID_MAG			BLUEIO_DATA_TYPE_MAG		//!< Magnetometer sensor data
#define BLUEIO_PACKET_ID_PROXY			BLUEIO_DATA_TYPE_PROXY		//!< Proximity sensor data
#define BLUEIO_PACKET_ID_ADC			BLUEIO_DATA_TYPE_ADC		//!< Analog converter data
#define BLUEIO_PACKET_ID_GPIO			BLUEIO_DATA_TYPE_GPIO		//!< GPIO pins data
#define BLUEIO_PACKET_ID_BUT			BLUEIO_DATA_TYPE_BUT		//!< Button state
#define BLUEIO_PACKET_ID_MOT            BLUEIO_DATA_TYPE_MOT		//!< Motion detection
#define BLUEIO_PACKET_ID_I2C			BLUEIO_DATA_TYPE_I2C		//!< I2C interface data
#define BLUEIO_PACKET_ID_SPI			BLUEIO_DATA_TYPE_SPI		//!< SPI interface data
#define BLUEIO_PACKET_ID_UART			BLUEIO_DATA_TYPE_UART		//!< UART interface data
#define BLUEIO_PACKET_ID_PPI			BLUEIO_DATA_TYPE_PPI		//!< PPI interface data
#define BLUEIO_PACKET_ID_AUDIO			BLUEIO_DATA_TYPE_AUDIO		//!< Audio data
#define BLUEIO_PACKET_ID_DFU			0xFF						//!< Re-boot to DFU mode

#define BLUEIOSRVC_DATA_LEN_MAX			48

#pragma pack(push, 1)

/// BLUEIO Cmd/Data packet definition
typedef struct __Blueio_Packet {
	uint8_t Id;		// Packet id (see defined code above)
	uint8_t Data[BLUEIOSRVC_DATA_LEN_MAX];	// Type specific data follows can be more than
} BLUEIO_PACKET;

#define BLUEIOSRVC_RDCHAR_IDX				0
#define BLUEIOSRVC_WDCHAR_IDX				1

#define DEVICE_NAME_LEN_MAX					10

// BLE configuration

typedef enum __Config_Cmd {
	CFGCMD_SET_ADV,
	CFGCMD_SET_DEVNAME
} CFGCMD;

typedef struct __Config_Data_Write {
	CFGCMD Cmd;
	uint8_t Param[1];	//!< Variable length parameters
} CFG_WR;

typedef struct __Config_Cmd_Param_Adv {
	uint32_t AdvIntervalmSec;	//!< Advertisement interval in msec
	uint32_t AdvTimeoutSec;		//!< Advertisement timeout in sec.
	uint32_t SleepTimeSec;		//!< Sleep time in sec.
} CFGCMD_PARAM_ADV;

#define BLE_CFG_DEVNAME_LEN_MAX	13

typedef struct __Config_Cmd_Param_DevName {
	char Name[BLE_CFG_DEVNAME_LEN_MAX];
} CFGMD_PARAM_DEVNAME;

// GPIO
// Write data format

typedef enum __Gpio_Write_Cmd {
	GPIOWR_CMD_RD,		// Read pin
	GPIOWR_CMD_WR,		// Write pin
	GPIOWR_CMD_CONFIG	// Configure pin
} GPIOWR_CMD;

typedef struct __Gpio_Write {
	GPIOWR_CMD 	Cmd;
	uint8_t		PortNo;
	uint8_t 	PinNo;
	uint8_t 	Value;
} GPIO_WR;

// Read data format
typedef struct __Gpio_Read {
	uint8_t	PortNo;
	uint8_t PinNo;
	uint8_t Value;
} GPIO_RD;

// Button data format

typedef struct __Button_Read {
	uint8_t ButNo;
	uint8_t Value;
} BUT_RD;

typedef struct __Adc_Read {
	uint8_t Chan;		//!< ADC channel number
	float Value;		//!< ADC value in volts
} ADC_RD;

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

