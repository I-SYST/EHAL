/**-------------------------------------------------------------------------
@file	blueio_types.h

@brief	Contains definitions for blueio standard data types

These data types are used in communication protocol to standardize data exchange format.
For example :
	- The manufacturer specific data in BLE advertisement packet.
	- BLUEIO BLE Service data characteristic
	- Serial communications
	- and so on


@author	Hoang Nguyen Hoan
@date	Jan 20, 2018

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

#ifndef __BLUEIO_TYPES_H__
#define __BLUEIO_TYPES_H__

#define BLUEIO_DATA_TYPE_APP			0		//!< Application custom data
#define BLUEIO_DATA_TYPE_TPH			1		//!< Environmental sensor data (Temperature, Pressure, Humidity)
#define BLUEIO_DATA_TYPE_GAS			2		//!< Gas sensor data
#define BLUEIO_DATA_TYPE_ACCEL			3		//!< Accelerometer sensor data
#define BLUEIO_DATA_TYPE_GYRO			4		//!< Gyroscope sensor data
#define BLUEIO_DATA_TYPE_MAG			5		//!< Magnetometer sensor data
#define BLUEIO_DATA_TYPE_PROXY			6		//!< Proximity sensor data
#define BLUEIO_DATA_TYPE_ADC			7		//!< Analog converter data
#define BLUEIO_DATA_TYPE_GPIO			8		//!< GPIO pins state
#define BLUEIO_DATA_TYPE_BUT			9		//!< Button/Switch state
#define BLUEIO_DATA_TYPE_MOT			10		//!< Motion detection
#define BLUEIO_DATA_TYPE_I2C			11		//!< I2C interface data
#define BLUEIO_DATA_TYPE_SPI			12		//!< SPI interface data
#define BLUEIO_DATA_TYPE_UART			13		//!< UART interface data
#define BLUEIO_DATA_TYPE_PPI			14		//!< PPI interface data
#define BLUEIO_DATA_TYPE_AUDIO			15		//!< Audio data
#define BLUEIO_DATA_TYPE_BAT			16		//!< Battery level

#pragma pack(push, 1)

/// Temperature, Pressure, Humidity sensor data
typedef struct __BlueIO_Data_TPH {
	uint32_t Pressure;			//!< Barometric pressure in Pa no decimal
	int16_t  Temperature;		//!< Temperature in degree C, 2 decimals fixed point
	uint16_t Humidity;			//!< Relative humidity in %, 2 decimals fixed point
} BLUEIO_DATA_TPH;

/// Gas sensor data
typedef struct __BlueIO_Data_Gas {
	uint32_t GasRes;				//!< Gas resistance
	uint16_t AirQIdx;			//!< Air quality index
	uint8_t	AirQuality;			//!< Indicate air quality
} BLUEIO_DATA_GAS;

/// Accelerometer data
typedef struct __BlueIO_Data_Accel {
	uint16_t AccelX;
	uint16_t AccelY;
	uint16_t AccelZ;
} BLUEIO_DATA_ACCEL;

/// Gyroscope data
typedef struct __BlueIO_Data_Gyro {
	uint16_t GyroX;
	uint16_t GyroY;
	uint16_t GyroZ;
} BLUEIO_DATA_GYRO;

/// Magnetometer data
typedef struct __BlueIO_Data_Mag {
	uint16_t MagX;
	uint16_t MagY;
	uint16_t MagZ;
} BLUEIO_DATA_MAG;

/// Proximity sensor data
typedef struct __BlueIO_Data_Proxy {
	uint32_t Id;					//!< Sensor ID
	uint32_t Val;				//!< Range value
} BLUEIO_DATA_PROXY;

/// ADC data
typedef struct __BlueIO_Data_Adc {
	uint32_t ChanId;				//!< ADC channel id
	float	 Voltage;			//!< Voltage measured
} BLUEIO_DATA_ADC;

/// GPIO data
typedef struct __BlueIO_Data_Gpio {
	uint8_t PortNo;				//!< Port number
	uint32_t PinVal;				//!< Bit field pin values (bit0 = Pin0, bit1 = Pin1, etc..)
} BLUEIO_DATA_GPIO;

#define BLUEIO_BUTTON_ARRAY_MAX		8

/// Button or switch state
typedef enum __BlueIO_But_State {
	BLUEIO_BUT_STATE_OFF,		//!< State OFF
	BLUEIO_BUT_STATE_ON,		//!< State ON
	BLUEIO_BUT_STATE_PRESSED,	//!< State transition from OFF to ON
	BLUEIO_BUT_STATE_RELEASED	//!< State transition from ON to OFF
} BLUEIO_BUT_STATE;

/// Button data
typedef struct __BlueIO_Data_Button {
	BLUEIO_BUT_STATE ButState[BLUEIO_BUTTON_ARRAY_MAX];	//!< Buttons state
} BLUEIO_DATA_BUT;

/// Motion sensor data
typedef struct __BlueIO_Data_Motiont {
	uint32_t Id;				//!< Sensor ID
	uint32_t Val;				//!< Detection state (0 - no motion, 1 - motion
} BLUEIO_DATA_MOTION;

#define BLUEIO_I2C_DATA_LEN_MAX		6
/// I2C data
typedef struct __BlueIO_Data_I2C {
	uint8_t	Id;					//!< interace ID
	uint8_t Len;				//!< data length
	uint8_t Data[BLUEIO_I2C_DATA_LEN_MAX];	//!< data array
} BLUEIO_DATA_I2C;

#define BLUEIO_SPI_DATA_LEN_MAX		6
/// SPI data
typedef struct __BlueIO_Data_SPI {
	uint8_t	Id;					//!< interace ID
	uint8_t Len;				//!< data length
	uint8_t Data[BLUEIO_I2C_DATA_LEN_MAX];	//!< data array
} BLUEIO_DATA_SPI;

#define BLUEIO_UART_DATA_LEN_MAX		6

/// UART data
typedef struct __BlueIO_Data_UART {
	uint8_t	Id;					//!< interace ID
	uint8_t Len;				//!< data length
	uint8_t Data[BLUEIO_I2C_DATA_LEN_MAX];	//!< data array
} BLUEIO_DATA_UART;

/// PPI data
typedef struct __BlueIO_Data_PPI {
	uint8_t	Id;					//!< interace ID
	uint8_t Len;				//!< data length
	uint8_t Data[1];			//!< data array, variable length
} BLUEIO_DATA_PPI;


/// Audio data, this structure has variable length
typedef struct __BlueIO_Data_Audio {
	uint8_t Chan;				//!< Chan number
	uint8_t CodeType;			//!< Encoding type
	uint16_t Len;				//!< Data length
	uint8_t Data[1];			//!< Audio data variable length
} BLUEIO_DATAAUDIO;

/// Battery level
typedef struct __BlueIO_Data_Bat {
	uint8_t Level;				//!< Battery level in %
	int32_t	Voltage;			//!< Battery voltage in mV (miliVolt)
} BLUEIO_DATA_BAT;

#pragma pack(pop)


#endif // __BLUEIO_TYPES_H__
