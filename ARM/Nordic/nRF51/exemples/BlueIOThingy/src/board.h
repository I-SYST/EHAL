/*
 * board.h
 *
 *  Created on: Nov 15, 2016
 *      Author: hoan
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "blueio_board.h"

//#define NORDIC_DK			// For Nordic DevKit
//#define NEBLINA_MODULE	// Motsai Neblina

#ifdef NORDIC_DK

#define UART_TX_PIN					9//7
#define UART_RX_PIN					11//8
#define UART_RTS_PIN					8//11
#define UART_CTS_PIN					10//12

#define SPI_DEVNO            		2
#define SPI_MISO_PORT        		0
#define SPI_MISO_PIN         		13
#define SPI_MISO_PINOP       		1
#define SPI_MOSI_PORT        		0
#define SPI_MOSI_PIN         		12
#define SPI_MOSI_PINOP       		1
#define SPI_SCK_PORT         		0
#define SPI_SCK_PIN          		11
#define SPI_SCK_PINOP        		1

#define SPI_BME280_CS_PORT         	0
#define SPI_BME280_CS_PIN          	26
#define SPI_BME280_CS_PINOP        	1

#elif defined(NEBLINA_MODULE)

#define UART_RX_PORT					0
#define UART_RX_PIN					6
#define UART_RX_PINOP				1
#define UART_TX_PORT					0
#define UART_TX_PIN					7
#define UART_TX_PINOP				1
#define UART_CTS_PORT				0
#define UART_CTS_PIN					29
#define UART_CTS_PINOP				1
#define UART_RTS_PORT				0
#define UART_RTS_PIN					28
#define UART_RTS_PINOP				1

#define SPI_DEVNO            		2
#define SPI_MISO_PORT        		0
#define SPI_MISO_PIN         		13
#define SPI_MISO_PINOP       		1
#define SPI_MOSI_PORT        		0
#define SPI_MOSI_PIN         		12
#define SPI_MOSI_PINOP       		1
#define SPI_SCK_PORT         		0
#define SPI_SCK_PIN          		11
#define SPI_SCK_PINOP        		1

#define BME280_CS_PORT         		0
#define BME280_CS_PIN          		26
#define BME280_CS_PINOP        		1

#else

// I-SYST BlueIO boards

#define UART_RX_PORT					BLUEIO_UART_RX_PORT
#define UART_RX_PIN					BLUEIO_UART_RX_PIN
#define UART_RX_PINOP				BLUEIO_UART_RX_PINOP
#define UART_TX_PORT					BLUEIO_UART_TX_PORT
#define UART_TX_PIN					BLUEIO_UART_TX_PIN
#define UART_TX_PINOP				BLUEIO_UART_TX_PINOP
#define UART_CTS_PORT				BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN					BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP				BLUEIO_UART_CTS_PINOP
#define UART_RTS_PORT				BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN					BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP				BLUEIO_UART_RTS_PINOP

#define BLUEIO_TAG_BME680_PROTO
#ifdef BLUEIO_TAG_BME680_PROTO
#define I2C0_SDA_PORT				BLUEIO_TAG_BME280_I2C_SDA_PORT
#define I2C0_SDA_PIN					BLUEIO_TAG_BME280_I2C_SDA_PIN
#define I2C0_SDA_PINOP				BLUEIO_TAG_BME280_I2C_SDA_PINOP
#define I2C0_SCL_PORT				BLUEIO_TAG_BME280_I2C_SCL_PORT
#define I2C0_SCL_PIN					BLUEIO_TAG_BME280_I2C_SCL_PIN
#define I2C0_SCL_PINOP				BLUEIO_TAG_BME280_I2C_SCL_PINOP
#else
#define I2C0_SDA_PORT				BLUEIO_TAG_BME680_I2C_SDA_PORT
#define I2C0_SDA_PIN					BLUEIO_TAG_BME680_I2C_SDA_PIN
#define I2C0_SDA_PINOP				BLUEIO_TAG_BME680_I2C_SDA_PINOP
#define I2C0_SCL_PORT				BLUEIO_TAG_BME680_I2C_SCL_PORT
#define I2C0_SCL_PIN					BLUEIO_TAG_BME680_I2C_SCL_PIN
#define I2C0_SCL_PINOP				BLUEIO_TAG_BME680_I2C_SCL_PINOP
#endif

#define SPI_DEVNO      				2
#define SPI_MISO_PORT       			0
#define SPI_MISO_PIN        			13
#define SPI_MISO_PINOP      			1
#define SPI_MOSI_PORT       			0
#define SPI_MOSI_PIN        			12
#define SPI_MOSI_PINOP      			1
#define SPI_SCK_PORT        			0
#define SPI_SCK_PIN         			11
#define SPI_SCK_PINOP       			1

#define BME280_CS_IDX          		0
#define BME280_CS_PORT         		0
#define BME280_CS_PIN          		26
#define BME280_CS_PINOP        		1

#endif

#endif // __BOARD_H__

