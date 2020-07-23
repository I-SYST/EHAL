/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

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
#ifndef __BOARD_H__
#define __BOARD_H__

#include "nrf.h"
#include "blueio_board.h"

//#define NORDIC_DK			// For Nordic DevKit
//#define NEBLINA			// Neblina board

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

#elif defined(NEBLINA)

#define UART_RX_PORT				0
#define UART_RX_PIN					6
#define UART_RX_PINOP				1
#define UART_TX_PORT				0
#define UART_TX_PIN					7
#define UART_TX_PINOP				1
#define UART_CTS_PORT				0
#define UART_CTS_PIN				29
#define UART_CTS_PINOP				1
#define UART_RTS_PORT				0
#define UART_RTS_PIN				28
#define UART_RTS_PINOP				1

#define SPI_DEVNO            		0
#define SPI_MISO_PORT        		0
#define SPI_MISO_PIN         		16
#define SPI_MISO_PINOP       		1
#define SPI_MOSI_PORT        		0
#define SPI_MOSI_PIN         		15
#define SPI_MOSI_PINOP       		1
#define SPI_SCK_PORT         		0
#define SPI_SCK_PIN          		14
#define SPI_SCK_PINOP        		1

#define SPI_FLASH_CS_PORT          	0
#define SPI_FLASH_CS_PIN           	25
#define SPI_FLASH_CS_PINOP         	0

#define FLASH_HOLD_PORT            	0
#define FLASH_HOLD_PIN             	26
#define FLASH_HOLD_PINOP           	0

#else

// I-SYST BlueIO boards

#define UART_RX_PORT					BLUEIO_UART_RX_PORT
#define UART_RX_PIN						BLUEIO_UART_RX_PIN
#define UART_RX_PINOP					BLUEIO_UART_RX_PINOP
#define UART_TX_PORT					BLUEIO_UART_TX_PORT
#define UART_TX_PIN						BLUEIO_UART_TX_PIN
#define UART_TX_PINOP					BLUEIO_UART_TX_PINOP
#define UART_CTS_PORT					BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN					BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP					BLUEIO_UART_CTS_PINOP
#define UART_RTS_PORT					BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN					BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP					BLUEIO_UART_RTS_PINOP

//#define BLUEIO_TAG_BME680_PROTO
#ifdef BLUEIO_TAG_BME680_PROTO
#define I2C0_SDA_PORT					BLUEIO_TAG_BME280_I2C_SDA_PORT
#define I2C0_SDA_PIN					BLUEIO_TAG_BME280_I2C_SDA_PIN
#define I2C0_SDA_PINOP					BLUEIO_TAG_BME280_I2C_SDA_PINOP
#define I2C0_SCL_PORT					BLUEIO_TAG_BME280_I2C_SCL_PORT
#define I2C0_SCL_PIN					BLUEIO_TAG_BME280_I2C_SCL_PIN
#define I2C0_SCL_PINOP					BLUEIO_TAG_BME280_I2C_SCL_PINOP
#else
#define I2C0_SDA_PORT					BLUEIO_TAG_BME680_I2C_SDA_PORT
#define I2C0_SDA_PIN					BLUEIO_TAG_BME680_I2C_SDA_PIN
#define I2C0_SDA_PINOP					BLUEIO_TAG_BME680_I2C_SDA_PINOP
#define I2C0_SCL_PORT					BLUEIO_TAG_BME680_I2C_SCL_PORT
#define I2C0_SCL_PIN					BLUEIO_TAG_BME680_I2C_SCL_PIN
#define I2C0_SCL_PINOP					BLUEIO_TAG_BME680_I2C_SCL_PINOP
#endif

#ifdef BLUEIO_TAG_EVIM
#define SPI_DEVNO      					1
#define SPI_MODE						SPIMODE_NORMAL
#define SPI_MISO_PORT       			0
#define SPI_MISO_PIN        			15//13
#define SPI_MISO_PINOP      			1
#define SPI_MOSI_PORT       			0
#define SPI_MOSI_PIN        			16//12
#define SPI_MOSI_PINOP      			1
#define SPI_SCK_PORT        			0
#define SPI_SCK_PIN         			17//11
#define SPI_SCK_PINOP       			1

#define SPI2_DEVNO      				2
#define SPI2_MISO_PORT       			0
#define SPI2_MISO_PIN        			15
#define SPI2_MISO_PINOP      			1
#define SPI2_MOSI_PORT       			0
#define SPI2_MOSI_PIN        			16
#define SPI2_MOSI_PINOP      			1
#define SPI2_SCK_PORT        			0
#define SPI2_SCK_PIN         			17
#define SPI2_SCK_PINOP       			1
#define SPI_FLASH_CS_PORT          	0
#define SPI_FLASH_CS_PIN           	26
#define SPI_FLASH_CS_PINOP         	0

#define FLASH_HOLD_PORT            	0
#define FLASH_HOLD_PIN             	26
#define FLASH_HOLD_PINOP           	0
#else
#define SPI_DEVNO      					1
#define SPI_PHY							SPIPHY_NORMAL
#define SPI_MISO_PORT       			0
#define SPI_MISO_PIN        			5//13
#define SPI_MISO_PINOP      			1
#define SPI_MOSI_PORT       			0
#define SPI_MOSI_PIN        			7//12
#define SPI_MOSI_PINOP      			1
#define SPI_SCK_PORT        			0
#define SPI_SCK_PIN         			4
#define SPI_SCK_PINOP       			1

#define SPI_FLASH_CS_PORT          	0
#define SPI_FLASH_CS_PIN           	18
#define SPI_FLASH_CS_PINOP         	0

#define FLASH_HOLD_PORT            	0
#define FLASH_HOLD_PIN             	26
#define FLASH_HOLD_PINOP           	0
#endif

#define BME280_CS_IDX          		0
#define BME280_CS_PORT         		0
#define BME280_CS_PIN          		26
#define BME280_CS_PINOP        		1


#define BLUEIO_TAG_EVIM_IMU_INT_NO		0

#define SPI_PINS_CFG	{ \
	{SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN, SPI_FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}


#endif


#endif // __BOARD_H__

