/**-------------------------------------------------------------------------
@file	board.h

@brief	Contains I/O pin defines for board application specific
		This file is included with each project.  To be modified to map
		for the board used.

@author Hoang Nguyen Hoan
@date	May 15, 2016

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

#include "blueio_board.h"

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

#define SPI_DEVNO      					2
#define SPI_MISO_PORT       			0
#define SPI_MISO_PIN        			17//13
#define SPI_MISO_PINOP      			1
#define SPI_MOSI_PORT       			0
#define SPI_MOSI_PIN        			16//12
#define SPI_MOSI_PINOP      			1
#define SPI_SCK_PORT        			0
#define SPI_SCK_PIN         			15//11
#define SPI_SCK_PINOP       			1

#define BMEx80_CS_IDX          			0
#define BMEx80_CS_PORT         			0
#define BMEx80_CS_PIN          			18//26
#define BMEx80_CS_PINOP        			1

#endif // __BOARD_H__

