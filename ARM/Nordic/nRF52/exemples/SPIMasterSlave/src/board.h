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

// IBK-NRF5x
// Button 1
#define BUT1_PORT		BLUEIO_BUT1_PORT
#define BUT1_PIN		BLUEIO_BUT1_PIN
#define BUT1_PINOP		BLUEIO_BUT1_PINOP
#define BUT1_SENSE		IOPINSENSE_LOW_TRANSITION
#define BUT1_SENSE_INT	0
#define BUT1_INT_PRIO	6

// Button 2
#define BUT2_PORT		BLUEIO_BUT2_PORT
#define BUT2_PIN		BLUEIO_BUT2_PIN
#define BUT2_PINOP		BLUEIO_BUT2_PINOP
#define BUT2_SENSE		IOPINSENSE_LOW_TRANSITION
#define BUT2_SENSE_INT	BUT2_PIN
#define BUT2_INT_PRIO	6


// LED4
#define LED1_PORT		BLUEIO_LED1_PORT
#define LED1_PIN		BLUEIO_LED1_PIN
#define LED1_PINOP		BLUEIO_LED1_PINOP

// LED5
#define LED2_PORT		BLUEIO_LED2_PORT
#define LED2_PIN		BLUEIO_LED2_PIN
#define LED2_PINOP		BLUEIO_LED2_PINOP

#define SPI_MASTER_DEVNO		1
#define SPI_MASTER_SCK_PORT		0
#define SPI_MASTER_SCK_PIN		21
#define SPI_MASTER_SCK_PINOP	1
#define SPI_MASTER_MISO_PORT	0
#define SPI_MASTER_MISO_PIN		22
#define SPI_MASTER_MISO_PINOP	1
#define SPI_MASTER_MOSI_PORT	0
#define SPI_MASTER_MOSI_PIN		23
#define SPI_MASTER_MOSI_PINOP	1

#define SPI_MASTER_CS_PORT		0
#define SPI_MASTER_CS_PIN		24
#define SPI_MASTER_CS_PINOP		1

#define SPI_SLAVE_DEVNO			2
#define SPI_SLAVE_SCK_PORT		0
#define SPI_SLAVE_SCK_PIN		19
#define SPI_SLAVE_SCK_PINOP		1
#define SPI_SLAVE_MISO_PORT		0
#define SPI_SLAVE_MISO_PIN		18
#define SPI_SLAVE_MISO_PINOP	1
#define SPI_SLAVE_MOSI_PORT		0
#define SPI_SLAVE_MOSI_PIN		17
#define SPI_SLAVE_MOSI_PINOP	1

#define SPI_SLAVE_CS_PORT		0
#define SPI_SLAVE_CS_PIN		16
#define SPI_SLAVE_CS_PINOP		1

#define BUT_N_LED_PINS_MAP		{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
}

#define UART_DEVNO			0
#define UART_RX_PORT		BLUEIO_UART_RX_PORT
#define UART_RX_PIN			BLUEIO_UART_RX_PIN
#define UART_RX_PINOP		BLUEIO_UART_RX_PINOP
#define UART_TX_PORT		BLUEIO_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_UART_TX_PINOP
#define UART_CTS_PORT		BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN		BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP		BLUEIO_UART_CTS_PINOP
#define UART_RTS_PORT		BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN		BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP		BLUEIO_UART_RTS_PINOP

#endif // __BOARD_H__








