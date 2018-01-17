/*
 * board.h
 *
 *  Created on: Nov 15, 2016
 *      Author: hoan
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "blueio_board.h"

#define NORDIC_DK

#ifdef NORDIC_DK
// Nordic DK PCA10040 board

#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1	//
#define UART_TX_PORT		0
#define UART_TX_PIN			6//7
#define UART_TX_PINOP		0
#define UART_CTS_PORT		0
#define UART_CTS_PIN		7//12
#define UART_CTS_PINOP		0
#define UART_RTS_PORT		0
#define UART_RTS_PIN		5//11
#define UART_RTS_PINOP		0

#define BUTTON1_PORT		0
#define BUTTON1_PIN			13
#define BUTTON2_PORT		0
#define BUTTON2_PIN			14

#else
// BlueIO breakout board
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

#define BUTTON1_PORT		BLUEIO_BUT1_PORT
#define BUTTON1_PIN			BLUEIO_BUT1_PIN
#define BUTTON2_PORT		BLUEIO_BUT2_PORT
#define BUTTON2_PIN			BLUEIO_BUT2_PIN

#endif


#endif // __BOARD_H__

