/*
 * board.h
 *
 *  Created on: Nov 15, 2016
 *      Author: hoan
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include "blueio_board.h"

//#define NORDIC_DK
//#define NEBLINA

#ifdef NORDIC_DK
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			6
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PIN		7
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		5
#define UART_RTS_PINOP		1
#elif defined(NEBLINA)
#define UART_RX_PORT		0
#define UART_RX_PIN			6
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			7
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PIN		29
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		28
#define UART_RTS_PINOP		1
#else
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
#endif


#endif // __BOARD_H__

