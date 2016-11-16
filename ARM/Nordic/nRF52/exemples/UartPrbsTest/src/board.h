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

#ifdef NORDIC_DK
#define UART_TX_PIN			9//7
#define UART_RX_PIN			11//8
#define UART_RTS_PIN		8//11
#define UART_CTS_PIN		10//12
#else
#define UART_TX_PIN			BLUEIO_UART_TX
#define UART_RX_PIN			BLUEIO_UART_RX
#define UART_RTS_PIN		BLUEIO_UART_RTS
#define UART_CTS_PIN		BLUEIO_UART_CTS
#endif


#endif // __BOARD_H__

