/*
 * board.h
 *
 *  Created on: Jun. 11, 2019
 *      Author: hoanmotsai
 */

#ifndef __BOARD_H__
#define __BOARD_H__


#define UART_RX_PORT		0
#define UART_RX_PIN			10
#define UART_RX_PINOP		(2 | (1 << 4))

#define UART_TX_PORT		0
#define UART_TX_PIN			9
#define UART_TX_PINOP		(2 | (1 << 4))

#define UART_CTS_PORT		0
#define UART_CTS_PIN		1
#define UART_CTS_PINOP		(2 | (1 << 4))
#define UART_RTS_PORT		0
#define UART_RTS_PIN		2
#define UART_RTS_PINOP		(2 | (1 << 4))



#endif // __BOARD_H__

