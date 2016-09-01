/*--------------------------------------------------------------------------
File   : blueio_board.h

Author : Hoang Nguyen Hoan          Sep. 20, 2014

Desc   : Contains definitions for BlueIO series boards for the IMM-NRF5x 
		 series modules

Copyright (c) 2014, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/

#ifndef __BLUEIO_BOARD_H__
#define __BLUEIO_BOARD_H__

#ifndef BLUEIO_BOARD
#define BLUEIO_BOARD				1
#endif

#define BLUEIO_LED1					30
#define BLUEIO_LED2					29
#define BLUEIO_LED3					28
#define BLUEIO_LED_BLUE				BLUEIO_LED1
#define BLUEIO_LED_GREEN			BLUEIO_LED2
#define BLUEIO_LED_RED				BLUEIO_LED3

// Most IMM-NRF5x base boards use P0.30 for Bluetooth connection state LED
#define BLUEIO_CONNECT_LED			BLUEIO_LED1

// UART pins
#define BLUEIO_UART_TX				7
#define BLUEIO_UART_RX				8
#define BLUEIO_UART_RTS				11
#define BLUEIO_UART_CTS				12


#endif // __BLUEIO_BOARD_H__
