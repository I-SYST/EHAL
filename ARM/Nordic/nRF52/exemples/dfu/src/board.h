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


#define LED1_PORT		BLUEIO_LED1_PORT
#define LED1_PIN		BLUEIO_LED1_PIN
#define LED1_PINOP		BLUEIO_LED1_PINOP

#define LED2_PORT		BLUEIO_LED2_PORT
#define LED2_PIN		BLUEIO_LED2_PIN
#define LED2_PINOP		BLUEIO_LED2_PINOP

#define LED3_PORT		BLUEIO_LED3_PORT
#define LED3_PIN		BLUEIO_LED3_PIN
#define LED3_PINOP		BLUEIO_LED3_PINOP

#define LED4_PORT		BLUEIO_LED4_PORT
#define LED4_PIN		BLUEIO_LED4_PIN
#define LED4_PINOP		BLUEIO_LED4_PINOP

#define BUT_N_LED_PINS_MAP		{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__

