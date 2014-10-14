/*--------------------------------------------------------------------------
File   : imm_lpc1769.h

Author : Hoang Nguyen Hoan          Dec. 30, 2011

Desc   : On board peripheral I/O mapping for IMM-LPC1769 module.

Copyright (c) 2011, I-SYST, all rights reserved

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
#ifndef __IMM_LPC1769_H__
#define __IMM_LPC1769_H__

// On board SEEP with MAC address
// SEEP device : Microchip 24AA02E48T-I/OT
// SEEP is interfaced via I2C1
#define IMM_LPC1769_SEEPMAC_SDA_PORT			0
#define IMM_LPC1769_SEEPMAC_SDA_PIN				19
#define IMM_LPC1769_SEEPMAC_SDA_PIN_MODE		PINSEL_PINMODE_PULLUP
#define IMM_LPC1769_SEEPMAC_SDA_PIN_ODMODE		PINSEL_PINMODE_OPENDRAIN
#define IMM_LPC1769_SEEPMAC_SCL_PORT			0
#define IMM_LPC1769_SEEPMAC_SCL_PIN				20
#define IMM_LPC1769_SEEPMAC_SCL_PIN_MODE		PINSEL_PINMODE_PULLUP
#define IMM_LPC1769_SEEPMAC_SCL_PIN_ODMODE		PINSEL_PINMODE_OPENDRAIN

// LEDs
#define IMM_LPC1769_LED_BLUE_PORT				1
#define IMM_LPC1769_LED_BLUE_PIN				20
#define IMM_LPC1769_LED_RED_PORT				1
#define IMM_LPC1769_LED_RED_PIN					21
#define IMM_LPC1769_LED_YEL_PORT				1
#define IMM_LPC1769_LED_YEL_PIN					23



#endif // __IMM_LPC1769_H__
