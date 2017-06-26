/*--------------------------------------------------------------------------
File   : lpci2c.h

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : I2C implementation on LPC
		 Current implementation
		 	 Master mode
		 	 Polling

Copyright (c) 2011, I-SYST inc., all rights reserved

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
#ifndef __LPCI2C_H__
#define __LPCI2C_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "i2c.h"

// Power connect
#define LPC_PCONP_I2C0				(1 << 7)
#define LPC_PCONP_I2C1				(1 << 19)
#define LPC_PCONP_I2C2				(1 << 26)

// Clock select
#define LPC_PCLKSEL0_I2C0_MASK 		(3 << 14)	// On PCLKSEL0
#define LPC_PCLKSEL0_I2C0_DIV1		(1 << 14)
#define LPC_PCLKSEL0_I2C0_DIV2		(2 << 14)
#define LPC_PCLKSEL0_I2C0_DIV4		(0)
#define LPC_PCLKSEL0_I2C0_DIV8		(3 << 14)

#define LPC_PCLKSEL1_I2C1_MASK 		(3 << 6)	// On PCLKSEL1
#define LPC_PCLKSEL1_I2C1_DIV1		(1 << 6)
#define LPC_PCLKSEL1_I2C1_DIV2		(2 << 6)
#define LPC_PCLKSEL1_I2C1_DIV4		(0)
#define LPC_PCLKSEL1_I2C1_DIV8		(3 << 6)

#define LPC_PCLKSEL1_I2C2_MASK 		(3 << 20)	// On PCLKSEL1
#define LPC_PCLKSEL1_I2C2_DIV1		(1 << 20)
#define LPC_PCLKSEL1_I2C2_DIV2		(2 << 20)
#define LPC_PCLKSEL1_I2C2_DIV4		(0)
#define LPC_PCLKSEL1_I2C2_DIV8		(3 << 20)

// I2C Register definitions

// Control Set
#define LPCI2C_I2CONSET_AA			4			// Assert acknowledge flag
#define LPCI2C_I2CONSET_SI			8			// I2C interrupt flag
#define LPCI2C_I2CONSET_STO			0x10		// STOP flag
#define LPCI2C_I2CONSET_STA			0x20		// START flag
#define LPCI2C_I2CONSET_I2EN		0x40		// I2C interface enable

// Control Clear
#define LPCI2C_I2CONCLR_AAC			4			// Assert acknowledge flag
#define LPCI2C_I2CONCLR_SIC			8			// I2C interrupt flag
#define LPCI2C_I2CONCLR_STOC		0x10		// STOP flag
#define LPCI2C_I2CONCLR_STAC		0x20		// START flag
#define LPCI2C_I2CONCLR_I2ENC		0x40		// I2C interface enable

//#define LPCI2C_MAX_INTRF			3			// Max number of I2C interface

typedef struct {
	volatile uint32_t I2CONSET;
	volatile  uint32_t I2STAT;
	volatile uint32_t I2DAT;
	volatile uint32_t I2ADR0;
	volatile uint32_t I2SCLH;
	volatile uint32_t I2SCLL;
	volatile  uint32_t I2CONCLR;
	volatile uint32_t MMCTRL;
	volatile uint32_t I2ADR1;
	volatile uint32_t I2ADR2;
	volatile uint32_t I2ADR3;
	volatile  uint32_t I2DATA_BUFFER;
	volatile uint32_t I2MASK0;
	volatile uint32_t I2MASK1;
	volatile uint32_t I2MASK2;
	volatile uint32_t I2MASK3;
} LPCI2CREG;

// Device driver data require by low level functions
typedef struct {
	int I2CNo;					// I2C interface number
	uint32_t Rate;				// Data rate
	I2CDEV	*pI2cDev;			// Pointer to generic I2C dev data
	LPCI2CREG *pI2CReg;	// pointer to I2C registers map
} LPCI2CDEV;

#ifdef __cplusplus
extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}
#endif	// __cplusplus

#endif	// __LPCI2C_H__
