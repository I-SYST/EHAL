/**-------------------------------------------------------------------------
@file	ledmxio.h

@brief		This is platform specific I/O control interface for IDM-LMX3208
series LED matrix display


@author	Hoang Nguyen Hoan
@date	Aug. 21, 2014

@license

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

----------------------------------------------------------------------------*/

#ifndef __LEDMXIO_H__
#define __LEDMXIO_H__

#include "miscdev/ledmx.h"

// LED matrix controller Breakout board pin mapping
#define LMXBRK_CSTYPE               LEDMX_CSTYPE_BIN

// IDC-LMXBLUE LED matrix display controller board pin mapping
// aka IBB-LMXBLUE
#define LMXBLUE_CSTYPE				LEDMX_CSTYPE_BIN
#define LMXBLUE_RD_PIN				15		// RR signal pin
#define LMXBLUE_WR_PIN				14		// WR signal pin
#define LMXBLUE_DATA_PIN			13		// Data pin
#define LMXBLUE_EN_PIN				12		// Enable pin
#define LMXBLUE_AD3_PIN				11		//
#define LMXBLUE_AD2_PIN				10		// CS addressing 4 pin => 16 Displays
#define LMXBLUE_AD1_PIN				9		//
#define LMXBLUE_AD0_PIN				8		//

#define LEDMX_MAX_ADDRPIN			4	// Number of pins to address the display

typedef struct _LedMxIOCfg {
  int WrPin;				// WR pin map GPIO pin #
  int RdPin;				// RD pin map GPIO pin #
  int DataPin;				// DATA pin map GPIO pin #
  int EnPin;				// Decoder enable pin map GPIO pin #
  int CsPins[LEDMX_MAX_ADDRPIN];	// CS pins map GPIO pin #
  int NbCsPins;				// Total number of CS pins used
  LEDMX_CSTYPE CsType;
} LEDMXIOCFG;

#endif // __LEDMXIO_H__
