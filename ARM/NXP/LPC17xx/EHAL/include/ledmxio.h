/*--------------------------------------------------------------------------
File   : ledmxio.h

Author : Hoang Nguyen Hoan          Aug. 15, 2013

Desc   : This is platform specific I/O control interface for LED Matrix Control

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

#ifndef __LEDMXIO_H__
#define __LEDMXIO_H__

#include "miscdev/ledmx.h"

#define LEDMX_MAX_ADDRPIN		4

typedef struct _LedMxIOCfg {
	int WrPort;					// WR pin map GPIO port #
	int WrPin;					// WR pin map GPIO pin #
	int RdPort;					// RD pin map GPIO port #
	int RdPin;					// RD pin map GPIO pin #
	int DataPort;				// DATA pin map GPIO port #
    int DataPin;				// DATA pin map GPIO pin #
	int EnPort;					// Decoder enable pin map GPIO port #
    int EnPin;					// Decoder enable pin map GPIO pin #
	int CsPorts[LEDMX_MAX_ADDRPIN];	// CS pins map GPIO port #s
    int CsPins[LEDMX_MAX_ADDRPIN];	// CS pins map GPIO pin #s
    int NbCsPins;				// Total number of CS pins used
	LEDMX_CSTYPE CsType;
} LEDMXIOCFG;




#endif /* __LEDMXIO_H__ */
