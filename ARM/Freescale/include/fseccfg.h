/*--------------------------------------------------------------------------
File   : fseccfg.h

Author : Hoang Nguyen Hoan          July 3, 2015

Desc   : Freescale Flash security configuration data structure.
		 This data is located at 0x400 in the Flash

Copyright (c) 2015, I-SYST inc., all rights reserved

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
#ifndef __FSECCFG_H__
#define __FSECCFG_H__
#include <stdint.h>

#pragma pack(push, 1)
typedef struct {
	uint8_t Key[8];		// Backdoor key
	uint8_t FProt[4];	// Default program flash protection settings
	uint8_t FDProt;		// Default data flash protection settings
	uint8_t FEProt;		// Default EEPROM protection settings
	uint8_t FOpt;		// Flash option byte
	uint8_t FSec;		// Default flash security register
} FSECCFG;

#pragma pack(pop)

#endif	// __FSECCFG_H__
