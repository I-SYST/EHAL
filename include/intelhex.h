/*--------------------------------------------------------------------------
File   : intelhex.h

Author : Hoang Nguyen Hoan          Feb. 8, 2015

Desc   : Intel Hex parser

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

#ifndef __INTELHEX_H__
#define __INTELHEX_H__

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define IHEX_RECTYPE_DATA		0
#define IHEX_RECTYPE_EOF		1		// End of file
#define IHEX_RECTYPE_EXTSEG		2		// Extended segment address
#define IHEX_RECTYPE_STARTSEG	3		// Start segment address
#define IHEX_RECTYPE_EXTLADDR	4		// Extended linear address
#define IHEX_RECTYPE_STARTLADDR	5		// Start linear address


#define IHEX_MAX_RECSIZE	16

typedef struct {
	int Count;		// Data count
	int Offset;		// Address offset
	int Type;		// Record type
	int Checksum;	// Record checksum
	uint8_t Data[IHEX_MAX_RECSIZE];
} IHEXDATA;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Parse Intel Hex record (one line)
 *
 * @param	pRec : Pointer to text line of intel hex record
 * 			pData : Pointer to place holder for parsed record
 *
 * @return	true - Success
 * 			false - Bad in record data
 */
bool IHexParseRecord(char *pRec, IHEXDATA *pData);

	
#ifdef __cplusplus
}
#endif

#endif // __INTELHEX_H__
