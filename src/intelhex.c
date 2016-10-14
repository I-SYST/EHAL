/*--------------------------------------------------------------------------
File   : intelhex.c

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
#include "convutil.h"
#include "intelhex.h"

/*
 * Parse Intel Hex record (one line)
 *
 * @param	pRec : Pointer to text line of intel hex record
 * 			pData : Pointer to place holder for parsed record
 *
 * @return	true - Success
 * 			false - Bad in record data
 */
bool IHexParseRecord(char *pRec, IHEXDATA *pData)
{
	if (pRec == NULL || pData == NULL)
		return false;
	
	if (pRec[0] != ':')
		return false;

	char *p = pRec + 1;
	int8_t cs = 0;
	
	pData->Count = (chex2i(*p++) << 4);
	pData->Count += chex2i(*p++);
	pData->Offset = (chex2i(*p++) << 12);
	pData->Offset += (chex2i(*p++) << 8);
	pData->Offset += (chex2i(*p++) << 4);
	pData->Offset += chex2i(*p++);
	pData->Type = (chex2i(*p++) << 4);
	pData->Type += chex2i(*p++);
	
	cs += pData->Count + (pData->Offset & 0xff) + ((pData->Offset >> 8u) & 0xff) + pData->Type;
	
	for (int i = 0; i < pData->Count; i++)
	{
		pData->Data[i] = (chex2i(*p++) << 4);
		pData->Data[i] += chex2i(*p++);
		cs += pData->Data[i];
	}
	
	pData->Checksum = (chex2i(*p++) << 4);
	pData->Checksum += chex2i(*p++);
	cs += pData->Checksum;
	
	return cs == 0;
}
