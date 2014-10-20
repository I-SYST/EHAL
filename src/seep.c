/*--------------------------------------------------------------------------
File   : seep.c

Author : Hoang Nguyen Hoan          Sept. 16, 2011

Desc   : Serial EEPROM device implementation

Copyright (c) 2011, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "istddef.h"
#include "seep.h"

bool SeepInit(SEEPDEV *pDev, int DevAddr, int PageSize, int AddrLen, SERINTRFDEV *pIntrf)
{
	pDev->pSerIntrf = pIntrf;
	//vpInterf = pInterf;
	pDev->DevAddr = DevAddr;
	pDev->PageSize = PageSize;
	pDev->AddrLen = AddrLen;

	return true;
}

int SeepRead(SEEPDEV *pDev, int Addr, uint8_t *pData, int Len)
{
	uint8_t ad[4];
	uint8_t *p = (uint8_t*)&Addr;

	for (int i = 0; i < pDev->AddrLen; i++)
	{
		ad[i] = p[pDev->AddrLen - i - 1];
	}

	if (SerialIntrfTx(pDev->pSerIntrf, pDev->DevAddr, (uint8_t*)ad, pDev->AddrLen))
	{
		return SerialIntrfRx(pDev->pSerIntrf, pDev->DevAddr, pData, Len);
	}

	return 0;
}

// Note: Sequential write is bound by page size boundary
int SeepWrite(SEEPDEV *pDev, int Addr, uint8_t *pData, int Len)
{
	int count = 0;
	uint8_t ad[4];
	uint8_t *p = (uint8_t*)&Addr;

	while (Len > 0)
	{
		int size = min(Len, pDev->PageSize - (Addr % pDev->PageSize));
		for (int i = 0; i < pDev->AddrLen; i++)
		{
			ad[i] = p[pDev->AddrLen - i - 1];
		}

		if (pDev->pSerIntrf->StartTx(pDev->pSerIntrf, pDev->DevAddr))
		{
			pDev->pSerIntrf->TxData(pDev->pSerIntrf, ad, pDev->AddrLen);
			count += pDev->pSerIntrf->TxData(pDev->pSerIntrf, pData, size);
			pDev->pSerIntrf->StopTx(pDev->pSerIntrf);
		}
		Addr += size;
		Len -= size;
		pData += size;
	}
	return count;
}

