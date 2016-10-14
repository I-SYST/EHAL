/*--------------------------------------------------------------------------
File   : cfifo.c

Author : Hoang Nguyen Hoan          Jan. 3, 2014

Desc   : Circular FIFO buffer manager


Copyright (c) 2014, I-SYST, all rights reserved

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
#include <stdint.h>
#include <string.h>
#include "atomic.h"
#include "cfifo.h"

HCFIFO CFifoInit(uint8_t *pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize)
{
	if (pMemBlk == NULL)
		return NULL;

	CFIFOHDR *hdr = (CFIFOHDR *)pMemBlk;
	hdr->PutIdx = 0;
	hdr->GetIdx = -1;
	hdr->BlkSize = BlkSize;
	hdr->MemSize = TotalMemSize;
	hdr->MaxIdxCnt = (TotalMemSize - sizeof(CFIFOHDR)) / BlkSize;
	hdr->pMemStart = (uint8_t*)(pMemBlk + sizeof(CFIFOHDR));

	return hdr;
}

uint8_t *CFifoGet(HCFIFO pFifo)
{
	if (pFifo == NULL || pFifo->GetIdx < 0)
		return NULL;

	int32_t idx = pFifo->GetIdx;
	int32_t getidx = idx + 1;

	if (getidx >= pFifo->MaxIdxCnt)
		getidx = 0;
	if (getidx == pFifo->PutIdx)
		getidx = -1;

	AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, getidx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoGetMultiple(HCFIFO pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoGet(pFifo);

	if (pFifo == NULL || pFifo->GetIdx < 0 || *pCnt == 0)
	{
		*pCnt = 0;
		return NULL;
	}

	int32_t cnt = *pCnt;
	int32_t putidx = pFifo->PutIdx;
	int32_t idx = pFifo->GetIdx;
	int32_t getidx = idx + cnt;

	if (idx < putidx)
	{
		if (getidx >= putidx)
		{
			getidx = -1;
			cnt = putidx- idx;
		}
	}
	else
	{
		if (getidx >= pFifo->MaxIdxCnt)
		{
			getidx = putidx == 0 ? -1 : 0;
			cnt = pFifo->MaxIdxCnt - idx;
		}
	}

	AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, getidx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;
	*pCnt = cnt;

	return p;
}

uint8_t *CFifoPut(HCFIFO pFifo)
{
	if (pFifo == NULL || pFifo->PutIdx == pFifo->GetIdx)
		return NULL;

	int32_t idx = pFifo->PutIdx;
	int32_t putidx = idx + 1;
	if (putidx >= pFifo->MaxIdxCnt)
		putidx = 0;
	AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, putidx);
	if (pFifo->GetIdx < 0)
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, idx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoPutMultiple(HCFIFO pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoPut(pFifo);

	if (pFifo == NULL || pFifo->PutIdx == pFifo->GetIdx || *pCnt == 0)
	{
		*pCnt = 0;
		return NULL;
	}

	int32_t cnt = *pCnt;
	int32_t idx = pFifo->PutIdx;
	int32_t getidx = pFifo->GetIdx;
	int32_t putidx = idx + cnt;

	if (idx > getidx)
	{
		if (putidx >= pFifo->MaxIdxCnt)
		{
			cnt = pFifo->MaxIdxCnt - idx;
			putidx = 0;
		}
	}
	else
	{
		if (putidx >= getidx)
		{
			cnt = getidx - idx;
			putidx = getidx;
		}
	}

	AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, putidx);

	if (getidx < 0)
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, idx);

	uint8_t *p = pFifo->pMemStart + idx * pFifo->BlkSize;

	*pCnt = cnt;

	return p;
}

void CFifoFlush(HCFIFO pFifo)
{
	AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, -1);
	AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, 0);
}

int CFifoAvail(HCFIFO pFifo)
{
	int len = 0;

	if (pFifo->GetIdx < 0)
		return pFifo->MaxIdxCnt;

	if (pFifo->PutIdx > pFifo->GetIdx)
	{
		len = pFifo->MaxIdxCnt - pFifo->PutIdx + pFifo->GetIdx;
	}
	else if (pFifo->PutIdx < pFifo->GetIdx)
	{
		len = pFifo->GetIdx - pFifo->PutIdx;
	}

	return len;
}

int CFifoUsed(HCFIFO pFifo)
{
	int len = 0;

	if (pFifo->GetIdx < 0)
		return 0;

	if (pFifo->GetIdx < pFifo->PutIdx)
	{
		len = pFifo->PutIdx - pFifo->GetIdx;
	}
	else
	{
		len = pFifo->MaxIdxCnt - pFifo->GetIdx + pFifo->PutIdx;
	}

	return len;
}

int CFifoRead(HCFIFO pFifo, uint8_t *pBuff, int BuffLen)
{
	if (pFifo == NULL || pFifo->GetIdx < 0 || pBuff == NULL)
		return 0;

	int cnt = 0;

	if (BuffLen <= pFifo->BlkSize)
	{
		// Single block
		uint8_t *p = CFifoGet(pFifo);
		if (p)
		{
			memcpy(pBuff, p, BuffLen);

			return BuffLen;
		}
	}
	else
	{
		// Span multiple blocks
		while (BuffLen > 0)
		{
			int l = BuffLen / pFifo->BlkSize;
			if ((BuffLen % pFifo->BlkSize) > 0)
				l++;
			uint8_t *p = CFifoGetMultiple(pFifo, &l);
			if (p == NULL)
				break;
			l *= pFifo->BlkSize;
			memcpy(pBuff, p, l);
			pBuff += l;
			BuffLen -= l;
			cnt += l;
		}
	}

	return cnt;
}

int CFifoWrite(HCFIFO pFifo, uint8_t *pData, int DataLen)
{
	if (pFifo == NULL || pFifo->PutIdx == pFifo->GetIdx || pData == NULL)
		return 0;

	int cnt = 0;

	if (DataLen <= pFifo->BlkSize)
	{
		// Single block
		uint8_t *p = CFifoPut(pFifo);
		if (p)
		{
			memcpy(p, pData, DataLen);

			return DataLen;
		}
	}
	else
	{
		// Span multiple blocks
		while (DataLen > 0)
		{
			int l = DataLen / pFifo->BlkSize;
			if ((DataLen % pFifo->BlkSize) > 0)
				l++;
			uint8_t *p = CFifoPutMultiple(pFifo, &l);
			if (p == NULL)
				break;
			l *= pFifo->BlkSize;
			memcpy(p, pData, l);
			pData += l;
			DataLen -= l;
			cnt += l;
		}
	}

	return cnt;
}

