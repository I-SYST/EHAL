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
#include <string.h>
#include "atomic.h"
#include "cfifo.h"

CFIFOHDL *CFifoInit(uint8_t *pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize)
{
	if (pMemBlk == NULL)
		return NULL;

	CFIFOHDL *hdr = (CFIFOHDL *)pMemBlk;
	hdr->PutIdx = 0;
	hdr->GetIdx = -1;
	hdr->BlkSize = BlkSize;
	hdr->MemSize = TotalMemSize;
	hdr->MaxIdxCnt = (TotalMemSize - sizeof(CFIFOHDL)) / BlkSize;
//	hdr->pMemStart = (uint8_t*)(pMemBlk + sizeof(CFIFOHDL));

	return hdr;
}

uint8_t *CFifoGet(CFIFOHDL *pFifo)
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

	uint8_t *p = (uint8_t*)pFifo + sizeof(CFIFOHDL) + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoGetMultiple(CFIFOHDL *pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoGet(pFifo);

	if (pFifo == NULL || pFifo->GetIdx < 0)
	{
		*pCnt = 0;
		return NULL;
	}

	int32_t cnt;
	int32_t putidx = pFifo->PutIdx;
	int32_t getidx = pFifo->GetIdx;
	int32_t idx = getidx;

	if (getidx < putidx)
		cnt = min(*pCnt, putidx - getidx);
	else
		cnt = min(*pCnt, pFifo->MaxIdxCnt - getidx);

	getidx += cnt;

	if (getidx >= pFifo->MaxIdxCnt)
		getidx = 0;

	if (getidx == putidx)
		getidx = -1;	// Empty

	AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, getidx);

	uint8_t *p = (uint8_t*)pFifo + sizeof(CFIFOHDL) + idx * pFifo->BlkSize;
	*pCnt = cnt;

	return p;
}

uint8_t *CFifoPut(CFIFOHDL *pFifo)
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

	uint8_t *p = (uint8_t*)pFifo + sizeof(CFIFOHDL) + idx * pFifo->BlkSize;

	return p;
}

uint8_t *CFifoPutMultiple(CFIFOHDL *pFifo, int *pCnt)
{
	if (pCnt == NULL)
		return CFifoPut(pFifo);

	if (pFifo == NULL || pFifo->PutIdx == pFifo->GetIdx)
	{
		*pCnt = 0;
		return NULL;
	}

	int32_t idx = pFifo->PutIdx;
	int32_t getidx = pFifo->GetIdx;
	int32_t putidx;
	int32_t cnt;

	if (getidx < 0)
	{
		//AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, idx);
		getidx = idx;
		cnt = min(*pCnt, pFifo->MaxIdxCnt - idx);
	}
	else
	{
		if (idx < getidx)
			cnt = min(*pCnt, getidx - idx);
		else
			cnt = min(*pCnt, pFifo->MaxIdxCnt - idx);
	}

	putidx = idx + cnt;
	if (putidx >= pFifo->MaxIdxCnt)
		putidx = 0;

	AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, putidx);
	AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, getidx);

	uint8_t *p = (uint8_t*)pFifo + sizeof(CFIFOHDL) + idx * pFifo->BlkSize;

	*pCnt = cnt;

	return p;
}

void CFifoFlush(CFIFOHDL *pFifo)
{
	AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, -1);
	AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, 0);
}

int CFifoAvail(CFIFOHDL *pFifo)
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

int CFifoUsed(CFIFOHDL *pFifo)
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

