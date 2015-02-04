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
	hdr->pMemStart = pMemBlk + sizeof(CFIFOHDL);
	hdr->MemSize = TotalMemSize;
	hdr->BlkSize = BlkSize;
	hdr->MaxIdxCnt = (TotalMemSize - sizeof(CFIFOHDL)) / BlkSize;
	hdr->PutIdx = 0;
	hdr->GetIdx = -1;

	return hdr;
}

uint8_t *CFifoGet(CFIFOHDL *pFifo)
{
	if (pFifo == NULL || pFifo->GetIdx < 0)
		return NULL;

	uint8_t *p = pFifo->pMemStart + pFifo->GetIdx * pFifo->BlkSize;

	AtomicInc((sig_atomic_t *)&pFifo->GetIdx);
	if (pFifo->GetIdx >= pFifo->MaxIdxCnt)
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, 0);

	if (pFifo->GetIdx == pFifo->PutIdx)
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, -1);	// Empty

	return p;
}

uint8_t *CFifoGetMultiple(CFIFOHDL *pFifo, int *pCnt)
{
	if (pFifo == NULL || pFifo->GetIdx < 0)
		return NULL;

	if (pCnt == NULL)
		return CFifoGet(pFifo);

	uint8_t *p = pFifo->pMemStart + pFifo->GetIdx * pFifo->BlkSize;
	int cnt = 0;

	if (pFifo->GetIdx < pFifo->PutIdx)
		cnt = min(*pCnt, pFifo->PutIdx - pFifo->GetIdx);
	else
		cnt = min(*pCnt, pFifo->MaxIdxCnt - pFifo->GetIdx);

	AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, pFifo->GetIdx + cnt);
	if (pFifo->GetIdx >= pFifo->MaxIdxCnt)
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, 0);

	if (pFifo->GetIdx == pFifo->PutIdx)
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, -1);	// Empty

	*pCnt = cnt;

	return p;
}

uint8_t *CFifoPut(CFIFOHDL *pFifo)
{
	if (pFifo == NULL || pFifo->PutIdx == pFifo->GetIdx)
		return NULL;

	uint8_t *p = pFifo->pMemStart + pFifo->PutIdx * pFifo->BlkSize;

	// If empty
	if (pFifo->GetIdx < 0)
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, pFifo->PutIdx);

	AtomicInc((sig_atomic_t *)&pFifo->PutIdx);
	if (pFifo->PutIdx >= pFifo->MaxIdxCnt)
		AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, 0);

	return p;
}

uint8_t *CFifoPutMultiple(CFIFOHDL *pFifo, int *pCnt)
{
	if (pFifo == NULL || pFifo->PutIdx == pFifo->GetIdx)
		return NULL;

	if (pCnt == NULL)
		return CFifoPut(pFifo);

	uint8_t *p = pFifo->pMemStart + pFifo->PutIdx * pFifo->BlkSize;
	int cnt = 0;

	// If empty
	if (pFifo->GetIdx < 0)
	{
		AtomicAssign((sig_atomic_t *)&pFifo->GetIdx, pFifo->PutIdx);
		cnt = min(*pCnt, pFifo->MaxIdxCnt - pFifo->PutIdx);
	}
	else
	{
		if (pFifo->PutIdx < pFifo->GetIdx)
			cnt = min(*pCnt, pFifo->GetIdx - pFifo->PutIdx);
		else
			cnt = min(*pCnt, pFifo->MaxIdxCnt - pFifo->PutIdx);
	}

	AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, pFifo->PutIdx + cnt);
	if (pFifo->PutIdx >= pFifo->MaxIdxCnt)
		AtomicAssign((sig_atomic_t *)&pFifo->PutIdx, 0);

	*pCnt = cnt;

	return p;
}

