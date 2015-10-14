/*--------------------------------------------------------------------------
File   : cfifo.h

Author : Hoang Nguyen Hoan          Jan. 3, 2014

Desc   : Implementation of a simple circular FIFO buffer

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


#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>

#pragma pack(push,4)
typedef struct {
	volatile int32_t PutIdx;
	volatile int32_t GetIdx;
	int32_t MaxIdxCnt;
	uint32_t BlkSize;
	uint32_t MemSize;
	uint8_t *pMemStart;
} CFIFOHDL;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize FIFO
 *
 * @params	pMemBlk :		Pointer to memory block to be used for fifo
 * 			TotalMemSize : 	Total memory size in byte
 * 			BlkSize : 		Block size in bytes
 */
CFIFOHDL *CFifoInit(uint8_t *pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize);

/*
 * Retreive FIFO data
 *
 * @return pointer to the FIFO buffer.
 */
uint8_t *CFifoGet(CFIFOHDL *pFifo);

/*
 * Retreive FIFO data in multiple blocks
 *
 * @params	pCnt	: in - Number of block to get, out - Number of blocks returned
 *
 * @return	Pointer to first block
 */
uint8_t *CFifoGetMultiple(CFIFOHDL *pFifo, int *pCnt);

/*
 * Insert FIFO data
 *
 * @return pointer to the inserted FIFO buffer.
 */
uint8_t *CFifoPut(CFIFOHDL *pFifo);

/*
 * Insert multiple FIFO blocks
 *
 * @params	pCnt	: in - Number of block to gut, out - Number of blocks returned
 *
 * @return pointer to the inserted FIFO buffer.
 */
uint8_t *CFifoPutMultiple(CFIFOHDL *pFifo, int *pCnt);

/**
 * Flush Fifo
 */
void CFifoFlush(CFIFOHDL *pFifo);

/**
 * Get available blocks in fifo
 */
int CFifoAvail(CFIFOHDL *pFifo);

/**
 * Get occupied blocks
 */
int CFifoLen(CFIFOHDL *pFifo);

#ifdef __cplusplus
}
#endif

#endif // __FIFO_H__ 
