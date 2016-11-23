/*--------------------------------------------------------------------------
File   : cfifo.h

Author : Hoang Nguyen Hoan          Jan. 3, 2014

Desc   : Implementation of an overly simple circular FIFO buffer with minimal thread safe.
		There is no Queuing implementation. The Put functions are used to get pointer to free
 	 	 block for writing. The Get functions are for retrieving data blocks.

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

typedef struct {
	volatile int32_t PutIdx;	// Idx to start of empty data block
	volatile int32_t GetIdx;	// Idx to start of used data block
	int32_t MaxIdxCnt;			// Max block count
	bool    bDrop;              // True to push out when fifo is full (drop)
	uint32_t BlkSize;			// Block size in bytes
	uint32_t MemSize;			// Total fifo memory size allocated
	uint8_t *pMemStart;			// Start of fifo data memory
} CFIFOHDR;

// defines CFIFO handle
typedef CFIFOHDR*	HCFIFO;

#define CFIFO_MEMSIZE(FSIZE)					((FSIZE) + sizeof(CFIFOHDR))
#define CFIFO_TOTAL_MEMSIZE(NbBlk, BlkSize)		((NbBlk) * (BlkSize) + sizeof(CFIFOHDR))

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize FIFO
 *
 * @params	pMemBlk :		Pointer to memory block to be used for fifo
 * 			TotalMemSize : 	Total memory size in byte
 * 			BlkSize : 		Block size in bytes
 *          bDrop   : Behavior when fifo is full.
 *                    true - Old data will be pushed out to make place
 *                           for new data
 *                    false - New data will not be pushed in
 * 	@return CFifo Handle
 */
HCFIFO CFifoInit(uint8_t *pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize, bool bDrop);

/*
 * Retrieve FIFO data by returning pointer to data block
 *
 * @param	hFifo : CFIFO handle
 *
 * @return pointer to the FIFO buffer.
 */
uint8_t *CFifoGet(HCFIFO pFifo);

/*
 * Retrieve FIFO data in multiple blocks by returning pointer to data blocks
 *
 * @param	hFifo : CFIFO handle
 * 			pCnt  : in - Number of block to get, out - Number of blocks returned
 *
 * @return	Pointer to first block
 */
uint8_t *CFifoGetMultiple(HCFIFO hFifo, int *pCnt);

/*
 * Insert FIFO data by returning pointer to memory block
 *
 * @param	hFifo : CFIFO handle
 *
 * @return pointer to the inserted FIFO buffer.
 */
uint8_t *CFifoPut(HCFIFO hFifo);

/*
 * Insert multiple FIFO blocks by returning pointer to memory blocks
 *
 * @param	hFifo : CFIFO handle
 * 			pCnt  : in - Number of block to gut, out - Number of blocks returned
 *
 * @return pointer to the inserted FIFO buffer.
 */
uint8_t *CFifoPutMultiple(HCFIFO hFifo, int *pCnt);

/*
 * Retrieve FIFO data into provided buffer
 *
 * @param	hFifo : CFIFO handle
 * 			pBuff : Pointer to buffer container for returned data
 * 			BuffLen : Size of container in bytes
 *
 * @return	Number of bytes copied into pBuff
 */
int CFifoPop(HCFIFO hFifo, uint8_t *pBuff, int BuffLen);

/*
 * Insert FIFO data with provided data
 *
 * @param	hFifo : CFIFO handle
 * 			pData : Pointer to data to be inserted
 * 			DataLen : Size of data in bytes
 *
 * @return	Number of bytes inserted into Fifo
 */
int CFifoPush(HCFIFO hFifo, uint8_t *pData, int DataLen);

/*
 * Flush Fifo
 */
void CFifoFlush(HCFIFO hFifo);

/*
 * Get available blocks in fifo
 */
int CFifoAvail(HCFIFO hFifo);

/*
 * Get number of block used blocks
 */
int CFifoUsed(HCFIFO hFifo);

#ifdef __cplusplus
}
#endif

#endif // __FIFO_H__ 
