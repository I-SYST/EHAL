/**--------------------------------------------------------------------------
@file 	cfifo.h

@brief	Implementation of an overly simple circular FIFO buffer.

There is no queuing implementation and non blocking to be able to be use in
interrupt. User must ensure thread safety when used in a threaded environment.

@author Hoang Nguyen Hoan
@date 	Jan. 3, 2014

@license

Copyright (c) 2014, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/

#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>

/** @addtogroup FIFO
  * @{
  */

#pragma pack(push,4)

/// Header defining a circular fifo memory block.
typedef struct __CFIFO_Header {
	volatile int32_t PutIdx;	//!< Index to start of empty data block
	volatile int32_t GetIdx;	//!< Index to start of used data block
	int32_t MaxIdxCnt;			//!< Max block count
	bool    bBlocking;          //!< False to push out when FIFO is full (drop)
	uint32_t DropCnt;           //!< Count dropped block
	uint32_t BlkSize;			//!< Block size in bytes
	uint32_t MemSize;			//!< Total FIFO memory size allocated
	uint8_t *pMemStart;			//!< Start of FIFO data memory
} CFIFOHDR;

#pragma pack(pop)

/// @brief	CFIFO handle.
///
/// This handle is used for all CFIFO function calls. It is the pointer to to CFIFO memory block.
///
typedef CFIFOHDR* HCFIFO;

/// This macro calculates total memory require in bytes including header for byte based FIFO.
#define CFIFO_MEMSIZE(FSIZE)					((FSIZE) + sizeof(CFIFOHDR))

/// This macro calculates total memory require in bytes including header for block based FIFO.
#define CFIFO_TOTAL_MEMSIZE(NbBlk, BlkSize)		((NbBlk) * (BlkSize) + sizeof(CFIFOHDR))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize FIFO.
 *
 * This function must be called first to initialize FIFO before any other functions
 * can be used.
 *
 * @param	pMemBlk 		: Pointer to memory block to be used for FIFO
 * @param	TotalMemSize	: Total memory size in byte
 * @param	BlkSize 		: Block size in bytes
 * @param   bBlocking  		: Behavior when FIFO is full.\n
 *                    			false - Old data will be pushed out to make place
 *                            			for new data.\n
 *                    			true  - New data will not be pushed in
 *
 * 	@return CFifo Handle
 */
HCFIFO const CFifoInit(uint8_t * const pMemBlk, uint32_t TotalMemSize, uint32_t BlkSize, bool bBlocking);

/**
 * @brief	Retrieve FIFO data by returning pointer to FIFO memory block for reading.
 *
 * This function returns a direct pointer to FIFO memory to quickly retrieve data.
 * User must ensure to transfer data quickly to avoid data being overwritten by a
 * new FIFO put. This is to allows FIFO handling within interrupt.
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Pointer to the FIFO buffer.
 */
uint8_t *CFifoGet(HCFIFO const pFifo);

/**
 * @brief	Retrieve FIFO data in multiple blocks by returning pointer to FIFO memory blocks
 * for reading.
 *
 * This function returns a direct pointer to FIFO memory to quickly retrieve data.
 * User must ensure to transfer data quickly to avoid data being overwritten by a
 * new FIFO put. This is to allows FIFO handling within interrupt.
 *
 * @param	hFifo : CFIFO handle
 * @param	pCnt  : Number of block to get\n
 * 					On return number of blocks available to read
 *
 * @return	Pointer to first FIFO block. Blocks are consecutive.
 */
uint8_t *CFifoGetMultiple(HCFIFO const hFifo, int *pCnt);

/**
 * @brief	Insert FIFO data by returning pointer to FIFO memory block for writing.
 *
 * @param	hFifo : CFIFO handle
 *
 * @return pointer to the inserted FIFO buffer.
 */
uint8_t *CFifoPut(HCFIFO const hFifo);

/**
 * @brief	Insert multiple FIFO blocks by returning pointer to memory blocks for writing.
 *
 * @param	hFifo : CFIFO handle
 * @param	pCnt  : Number of block to put\n
 * 					On return pCnt contains the number of blocks available for writing
 *
 * @return	pointer to the first FIFO block. Blocks are consecutive.
 */
uint8_t *CFifoPutMultiple(HCFIFO const hFifo, int *pCnt);

/**
 * @brief	Retrieve FIFO data into provided buffer
 *
 * @param	hFifo : CFIFO handle
 * @param	pBuff : Pointer to buffer container for returned data
 * @param	BuffLen : Size of container in bytes
 *
 * @return	Number of bytes copied into pBuff
 */
int CFifoPop(HCFIFO const hFifo, uint8_t *pBuff, int BuffLen);

/**
 * @brief	Insert FIFO data with provided data
 *
 * @param	hFifo : CFIFO handle
 * @param	pData : Pointer to data to be inserted
 * @param	DataLen : Size of data in bytes
 *
 * @return	Number of bytes inserted into FIFO
 */
int CFifoPush(HCFIFO const Fifo, uint8_t *pData, int DataLen);

/**
 * @brief	Reset FIFO
 *
 * @param	hFifo : CFIFO handle
 */
void CFifoFlush(HCFIFO const hFifo);

/**
 * @brief	Get available blocks in FIFO
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Number of FIFO block available for writing
 */
int CFifoAvail(HCFIFO const hFifo);

/**
 * @brief	Get number of block used blocks
 *
 * @param	hFifo : CFIFO handle
 *
 * @return	Number of FIFO block used
 */
int CFifoUsed(HCFIFO const hFifo);

#ifdef __cplusplus
}
#endif

/** @} end group FIFO */

#endif // __FIFO_H__ 
