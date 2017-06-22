/*--------------------------------------------------------------------------
File   : diskio.h

Author : Hoang Nguyen Hoan          Mar. 1, 2015

Desc   : Generic disk I/O driver class

Copyright (c) 2015, I-SYST, all rights reserved

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
#ifndef __DISKIO_H__
#define __DISKIO_H__

#include <stdint.h>

#define DISKIO_SECT_SIZE		    512     // Disk sector size in bytes
#define DISKIO_CACHE_SECT_MAX	    1       // Max number of cache sector
#define DISKIO_CACHE_DIRTY_BIT      (1<<31) // This bit is set in the UseCnt if there was
                                            // write to the cache

#pragma pack(push, 1)
typedef struct _DiskPartition {
	uint8_t PartState;		// Current State of Partition (00h=Inactive, 80h=Active)
	uint8_t	CHSStart[3];	// CHS Start partition
	uint8_t Type;			// Partition type
	uint8_t CHSEnd[3];		// CHS End partition
	uint32_t LBAStart;		// LBA Start partition
	uint32_t LBASize;		// Number of sectors in partition
} DISKPART;

typedef struct _MasterBootRecord {
	uint8_t Boostrap[446];		// All zeroes
	DISKPART Part[4];
	uint16_t Sig;
} MBR;

#pragma pack(pop)


#pragma pack(push, 4)


typedef struct _Cache_Desc {
	volatile int UseCnt;		                // semaphore
	uint32_t    SectNo;			                // sector number of this cache
	//uint8_t     SectData[DISKIO_SECT_SIZE];		// sector data
	uint8_t		*pSectData;		// Pointer to sector cache memory. Must be 1 sector size
} DISKIO_CACHE_DESC;

#pragma pack(pop)

#ifdef __cplusplus

class DiskIO {
public:
	DiskIO();

	virtual int GetSectSize(void) { return DISKIO_SECT_SIZE; }
	virtual uint32_t GetNbSect(void) { return GetSize() / GetSectSize(); }
	/**
	 *
	 * @return total disk size in BYTE
	 */
	virtual uint64_t GetSize(void) = 0;

	/**
	 * Read one sector from physical device
	 */
	virtual bool SectRead(uint32_t SectNo, uint8_t *pData) = 0;

	/**
	 * Write one sector to physical device
	 */
	virtual bool SectWrite(uint32_t SectNo, uint8_t *pData) = 0;
	virtual void Reset();

	/**
	 * Optional implementations.  The following Read/Write functions are
	 * implemented in with sector caching. Physical SectRead/SectWrite are
	 * called internally to flush cache as needed.
	 *
	 */
	virtual int Read(uint32_t SetNo, uint32_t SectOffset, uint8_t *pBuff, uint32_t Len);
	virtual int Read(uint64_t Offset, uint8_t *pBuff, uint32_t Len);
	virtual int Write(uint32_t SetNo, uint32_t SectOffset, uint8_t *pBuff, uint32_t Len);
	virtual int Write(uint64_t Offset, uint8_t *pBuff, uint32_t Len);

	/**
	 * Erase whole disk
	 */
	virtual void Erase() {}
	int	GetCacheSect(uint32_t SectNo, bool bLock = false);
	void SetCache(DISKIO_CACHE_DESC *pCacheBlk, int NbCacheBlk);
	void Flush();

protected:

private:
	int vLastIdx;	    // Last cache sector accessed
	int vNbCache;       // Number of cache sector
	DISKIO_CACHE_DESC *vpCacheSect;
};

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif	// __DISKIO_H__

