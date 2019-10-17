/**-------------------------------------------------------------------------
@file	diskio.h

@brief	Generic disk I/O driver class

@author	Hoang Nguyen Hoan
@date	Mar. 1, 2015

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __DISKIO_H__
#define __DISKIO_H__

#include <stdint.h>

/** @addtogroup Storage
  * @{
  */

#define DISKIO_SECT_SIZE		    512     //!< Disk sector size in bytes
#define DISKIO_CACHE_SECT_MAX	    1       //!< Max number of cache sector
#define DISKIO_CACHE_DIRTY_BIT      (1<<31) //!< This bit is set in the UseCnt if there was
                                            //!< write to the cache

#pragma pack(push, 1)
typedef struct __DiskPartition {
	uint8_t PartState;		//!< Current State of Partition (00h=Inactive, 80h=Active)
	uint8_t	CHSStart[3];	//!< CHS Start partition
	uint8_t Type;			//!< Partition type
	uint8_t CHSEnd[3];		//!< CHS End partition
	uint32_t LBAStart;		//!< LBA Start partition
	uint32_t LBASize;		//!< Number of sectors in partition
} DISKPART;

typedef struct __MasterBootRecord {
	uint8_t Boostrap[446];	// All zeroes
	DISKPART Part[4];
	uint16_t Sig;
} MBR;

#pragma pack(pop)


#pragma pack(push, 4)

/// DiskIO cache descriptor
typedef struct __DiskIO_Cache_Desc {
	volatile int UseCnt;	//!< semaphore
	uint32_t    SectNo;		//!< sector number of this cache
	uint8_t		*pSectData;	//!< Pointer to sector cache memory. Must be at least 1 sector size
} DISKIO_CACHE_DESC;

#pragma pack(pop)

#ifdef __cplusplus

/// DiskIO base class
class DiskIO {
public:
	DiskIO();

	/**
	 * @brief	Get the size of one sector.
	 *
	 * @return	Sector size in bytes.
	 */
	virtual int GetSectSize(void) { return DISKIO_SECT_SIZE; }

	/**
	 * @brief	Get total number of sect.
	 *
	 * @return	Number of sectors
	 */
	virtual uint32_t GetNbSect(void) { return GetSize() / GetSectSize(); }

	/**
	 * @brief	Get to=tal disk size in bytes.
	 *
	 * @return	Total disk size in bytes.
	 */
	virtual uint64_t GetSize(void) = 0;

	/**
	 * @brief	Read one sector from physical device.
	 *
	 * @param	SectNo	: Sector number to read
	 * @param	pBuff	: Buffer to receive sector data. This buffer must be at least
	 * 					  1 sector in size.
	 *
	 * @return
	 * 			- true  : Success
	 * 			- false : Failed
	 */
	virtual bool SectRead(uint32_t SectNo, uint8_t *pBuff) = 0;

	/**
	 * @brief	Write one sector to physical device.
	 *
	 * @param	SectNo	: Sector number to read
	 * @param	pDate	: Sector data to write. This must be at least
	 * 					  1 sector in size.
	 *
	 * @return
	 * 			- true  : Success
	 * 			- false : Failed
	 */
	virtual bool SectWrite(uint32_t SectNo, uint8_t *pData) = 0;

	/**
	 * @brief	Reset DiskIO to its default state
	 */
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
	 * @brief	Erase whole disk
	 */
	virtual void Erase() {}

	int	GetCacheSect(uint32_t SectNo, bool bLock = false);
	void SetCache(DISKIO_CACHE_DESC * const pCacheBlk, int NbCacheBlk);
	void Flush();

protected:

private:
	int vLastIdx;	    //!< Last cache sector accessed
	int vNbCache;       //!< Number of cache sector
	DISKIO_CACHE_DESC *vpCacheSect;	//!< pointer to static disk cache
};

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/** @} End of group Storage */

#endif	// __DISKIO_H__

