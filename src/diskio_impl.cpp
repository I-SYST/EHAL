/*--------------------------------------------------------------------------
File   : diskio.c

Author : Hoang Nguyen Hoan          Mar. 1, 2015

Desc   : Generic Disk I/O driver class

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

#include <string.h>
#include <stdio.h>
#include <atomic>

#include "istddef.h"
#include "crc.h"
#include "diskio.h"

using namespace std;

DiskIO::DiskIO() : vLastIdx(1), vNbCache(0), vpCacheSect(NULL)
{
}

void DiskIO::SetCache(DISKIO_CACHE_DESC *pCacheBlk, int NbCacheBlk)
{
	if (pCacheBlk == NULL || NbCacheBlk <= 0)
		return;

	vNbCache = NbCacheBlk;

	vpCacheSect = pCacheBlk;

	for (int i = 0; i < vNbCache; i++)
	{
		vpCacheSect[i].UseCnt = 0;
		vpCacheSect[i].SectNo = -1;
	}
}

void DiskIO::Reset()
{
	for (int i = 0; i < vNbCache; i++)
	{
		vpCacheSect[i].UseCnt = 0;
		vpCacheSect[i].SectNo = -1;
	}
}

int	DiskIO::GetCacheSect(uint32_t SectNo, bool bLock)
{
    // Try to find sector in cache
	for (int i = 0; i < vNbCache; i++)
	{
		// Grab first cache
		vpCacheSect[i].UseCnt++;
		if (vpCacheSect[0].SectNo == SectNo)
			return i;
		// Not requested sector release it
		vpCacheSect[i].UseCnt--;
	}

	// Not in cache, try to pick unused cache
	int i = vNbCache;

	while (i > 0)
	{
		vLastIdx++;

		if (vLastIdx >= vNbCache)
			vLastIdx = 0;

		if ((vpCacheSect[vLastIdx].UseCnt & ~DISKIO_CACHE_DIRTY_BIT) == 0)
		{
			// Got unused cache

		    // Flush cache is dirty
		    if (vpCacheSect[vLastIdx].UseCnt & DISKIO_CACHE_DIRTY_BIT)
		        SectWrite(vpCacheSect[vLastIdx].SectNo, vpCacheSect[vLastIdx].SectData);

	        vpCacheSect[vLastIdx].UseCnt = 1;

	        // Fill cache
			SectRead(SectNo, vpCacheSect[vLastIdx].SectData);

			vpCacheSect[vLastIdx].SectNo = SectNo;
			return vLastIdx;
		}
		i--;
	}

	// No Cache avail
	return -1;
}

int DiskIO::Read(uint32_t SectNo, uint32_t SectOffset, uint8_t *pBuff, uint32_t Len)
{
	if (pBuff == NULL)
		return -1;

	int l = min(Len, DISKIO_SECT_SIZE - SectOffset);

	int idx = GetCacheSect(SectNo);
	if (idx < 0)
	{
	    // No cache, do physical read
	    uint8_t d[DISKIO_SECT_SIZE];
	    SectRead(SectNo, d);
	    memcpy(pBuff, d + SectOffset, l);
	}
	else
	{
	    // Get it from cache
	    memcpy(pBuff, vpCacheSect[idx].SectData + SectOffset, l);

	    // Done with cache sector, release it
	    vpCacheSect[idx].UseCnt--;
	}

	return l;
}

int DiskIO::Read(uint64_t Offset, uint8_t *pBuff, uint32_t Len)
{
	uint64_t sectno = Offset / DISKIO_SECT_SIZE;
	uint32_t sectoff = Offset % DISKIO_SECT_SIZE;

	uint32_t retval = 0;

	while (Len > 0)
	{
		int l = Read(sectno, sectoff, pBuff, Len);
		if (l <= 0)
			break;
		pBuff += l;
		Len -= l;
		retval += l;
		sectno++;
		sectoff = 0;
	}

	return retval;
}

int DiskIO::Write(uint32_t SectNo, uint32_t SectOffset, uint8_t *pData, uint32_t Len)
{
	if (pData == NULL)
		return -1;

	uint32_t l = min(Len, DISKIO_SECT_SIZE - SectOffset);

	int idx = GetCacheSect(SectNo, true);
	if (idx < 0)
	{
	    // No cache, do physical write
	    uint8_t d[DISKIO_SECT_SIZE];
	    SectRead(SectNo, d);
	    memcpy(d + SectOffset, pData, l);
	    SectWrite(SectNo, d);
	}
	else
	{
	    // Write to cache
        memcpy(vpCacheSect[idx].SectData + SectOffset, pData, l);

        // Done with cache sector, release it
        vpCacheSect[idx].UseCnt |= DISKIO_CACHE_DIRTY_BIT;
        vpCacheSect[idx].UseCnt--;
	}

	return l;
}

int DiskIO::Write(uint64_t Offset, uint8_t *pData, uint32_t Len)
{
	uint64_t sectno = Offset / DISKIO_SECT_SIZE;
	uint32_t sectoff = Offset % DISKIO_SECT_SIZE;

	uint32_t retval = 0;

	while (Len > 0)
	{
		int l = Write(sectno, sectoff, pData, Len);
		if (l < 0)
			break;
		pData += l;
		Len -= l;
		retval += l;
		sectno++;
		sectoff = 0;
	}

	return retval;
}

void DiskIO::Flush()
{
    for (int i = 0; i < vNbCache; i++)
    {
        if (vpCacheSect[i].UseCnt & DISKIO_CACHE_DIRTY_BIT)
        {
            SectWrite(vpCacheSect[i].SectNo, vpCacheSect[i].SectData);
            vpCacheSect[i].UseCnt &= ~DISKIO_CACHE_DIRTY_BIT;
        }
    }
}
