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

DiskIO::DiskIO() : vLastIdx(1), vNbCache(1), vExtCache(false)
{
	vpCacheSect = new SECTDESC[vNbCache];

	for (int i = 0; i < vNbCache; i++)
	{
		vpCacheSect[i].UseCnt = 0;
		vpCacheSect[i].SectNo = -1;
		vpCacheSect[i].pSect = new uint8_t[DISKIO_SECT_SIZE];
	}
}

void DiskIO::SetCache(uint8_t *pCacheBlk, size_t CacheSize)
{
	if (pCacheBlk == NULL || CacheSize < DISKIO_SECT_SIZE)
		return;

	if (vExtCache == false)
	{
		for (int i = 0; i < vNbCache; i++)
		{
			delete[] vpCacheSect[i].pSect;
		}
	}

	delete[] vpCacheSect;

	vNbCache = CacheSize / DISKIO_SECT_SIZE;

	vpCacheSect = new SECTDESC[vNbCache];

	for (int i = 0; i < vNbCache; i++)
	{
		vpCacheSect[i].UseCnt = 0;
		vpCacheSect[i].SectNo = -1;
		vpCacheSect[i].pSect = pCacheBlk + i * DISKIO_SECT_SIZE;
	}
}

int	DiskIO::GetCacheSect(uint32_t SectNo, bool bLock)
{
	for (int i = 0; i < DISKIO_CACHE_SECT_MAX; i++)
	{
		// Grab first cache
		vpCacheSect[i].UseCnt++;
		if (vpCacheSect[0].SectNo == SectNo)
			return i;
		// Not requested sector release it
		vpCacheSect[i].UseCnt--;
	}

	// Not in cache, try to pick unused cache
	int i = DISKIO_CACHE_SECT_MAX;

	do
	{
		vLastIdx++;
		if (vLastIdx >= DISKIO_CACHE_SECT_MAX)
			vLastIdx = 0;
		vpCacheSect[vLastIdx].UseCnt++;
		if (vpCacheSect[vLastIdx].UseCnt <= 1)
		{
			// Got unused cache
			SectRead(SectNo, vpCacheSect[vLastIdx].pSect);
			vpCacheSect[vLastIdx].SectNo = SectNo;
			return vLastIdx;
		}
		// Cache in use, release it
		vpCacheSect[vLastIdx].UseCnt--;
	} while (--i > 0);

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
		return -1;

	memcpy(pBuff, vpCacheSect[idx].pSect + SectOffset, l);

	// Done with cache sector, release it
	vpCacheSect[idx].UseCnt--;

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
		if (l < 0)
			break;
		pBuff += l;
		Len -= l;
		retval += l;
		sectno++;
		sectoff = 0;
	}

	return retval;
}

int DiskIO::Write(uint32_t SetNo, uint32_t SectOffset, uint8_t *pData, uint32_t Len)
{
	if (pData == NULL)
		return -1;

	uint32_t l = min(Len, DISKIO_SECT_SIZE - SectOffset);

	int idx = GetCacheSect(SetNo, true);
	if (idx < 0)
		return -1;

	memcpy(vpCacheSect[idx].pSect + SectOffset, pData, l);

	// Write sector to disk
	SectWrite(SetNo, vpCacheSect[idx].pSect);

	// Done with cache sector, release it
	vpCacheSect[idx].UseCnt--;

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

