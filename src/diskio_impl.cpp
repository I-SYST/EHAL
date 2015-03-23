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
#include <thread>

#include "istddef.h"
#include "crc.h"
#include "diskio.h"

using namespace std;

DiskIO::DiskIO() : vLastIdx(1)
{
	for (int i = 0; i < DISKIO_CACHE_SECT_MAX; i++)
	{
		vCacheSect[i].UseCnt = 0;
		vCacheSect[i].SectNo = -1;
	}
}

int	DiskIO::GetCacheSect(uint32_t SectNo, bool bLock)
{
	for (int i = 0; i < DISKIO_CACHE_SECT_MAX; i++)
	{
		// Grab first cache
		vCacheSect[i].UseCnt++;
		if (vCacheSect[0].SectNo == SectNo)
			return i;
		// Not requested sector release it
		vCacheSect[i].UseCnt--;
	}

	// Not in cache, try to pick unused cache
	int i = DISKIO_CACHE_SECT_MAX;

	do
	{
		vLastIdx++;
		if (vLastIdx >= DISKIO_CACHE_SECT_MAX)
			vLastIdx = 0;
		vCacheSect[vLastIdx].UseCnt++;
		if (vCacheSect[vLastIdx].UseCnt <= 1)
		{
			// Got unused cache
			SectRead(SectNo, vCacheSect[vLastIdx].Sect);
			vCacheSect[vLastIdx].SectNo = SectNo;
			return vLastIdx;
		}
		// Cache in use, release it
		vCacheSect[vLastIdx].UseCnt--;
	} while (--i > 0);

	// No Cache avail
	return -1;
}

int DiskIO::Read(uint64_t Offset, uint8_t *pBuff, uint32_t Len)
{
	if (pBuff == NULL)
		return 0;

	uint32_t retval = -1;

	uint64_t sectno = Offset / DISKIO_SECT_SIZE;
	uint32_t sectoff = Offset % DISKIO_SECT_SIZE;

	retval = 0;

	while (Len > 0)
	{
		uint32_t l = std::min(Len, DISKIO_SECT_SIZE - sectoff);

		int idx = GetCacheSect(sectno);
		if (idx < 0)
			break;

		memcpy(pBuff, vCacheSect[idx].Sect + sectoff, l);

		// Done with cache sector, release it
		vCacheSect[idx].UseCnt--;

		pBuff += l;
		Len -= l;
		retval += l;
		sectno++;
		sectoff = 0;
	}

	return retval;
}

int DiskIO::Write(uint64_t Offset, uint8_t *pData, uint32_t Len)
{
	if (pData == NULL)
		return 0;

	uint32_t retval = -1;
	uint64_t sectno = Offset / DISKIO_SECT_SIZE;
	uint32_t sectoff = Offset % DISKIO_SECT_SIZE;

	retval = 0;

	while (Len > 0)
	{
		uint32_t l = std::min(Len, DISKIO_SECT_SIZE - sectoff);

		int idx = GetCacheSect(sectno, true);
		if (idx < 0)
			break;

		memcpy(vCacheSect[idx].Sect + sectoff, pData, l);

		// Write sector to disk
		SectWrite(sectno, vCacheSect[idx].Sect);

		// Done with cache sector, release it
		vCacheSect[idx].UseCnt--;

		pData += l;
		Len -= l;
		retval += l;
		sectno++;
		sectoff = 0;
	}

	return retval;
}

