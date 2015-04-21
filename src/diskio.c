/*--------------------------------------------------------------------------
File   : diskio.c

Author : Hoang Nguyen Hoan          Mar. 1, 2015

Desc   : Generic Disk I/O driver

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

#include "istddef.h"
#include "crc.h"
#include "diskio.h"

bool DiskIOInit(DISKIODEV *pDev, DISKIOCFG *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
		return false;


	return true;
}

uint32_t DiskIOGetSize(DISKIODEV *pDev)
{
	return 0;
}


int DiskIOSectRead(DISKIODEV *pDev, uint32_t Addr, uint8_t *pData, int len)
{
	if (pData)
	{
	}
	return 0;
}

int DiskIOSectWrite(DISKIODEV *pDev, uint32_t Addr, uint8_t *pData, int Len)
{
	if (pData)
	{
	}

	return 0;
}

