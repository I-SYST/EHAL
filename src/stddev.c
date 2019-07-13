/*--------------------------------------------------------------------------
File   : stddev.c

Author : Hoang Nguyen Hoan          Mar. 4, 2015

Desc   : System functions overload for libc

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
#include <stdlib.h>
#include <string.h>
#ifdef __ICCARM__
#define STDIN_FILENO    0       /* standard input file descriptor */
#define STDOUT_FILENO   1       /* standard output file descriptor */
#define STDERR_FILENO   2       /* standard error file descriptor */
#else
#include <unistd.h>
#endif

#include "stddev.h"

#define STDDEV_FDIDX_MASK		0xF
#define STDDEV_FDIDX_NBITS		4



STDDEV *g_DevTable[STDDEV_MAX] = {
	NULL,
};

int InstallBlkDev(STDDEV * const pDev, int MapId)
{
	int retval = -1;

	switch (MapId)
	{
		case STDIN_FILENO:
		case STDOUT_FILENO:
		case STDERR_FILENO:
		case STDFS_FILENO:
			retval = MapId;
			break;
		default:
			for (int i = STDDEV_USER_FILENO; i < STDDEV_MAX; i++)
			{
				if (g_DevTable[i] == NULL)
				{
					retval = i;
					break;
				}
			}

	}
	if (retval >= 0)
	{
		g_DevTable[retval] = pDev;
	}

	return retval;
}

void RemoveBlkDev(int Idx)
{
	if (Idx >=0 && Idx < STDDEV_MAX)
		g_DevTable[Idx] = NULL;
}

int _open(const char * const pPathName, int Flags, int Mode)
{
//	return g_FatFS.Open((char*)pPathName, Flags, Mode);
	if (pPathName == NULL)
		return -1;

	char *p;
	int retval = -1;

	p = strchr(pPathName, ':');

	if (p == NULL || strncmp(pPathName, "FAT:", 4) == 0)
	{
		retval = g_DevTable[STDFS_FILENO]->Open(g_DevTable[STDFS_FILENO]->pDevObj, pPathName, Flags, Mode);
		if (retval != -1)
		{
			retval = (retval << 4) | STDFS_FILENO;
		}
	}
	else
	{
		// check for named device
		for (int i = STDFS_FILENO; i < STDDEV_MAX; i++)
		{
			if (strncmp(g_DevTable[i]->Name, pPathName, 4) == 0)
			{
				retval = g_DevTable[i]->Open(g_DevTable[i]->pDevObj, pPathName, Flags, Mode);

				if (retval != -1)
				{
					retval = (retval << 4) | i;
				}
			}
		}
	}

	return retval;
}

int _close(int Fd)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx] && g_DevTable[idx]->Close)
		return g_DevTable[idx]->Close(g_DevTable[idx]->pDevObj, Fd);

	return -1;
}

int _lseek(int Fd, int Offset)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx]  && g_DevTable[idx]->Seek)
		return g_DevTable[idx]->Seek(g_DevTable[idx]->pDevObj, Fd, Offset);

	return -1;
}

int _read (int Fd, char *pBuff, size_t Len)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx] && g_DevTable[idx]->Read)
		return g_DevTable[idx]->Read(g_DevTable[idx]->pDevObj, Fd, (uint8_t*)pBuff, Len);

	return -1;
}

int _write (int Fd, char *pBuff, size_t Len)
{
	int idx = Fd & STDDEV_FDIDX_MASK;

	if (idx < 0 || idx >= STDDEV_MAX)
		return -1;

	if (idx >= STDFS_FILENO)
		Fd >>= STDDEV_FDIDX_NBITS;

	if (g_DevTable[idx] && g_DevTable[idx]->Write)
	{
		return g_DevTable[idx]->Write(g_DevTable[idx]->pDevObj, Fd, (uint8_t*)pBuff, Len);
	}

	return -1;
}
