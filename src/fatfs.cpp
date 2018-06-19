/*--------------------------------------------------------------------------
File   : fatfs.cpp

Author : Hoang Nguyen Hoan          Mar. 1, 2015

Desc   : FAT file system object

Copyright (c) 2015, I-SYST inc., all rights reserved

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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/unistd.h>
#include <reent.h>
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>
#include <memory>
#include <wchar.h>

#include "stddev.h"
#include "sdcard.h"
#include "fatfs.h"


//#define FATFS_FDBASE_ID		0x5A00L
#define FATFS_FDIDX_MASK	0xFFL

FatFS g_FatFS;

const STDDEV g_FatFSBlkDev = {
	"FAT:",
	(void*)&g_FatFS,
	FATFSOpen,		// Open function
	FATFSClose,		// Close
	FATFSRead,		// Read
	FATFSWrite,		// Write
	FATFSSeek
};


bool FatFS::Init(DiskIO *pDiskIO)
{
	if (pDiskIO == NULL)
		return false;

	uint8_t sect[512];

	//vDiskIO = std::shared_ptr<DiskIO>(pDiskIO);
	vDiskIO = pDiskIO;
	int res = vDiskIO->Read(0, sect, 512);
	if (!res)
		return false;

	uint32_t *p = (uint32_t*)sect;
	if (*p == 0)
	{
		MBR *pmbr = (MBR*)sect;

		vPartStartSect = pmbr->Part[0].LBAStart;
		res = vDiskIO->Read(pmbr->Part[0].LBAStart * 512, sect, 512);
		if (!res)
			return false;
	}

	FATFS_BSBPB *fatbs = (FATFS_BSBPB*)sect;

	vFATStartSect = vPartStartSect + fatbs->RsvdSecCnt;
	vClusterSize = fatbs->SecPerClus;
	if (fatbs->TotSec32 && fatbs->FATSz16 == 0)
	{
		// FAT32
		vFatSize = fatbs->BPB.Bpb32.FATSz32;
		vTotalSect = fatbs->TotSec32;
		vDataStartSect = vFATStartSect + fatbs->NumFATs * fatbs->BPB.Bpb32.FATSz32;
		vRootDirSect = vDataStartSect + (fatbs->BPB.Bpb32.RootClus - 2) * fatbs->SecPerClus;
	}
	else
	{
		// FAT12 or FAT16
		vFatSize = fatbs->FATSz16;
		vTotalSect = fatbs->TotSec16;
		vRootDirSect = vFATStartSect + fatbs->NumFATs * fatbs->FATSz16;
		vDataStartSect = /*vFATStartSect + fatbs->NumFATs * fatbs->FATSz16 +*/
						vRootDirSect + fatbs->RootEntCnt * 32 / 512;
	}

	//res = vDiskIO->SectRead(vDataStartSect, sect);
/*
	res = vDiskIO->SectRead(vRootDirSect, sect);
	FATFS_DIR *dir = (FATFS_DIR *)sect;

		for (int j = 0; j < 16; j++)
		{
			if (dir->ShortName.Name[0] != 0)
			{
			//	break;
			}
			dir++;
		}*/
	InstallBlkDev((STDDEV*)&g_FatFSBlkDev, STDFS_FILENO);

	return true;
}
/*
extern "C" size_t wcstombs(char *p, const wchar_t *pw, size_t Cnt)
{
	size_t retval = 0;
	uint8_t *s = (uint8_t*)pw;

	while (Cnt-- > 0 && *s != 0)
	{
		*p = *s;
		p++;
		s+=2;
		retval++;
	}

	return retval;
}
*/
int ExtractLongName(FATFS_DIR *pDirEnt, char *pBuff)
{
	char *p = (char *)pDirEnt->LongName.Name1;
	int n = 5;
	int len = 0;

	while (n > 0 && *p != -1)
	{
		*pBuff++ = *p;
		p+=2;
		n--;
		len++;
	}
	if (n == 0)
	{
		p = (char *)pDirEnt->LongName.Name2;
		n = 6;
		while (n > 0 && *p != -1)
		{
			*pBuff++ = *p;
			p+=2;
			n--;
			len++;
		}
		if (n == 0)
		{
			p = (char *)pDirEnt->LongName.Name3;
			n = 2;
			while (n > 0 && *p != -1)
			{
				*pBuff++ = *p;
				p+=2;
				n--;
				len++;
			}
		}
	}
	if (n > 0)
		*pBuff = 0;

	return len;
}

bool FatFS::Find(char *const pPathName, DIR *pDir)
{
	char *pname;
	char tok[] = {"/"};
	//uint8_t sect[512];
	int diridx = 0;
	int nsect = vClusterSize;
	int sectno = vRootDirSect;
	bool found = false;
	char name[64];
	uint32_t clus = 0;
	char path[256];

	pDir->d_dirent.d_name[0] = '/';
	pDir->d_dirent.d_name[1] = 0;
	pDir->d_dirent.FirstClus = 2;
	pDir->d_dirent.EntrySect = vRootDirSect;
	pDir->d_dirent.EntryIdx = 0;
	pDir->d_dirent.d_type = DT_DIR;

	path[0] = 0;

	pname = strtok((char *)pPathName, tok);

	while (pname)
	{
		//FATFS_DIR dir[16];
		FATFS_DIR dir;// = (FATFS_DIR*)vSectData;
		while (nsect > 0)
		{
			uint64_t off = sectno * 512;
			//vDiskIO->SectRead(sectno, vSectData);
			for (int i = 0; i < 16; i++)
			{
				vDiskIO->Read(off, (uint8_t*)&dir, sizeof(FATFS_DIR));
				if (dir.LongName.Ord == FATFS_DIRENT_DELETED || dir.LongName.Ord == 0)
				{
					off += sizeof(FATFS_DIR);
					continue;
				}
				if (dir.ShortName.Attr == 0xf && dir.LongName.Type == 0)// &&
				//	(dir[i].LongName.Ord & FATFS_DIRENT_LASTLONG) == FATFS_DIRENT_LASTLONG)
				{
					// Long file name
					int n = dir.LongName.Ord & 0x3f;
					while (n > 0)
					{
						char *p = name + (13 * (n-1));
						ExtractLongName(&dir, p);
						i++;
						if (i > 15)
						{
							i = 0;
							sectno++;
							nsect--;
							off = sectno * 512;
						}
						else
							off += sizeof(FATFS_DIR);
						n--;
						vDiskIO->Read(off, (uint8_t*)&dir, sizeof(FATFS_DIR));
					}
				}
				else
				{
					// Short file name
					int n = 8;
					char *p = name;
					char *ps = (char *)dir.ShortName.Name;
					while (n > 0 &&  *ps != ' ')
					{
						*p++ = *ps++;
						n--;
					}
					ps = (char *)&dir.ShortName.Name[8];
					if (*ps != ' ')
					{
						*p++ = '.';
						n = 3;
						while (n > 0 && *ps != ' ')
						{
							*p++ = *ps++;
							n--;
						}
					}
					*p = 0;
				}
//				printf("Name : %s\r\n", name);
				if (strcasecmp((const char*)pname, (const char*)name) == 0)
				{
					// Found a matching entry
					//printf("Match filename\r\n");
					found = true;
					diridx = i;
					// Previous matched is the parent directory
					strcat(path, pDir->d_dirent.d_name);
					strcat(path, "/");
					pDir->DirClus = pDir->d_dirent.FirstClus;
							//(dir[diridx].ShortName.FstClusHI << 16L) | dir[diridx].ShortName.FstClusLO;
					if (dir.ShortName.Attr & FATFS_DIRATTR_DIRECTORY)
					{
						pDir->d_dirent.d_type = DT_DIR;

					}
					else
					{
						pDir->d_dirent.d_type = DT_REG;
					}
					pDir->d_dirent.FirstClus = (dir.ShortName.FstClusHI << 16L) | dir.ShortName.FstClusLO;
					pDir->d_dirent.EntrySect = sectno;
					pDir->d_dirent.EntryIdx = diridx;
					if (dir.ShortName.Attr & FATFS_DIRATTR_HIDDEN)
						pDir->d_dirent.d_att |= DA_HIDDEN;
					if (dir.ShortName.Attr & FATFS_DIRATTR_SYSTEM)
						pDir->d_dirent.d_att |= DA_SYSTEM;
					if (dir.ShortName.Attr & FATFS_DIRATTR_READ_ONLY)
						pDir->d_dirent.d_att |= DA_READONLY;


					strcpy(pDir->d_dirent.d_name, name);
					pDir->d_dirent.d_namelen = strlen(name);
					pDir->d_dirent.d_size = dir.ShortName.FileSize;
					pDir->d_dirent.d_offset = 0;
					break;
				}
				off += sizeof(FATFS_DIR);
	//				diridx++;
			}
			if (!found)
			{
				sectno++;
				nsect--;
			}
			else
			{
				pname = strtok(NULL, tok);
				if (pname == NULL)
				{
					pDir->d_dirnamelen = strlen(path);
					pDir->d_dirname = new char[pDir->d_dirnamelen + 1];//(char *)malloc(pDir->d_dirnamelen + 1);
					strcpy(pDir->d_dirname, path);
					break;
				}
				sectno = ClusToSect(clus);
				nsect = vClusterSize;
				found = false;
				clus = 0;
			}
		}
		if (pname && nsect <= 0)
			return false;
	}
//printf("Found = %x\r\n", found);
	return found;
}

bool FatFS::FindFreeDirEntry()
{
	int nsect = vClusterSize;
	int sectno = vRootDirSect;
	bool found = false;
	bool retval = false;

	FATFS_DIR dir;

	while (nsect > 0)
	{
		uint64_t off = sectno * 512;
		//vDiskIO->SectRead(sectno, vSectData);
		for (int i = 0; i < 16; i++)
		{
			vDiskIO->Read(off, (uint8_t*)&dir, sizeof(FATFS_DIR));
			if (dir.LongName.Ord == FATFS_DIRENT_DELETED || dir.LongName.Ord == 0)
			{
				off += sizeof(FATFS_DIR);
				found = true;
				break;
			}

			off += sizeof(FATFS_DIR);
		}

		retval = found;

		if (found)
		{
			break;
		}
		sectno++;
		nsect--;
	}

	return retval;
}

int FatFS::Create()
{
	int retval = -1;

	return retval;
}

int FatFS::Open(char * const pPathName, int Flags, int Mode)
{
	DIR dirinfo;
	FATFS_FD *fatfd = NULL;
	int fd = 0;//FATFS_FDBASE_ID;

	// Find empty slot
	for (int i = 0; i < MAX_FILE; i++)
	{
		if (vOpenFiles[i].pFs == NULL)
		{
			fatfd = &vOpenFiles[i];
			fd |= i;
			break;
		}
	}

	if (fatfd == NULL)
	{
		//printf("NO FD available to Open File\n\r");
		// No more file handle available
		return -1;
	}

	memset(fatfd, 0, sizeof(FATFS_FD));

	switch (Flags)
	{
		case O_RDONLY:
			break;
		case O_WRONLY:
			break;
		case O_APPEND:
			break;
		case O_CREAT:
			break;
		case O_SYNC:
			break;
	}

	if (Find(pPathName, &fatfd->DirEntry))
	{
		// File found
		if (Flags & O_CREAT)
		{
			return -1;
		}

		if (fatfd->DirEntry.d_dirent.d_type == DT_DIR)
			return -1;

		fatfd->pFs = (void*)this;
		fatfd->CurClus = fatfd->DirEntry.d_dirent.FirstClus;
		fatfd->SectIdx = 0;
		fatfd->SectOff = 0;
		//uint32_t sectno = ClusToSect(fatfd->CurClus) + fatfd->SectIdx;
		//vDiskIO->SectRead(sectno, fatfd->SectData);
		fatfd->DirEntry.d_dirent.d_offset = 0;

		return fd;
	}
	else
	{
		// File does not exist
		if ((Flags & O_CREAT) == 0)
		{
			return -1;
		}

		Create();
	}

	return -1;
}

int FatFS::Close(int Fd)
{
//	uint8_t sect[512];
	FATFS_FD *fatfd = &vOpenFiles[Fd & FATFS_FDIDX_MASK];
	//FATFS_DIR *fatdir = (FATFS_DIR*)vSectData;
	int idx = fatfd->DirEntry.d_dirent.EntryIdx;

	if (fatfd->pFs == NULL || fatfd->pFs != this)
		return -1;

	FATFS_DIR fatdir;
	uint32_t sectno = ClusToSect(fatfd->DirEntry.DirClus);
	uint64_t off = (uint64_t)sectno + idx * sizeof(FATFS_DIR);
	//vDiskIO->SectRead(fatfd->DirEntry.d_dirent.EntrySect, vSectData);
	//vDiskIO->Read(off, (uint8_t*)&fatdir, sizeof(FATFS_DIR));
	fatdir.ShortName.FileSize = fatfd->DirEntry.d_dirent.d_size;
	fatdir.ShortName.FstClusLO = fatfd->DirEntry.d_dirent.FirstClus & 0xFFFF;
	fatdir.ShortName.FstClusHI = (fatfd->DirEntry.d_dirent.FirstClus >> 16L) & 0xFFFF;
	delete fatfd->DirEntry.d_dirname;
	fatfd->DirEntry.d_dirname = NULL;
	//vDiskIO->SectWrite(fatfd->DirEntry.d_dirent.EntrySect, sect);
	vDiskIO->Write(off, (uint8_t *)&fatdir, sizeof(FATFS_DIR));
	fatfd->pFs = NULL;	// Close handle

	return 0;
}

int FatFS::Read(int Fd, uint8_t *pBuff, size_t Len)
{
	FATFS_FD *fatfd = &vOpenFiles[Fd & FATFS_FDIDX_MASK];
	DIR *pdir = &fatfd->DirEntry;
	int retval = 0;
	int sectno;

	if (fatfd->pFs == this)
	{
		sectno = ClusToSect(fatfd->CurClus) + fatfd->SectIdx;
//printf("sectno %d %d\r\n", sectno, fatfd->CurClus);
		while (Len > 0)
		{
			uint32_t c = std::min(FATFS_SECTOR_SIZE - fatfd->SectOff, (uint32_t)Len);
			c = std::min(c, pdir->d_dirent.d_size - pdir->d_dirent.d_offset);
//printf("read len %d, %d\r\n", Len, c);
			if (c > 0)
			{
				uint64_t off = (uint64_t)sectno * FATFS_SECTOR_SIZE + fatfd->SectOff;
				vDiskIO->Read(off, pBuff, c);
				Len -= c;
				retval += c;
				fatfd->SectOff += c;
				pdir->d_dirent.d_offset += c;
				pBuff += c;
			}
			if (fatfd->SectOff >= FATFS_SECTOR_SIZE)// && pdir->d_dirent.d_offset < pdir->d_dirent.d_size)
			{
				sectno++;
				fatfd->SectIdx++;
				if (fatfd->SectIdx >= vClusterSize)
				{
					fatfd->SectIdx = 0;
					uint64_t off = vFATStartSect * 512 + fatfd->CurClus * sizeof(uint32_t);
					uint32_t x = 0;
					vDiskIO->Read(off, (uint8_t*)&x, sizeof(uint32_t));
					if (x > 0 && x != FATFS_FATENTRY_ALLOCATED)
						fatfd->CurClus = x;
					sectno = ClusToSect(fatfd->CurClus) + fatfd->SectIdx;
				}
				fatfd->SectOff = 0;
			}
			//else
			//	break;
			if (pdir->d_dirent.d_offset >= pdir->d_dirent.d_size)
				break;
		}
	}
	return retval;
}

int FatFS::Write(int Fd, uint8_t *pBuff, size_t Len)
{
	FATFS_FD *fatfd = &vOpenFiles[Fd & FATFS_FDIDX_MASK];
	DIR *pdir = &fatfd->DirEntry;

	if (fatfd->pFs == this)
	{

	}
	return 0;
}

uint32_t FatFS::ClusToSect(uint32_t ClusNo)
{
	return vClusterSize * (ClusNo - 2) + vDataStartSect;
}

bool FATFSInit(void *pDiskIO)
{
	return g_FatFS.Init((DiskIO*)pDiskIO);
}

int FATFSOpen(void *pDevObj, const char *pPathName, int Flags, int Mode)
{
	int retval = g_FatFS.Open((char*)pPathName, Flags, Mode);

	return retval;
}

int FATFSClose(void *pDevObj, int Fd)
{
	return g_FatFS.Close(Fd);
}

int FATFSSeek(void *pDevObj, int Fd, int Offset)
{
	return -1;
}

int FATFSRead(void *pDevObj, int Fd, uint8_t *pBuff, size_t Len)
{
	return g_FatFS.Read(Fd, pBuff, Len);
}

int FATFSWrite(void *pDevObj, int Fd, uint8_t *pBuff, size_t Len)
{
	return g_FatFS.Write(Fd, pBuff, Len);
}


