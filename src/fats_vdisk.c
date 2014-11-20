/*--------------------------------------------------------------------------
File   : fatfs_vdisk.c

Author : Hoang Nguyen Hoan          Nov. 6, 2014

Desc   : FAT filesystem virtual disk
		 implementing FAT virutal file system for Flash base firmware update
		 this emulate the bootsect sector and disk structure.  The root
		 directory structure is provided by application which decides what
		 is to be on it.  Use to fool computer that there is an actual
		 removable disk.  Application decides what to do with read/write
		 request.

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/


#include <stdint.h>
#include <string.h>

#include "fatfs.h"

#define OS_REQUIRE_CLUSTERS		12

uint32_t FATFSVDiskGetClusterSector(FATFS_VDISK *pVDisk, uint32_t ClusterNo)
{
	return pVDisk->BootSect.BPB_SecPerClus * (ClusterNo - 2) + pVDisk->DataStartSector;
}

uint8_t *FATFSVDiskGetSectorData(FATFS_VDISK *pVDisk, uint32_t SectNo)
{
	uint8_t *p = NULL;

	if (SectNo == pVDisk->RootDirSect)
		p = (uint8_t*)pVDisk->pRootDir;
	if (SectNo == 0)
		p = (uint8_t*)&pVDisk->BootSect;
	if (SectNo == pVDisk->Fat1Sector)
		p = (uint8_t*)pVDisk->pFat1;

	return p;
}

bool FATFSVDiskInit(FATFS_VDISK *pVDisk, const FATFS_VDISKCFG *pCfg)
{
	if (pVDisk == NULL)
		return false;

	uint32_t nbsect, nbcluster;
	uint32_t RsvdCount, RootDirCnt;

	//memset(pVDisk->Fat32, 0, sizeof(pVDisk->Fat32));
	memset(&pVDisk->BootSect, 0, sizeof(FATFS_BSBPB));

	nbsect = (pCfg->DataSize + pCfg->SectSize - 1) / pCfg->SectSize + OS_REQUIRE_CLUSTERS;

	pVDisk->BootSect.BPB_SecPerClus = 1;
	if (nbsect >= 8192)
		pVDisk->BootSect.BPB_SecPerClus <<= 1;
	if (nbsect >= 32768)
		pVDisk->BootSect.BPB_SecPerClus <<= 1;
	if (nbsect >= 262144)
		pVDisk->BootSect.BPB_SecPerClus <<= 1;

	nbcluster = (nbsect + pVDisk->BootSect.BPB_SecPerClus - 1) / pVDisk->BootSect.BPB_SecPerClus
				;

	if (nbcluster < 4085)
		pVDisk->FatType = FATFS_TYPE_FAT12;
	else if (nbsect < 65525)
		pVDisk->FatType = FATFS_TYPE_FAT16;
	else
		pVDisk->FatType = FATFS_TYPE_FAT32;

	pVDisk->pFat1 = pCfg->pFat1;
	pVDisk->pRootDir = pCfg->pRootDir;

	// Filling Boot Sector

	pVDisk->BootSect.BS_jmpBoot[0] = 0xEB;
	pVDisk->BootSect.BS_jmpBoot[1] = 0x3C;
	pVDisk->BootSect.BS_jmpBoot[2] = 0x90;
	memcpy(pVDisk->BootSect.BS_OEMName, "BSD  4.4", 8);
	pVDisk->BootSect.BPB_BytsPerSec = pCfg->SectSize;
	pVDisk->BootSect.BPB_NumFATs = 2;
	pVDisk->BootSect.BPB_Media = FATFS_MEDIA_REMOVEABLE;

	uint32_t fatsiz;

	if (pCfg->FatType == FATFS_TYPE_FAT32)
	{
		pVDisk->BootSect.BPB_RsvdSecCnt = 32;
		pVDisk->BootSect.BPB_RootEntCnt = 0;
		pVDisk->BootSect.BPB_TotSec16 = 0;
		pVDisk->BootSect.BPB_FATSz16 = 0;
		pVDisk->BootSect.Bpb32.BS_BootSig = 0x29;
		pVDisk->BootSect.Bpb32.BS_VolID = 0x887812E3;
		memcpy(pVDisk->BootSect.Bpb32.BS_FilSysType, "FAT32   ", 8);
		memcpy(pVDisk->BootSect.Bpb32.BS_VolLab, pCfg->VolName, 11);

		fatsiz = (nbcluster + 127) / 128;
		pVDisk->BootSect.Bpb32.BPB_FATSz32 = fatsiz;

		RsvdCount = 32;
		RootDirCnt = pVDisk->BootSect.BPB_SecPerClus;
/*		pVDisk->Fat32[0] = 0xFFFFFF00 | FATFS_MEDIA_REMOVEABLE;
		pVDisk->Fat32[1] = 0xFFFFFFFF;
		pVDisk->Fat32[2] = 0xFFFFFFFF;
		pVDisk->Fat32[3] = 0xFFFFFFFF;*/

	}
	else
	{
		pVDisk->BootSect.BPB_RsvdSecCnt = 1;
		pVDisk->BootSect.BPB_RootEntCnt = 512;
		pVDisk->BootSect.BPB_TotSec32 = 0;
		pVDisk->BootSect.Bpb16.BS_BootSig = 0x29;
		pVDisk->BootSect.Bpb16.BS_VolID = 0x887812E3;
		memcpy(pVDisk->BootSect.Bpb16.BS_FilSysType, "FAT16   ", 8);
		memcpy(pVDisk->BootSect.Bpb16.BS_VolLab, pCfg->VolName, 11);

		fatsiz = (nbcluster + 255) / 256;
		pVDisk->BootSect.BPB_FATSz16 = fatsiz;

		RsvdCount = 1;
		RootDirCnt = (512 * 32 + pCfg->SectSize - 1) / pCfg->SectSize;
/*		pVDisk->Fat16[0] = 0xFF00 | FATFS_MEDIA_REMOVEABLE;
		pVDisk->Fat16[1] = 0xFF;
//		pVDisk->Fat16[2] = 0xFFFF;
//		pVDisk->Fat16[3] = 0xFFFF;
		pVDisk->Fat16[4] = 0xFFFF;*/
	}

	pVDisk->Fat1Sector = RsvdCount;
	RsvdCount += FATFS_NBFAT * fatsiz;
	pVDisk->RootDirSect = RsvdCount;
	pVDisk->DataStartSector = pVDisk->RootDirSect + RootDirCnt;
	pVDisk->TotalSectors = nbcluster * pVDisk->BootSect.BPB_SecPerClus +
							RsvdCount + RootDirCnt;
	if (pCfg->FatType == FATFS_TYPE_FAT32)
		pVDisk->BootSect.BPB_TotSec32 = pVDisk->TotalSectors;
	else
		pVDisk->BootSect.BPB_TotSec16 = pVDisk->TotalSectors;

	pVDisk->BootSect.Signature_word[0] = 0x55;
	pVDisk->BootSect.Signature_word[1] = 0xAA;

	return true;
}



