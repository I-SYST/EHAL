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
#include <stdio.h>

#include "cmsis_compiler.h"

#include "fatfs.h"

#define OS_REQUIRE_CLUSTERS		12

typedef struct {
	uint32_t DiskSize;
	FATFS_TYPE Type;
	uint32_t SecPerClus;
} FATCLUSCFG;

/*
static const FATCLUSCFG g_FatClusterSectTbl[] = {
	{4085, FATFS_TYPE_FAT12, 1},
	{8400, FATFS_TYPE_FAT16, 2},
	{32680, FATFS_TYPE_FAT16, 2},
	{262144, FATFS_TYPE_FAT16, 4},
	{524288, FATFS_TYPE_FAT16, 8},
	{16777216, FATFS_TYPE_FAT32, 8},
	{33554432, FATFS_TYPE_FAT32, 16},
	{67108864, FATFS_TYPE_FAT32, 32},
};

static const int g_NbFatClusterSectTbl = sizeof(g_FatClusterSectTbl) / sizeof(FATCLUSCFG);

static const MBR g_MbrSect = {
	{0, },
	{{0, {0,0,0}, 0xC, {0,0,0}, 1, 520},},
	0xAA55
};
*/

//FATFS_BSBPB s_FatFsBootSect;

uint32_t FATFSVDiskGetClusterSector(FATFS_VDISK *pVDisk, uint32_t ClusterNo)
{
	return pVDisk->pBootSect->SecPerClus * (ClusterNo - 2) + pVDisk->DataStartSectNo;
}

__WEAK uint8_t *FATFSVDiskGetSectorData(FATFS_VDISK *pVDisk, uint32_t SectNo, uint32_t SectOff, int Len)
{
	uint8_t *p = NULL;

	if (SectNo == pVDisk->RootDirSectNo)
		p = (uint8_t*)pVDisk->pRootDir + SectOff;
	if (SectNo == 0 && pVDisk->pMbrSect != NULL)
		p = (uint8_t*)pVDisk->pMbrSect + SectOff;
	if (SectNo == pVDisk->PartStartSectNo)
		p = (uint8_t*)&pVDisk->pBootSect + SectOff;
	if (SectNo == pVDisk->Fat1SectNo || SectNo == pVDisk->Fat2SectNo)
		p = (uint8_t*)pVDisk->pFat1 + SectOff;

	return p;
}

bool FATFSVDiskInit(FATFS_VDISK *pVDisk, const FATFS_VDISKCFG *pCfg)
{
	if (pVDisk == NULL)
		return false;

	//uint32_t nbsect, nbcluster;
	//uint32_t RsvdCount, RootDirCnt;
	//uint32_t partstartsect = 0;
	//memset(pVDisk->Fat32, 0, sizeof(pVDisk->Fat32));

	//nbsect = (pCfg->DataSize + pCfg->SectSize - 1) / pCfg->SectSize + OS_REQUIRE_CLUSTERS;
	//nbsect = pCfg->VolumeSize /  pCfg->SectSize;

	pVDisk->pMbrSect = pCfg->pMbrSect;
	pVDisk->pFat1 = pCfg->pFat1;
	pVDisk->pRootDir = pCfg->pRootDir;
//	pVDisk->pBootSect = (FATFS_BSBPB*)pCfg->pBootSec;

	if (pCfg->pBootSect != NULL)
	{
		if (pVDisk->pMbrSect != NULL)
		{
			pVDisk->PartStartSectNo = pVDisk->pMbrSect->Part[0].LBAStart;
		}
		else
			pVDisk->PartStartSectNo = 0;

		pVDisk->pBootSect = (FATFS_BSBPB*)pCfg->pBootSect;
		//memcpy(&pVDisk->pBootSect, pCfg->pBootSec, sizeof(FATFS_BSBPB));

		//uint32_t fatstartsect = pVDisk->BootSect.RsvdSecCnt;
		pVDisk->Fat1SectNo = pVDisk->PartStartSectNo + pVDisk->pBootSect->RsvdSecCnt;
		if (pVDisk->FatType == FATFS_TYPE_FAT32)
		{
			pVDisk->TotalSectors = pVDisk->pBootSect->TotSec32;
			pVDisk->Fat2SectNo = pVDisk->Fat1SectNo + pVDisk->pBootSect->BPB.Bpb32.FATSz32;
			pVDisk->DataStartSectNo = pVDisk->Fat1SectNo + pVDisk->pBootSect->NumFATs * pVDisk->pBootSect->BPB.Bpb32.FATSz32;
			pVDisk->RootDirSectNo = pVDisk->DataStartSectNo + (pVDisk->pBootSect->BPB.Bpb32.RootClus - 2) * pVDisk->pBootSect->SecPerClus;
		}
		else
		{
			pVDisk->TotalSectors = pVDisk->pBootSect->TotSec16;
			pVDisk->Fat2SectNo = pVDisk->Fat1SectNo + pVDisk->pBootSect->FATSz16;
			pVDisk->RootDirSectNo = pVDisk->Fat1SectNo + pVDisk->pBootSect->NumFATs * pVDisk->pBootSect->FATSz16;
			pVDisk->DataStartSectNo = /*pVDisk->Fat1SectNo + pVDisk->BootSect.NumFATs * pVDisk->BootSect.FATSz16 +*/
					pVDisk->RootDirSectNo + pVDisk->pBootSect->RootEntCnt * 32 / 512;
		}

		return true;
	}
/*
	else
	{
		pVDisk->pBootSect = &s_FatFsBootSect;

		memset(pVDisk->pBootSect, 0, sizeof(FATFS_BSBPB));

		pVDisk->pBootSect->SecPerClus = 1;
		if (nbsect >= 8192)
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (nbsect >= 32768)
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (nbsect >= 262144)
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (pCfg->VolumeSize > (32U*1024U*1024U))
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (pCfg->VolumeSize > (64U*1024U*1024U))
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (pCfg->VolumeSize > (128U*1024U*1024U))
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (pCfg->VolumeSize > (256U*1024U*1024U))
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (pCfg->VolumeSize > (512U*1024U*1024U))
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (pCfg->VolumeSize > (1024U*1024U*1024U))
			pVDisk->pBootSect->SecPerClus <<= 1;
		if (pCfg->VolumeSize > (2048U*1024U*1024U))
			pVDisk->pBootSect->SecPerClus <<= 1;

		nbcluster = (nbsect + pVDisk->pBootSect->SecPerClus - 1) / pVDisk->pBootSect->SecPerClus;

		if (nbcluster < 8192) //4085)
			pVDisk->FatType = FATFS_TYPE_FAT12;
		else if (nbsect < 65525)
			pVDisk->FatType = FATFS_TYPE_FAT16;
		else
			pVDisk->FatType = FATFS_TYPE_FAT32;

		// Filling Boot Sector

		pVDisk->pBootSect->JmpBoot[0] = 0xEB;
		pVDisk->pBootSect->JmpBoot[1] = 0x3C;
		pVDisk->pBootSect->JmpBoot[2] = 0x90;
		memcpy(pVDisk->pBootSect->OEMName, "BSD  4.4", 8);
		pVDisk->pBootSect->BytsPerSec = pCfg->SectSize;
		pVDisk->pBootSect->NumFATs = 2;
		pVDisk->pBootSect->Media = FATFS_MEDIA_REMOVEABLE;

		uint32_t fatsiz;

		if (pVDisk->FatType == FATFS_TYPE_FAT32)
		{
			pVDisk->pBootSect->RsvdSecCnt = 32;
			pVDisk->pBootSect->RootEntCnt = 0;
			pVDisk->pBootSect->TotSec16 = 0;
			pVDisk->pBootSect->FATSz16 = 0;
			pVDisk->pBootSect->BPB.Bpb32.BootSig = 0x29;
			pVDisk->pBootSect->BPB.Bpb32.VolID = 0x887812E3;
			memcpy(pVDisk->pBootSect->BPB.Bpb32.FilSysType, "FAT32   ", 8);
			memcpy(pVDisk->pBootSect->BPB.Bpb32.VolLab, pCfg->VolName, 11);

			fatsiz = (nbcluster + 127) / 128;
			pVDisk->pBootSect->BPB.Bpb32.FATSz32 = fatsiz;
			pVDisk->Fat2SectNo = pVDisk->Fat1SectNo + pVDisk->pBootSect->BPB.Bpb32.FATSz32;

			RsvdCount = 33;
			RootDirCnt = pVDisk->pBootSect->SecPerClus;

		}
		else
		{
			pVDisk->pBootSect->RsvdSecCnt = 1;
			pVDisk->pBootSect->RootEntCnt = 16;//512;
			pVDisk->pBootSect->TotSec32 = 0;
			pVDisk->pBootSect->BPB.Bpb16.BootSig = 0x29;
			pVDisk->pBootSect->BPB.Bpb16.VolID = 0x887812E3;
			memcpy(pVDisk->pBootSect->BPB.Bpb16.FilSysType, "FAT16   ", 8);
			memcpy(pVDisk->pBootSect->BPB.Bpb16.VolLab, pCfg->VolName, 11);

			fatsiz = (nbcluster + 255) / 256;
			pVDisk->pBootSect->FATSz16 = fatsiz;
			pVDisk->Fat2SectNo = pVDisk->Fat1SectNo + pVDisk->pBootSect->FATSz16;

			RsvdCount = 1;
			RootDirCnt = (pVDisk->pBootSect->RootEntCnt * 32 + pCfg->SectSize - 1) / pCfg->SectSize;
			//pVDisk->BootSect.RsvdSecCnt = 2 + FATFS_NBFAT * fatsiz + RootDirCnt;

		}
		pVDisk->pBootSect->Signature_word[0] = 0x55;
		pVDisk->pBootSect->Signature_word[1] = 0xAA;

		pVDisk->Fat1SectNo = RsvdCount;
		RsvdCount += FATFS_NBFAT * fatsiz;
		pVDisk->RootDirSectNo = RsvdCount;
		pVDisk->DataStartSectNo = pVDisk->RootDirSectNo + RootDirCnt;
		pVDisk->TotalSectors = nbcluster * pVDisk->pBootSect->SecPerClus -
								RsvdCount - RootDirCnt;
		if (pVDisk->FatType == FATFS_TYPE_FAT32)
			pVDisk->pBootSect->TotSec32 = pVDisk->TotalSectors;
		else
			pVDisk->pBootSect->TotSec16 = pVDisk->TotalSectors;

	//pVDisk->pFat1 = new uint16_t[fatsiz * 256];
	//memset((void *)pVDisk->pFat1, 0, fatsiz * 512);
	}
*/
	return false;
}



