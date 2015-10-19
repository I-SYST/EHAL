/*--------------------------------------------------------------------------
File   : fatfs.h

Author : Hoang Nguyen Hoan          Nov. 3, 2014

Desc   : FAT file system definitions

Copyright (c) 2014, I-SYST, all rights reserved

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
#ifndef __FATFS_H__
#define __FATFS_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dirent.h"

// FAT type
typedef enum {
	FATFS_TYPE_FAT12,
	FATFS_TYPE_FAT16,
	FATFS_TYPE_FAT32
} FATFS_TYPE;

#define FATFS_SECTOR_SIZE			512		// Number of bytes per sector
#define FATFS_NBFAT					2
#define FATFS_RSVDSECCNT_FAT16		1
#define FATFS_RSVDSECCNT_FAT32		32
#define FATFS_ROOTENTCNT_FAT16		512
#define FATFS_ROOTENTCNT_FAT32		0

#define FATFS_MEDIA_FIXED			0xF8
#define FATFS_MEDIA_REMOVEABLE		0xF0

#define FATFS_FAT16_CLNSHUT_BITMASK	0x8000
#define FATFS_FAT16_HRDERR_BITMASK	0x4000

#define FATFS_FAT32_CLNSHUT_BITMASK	0x80000000L
#define FATFS_FAT32_HRDERR_BITMASK	0x40000000L

#ifndef MAX_FILE
#define MAX_FILE					OPEN_MAX
#endif

#pragma pack(push, 1)

typedef enum _FATFS_FAT_Entry_Value {
	FATFS_FATENTRY_FREE	= 0,
	FATFS_FATENTRY_BAD = 0xFFFFFF7,
	FATFS_FATENTRY_ALLOCATED = 0xFFFFFFFF
} FATFS_FATENTRY;

typedef struct _FATFS_BootSector_BPB {
	uint8_t 	JmpBoot[3]; 	// Jump instruction to boot code. This field has two allowed forms:
								// jmpBoot[0] = 0xEB, jmpBoot[1] = 0x??, jmpBoot[2] = 0x90
								// and jmpBoot[0] = 0xE9, jmpBoot[1] = 0x??, jmpBoot[2] = 0x??
								// 0x?? indicates that any 8-bit value is allowed in that byte.
								// What this forms is a three-byte Intel x86 unconditional
								// branch (jump) instruction that jumps to the start of the operating
								// system bootstrap code.
	uint8_t 	OEMName[8];		// "MSWIN4.1" is the recommended setting, because it is the setting
								// least likely to cause compatibility problems.
	uint16_t 	BytsPerSec;		// Count of bytes per sector. 512, 1024, 2048 or 4096. If maximum
								// compatibility 512 is recommended
	uint8_t		SecPerClus;		// Number of sectors per allocation unit. Must be a power
								// of 2 that is greater than 0 where SecPerClus * BytesPerSec <= 32KB
	uint16_t	RsvdSecCnt; 	// Number of reserved sectors in the Reserved.
								// 		1 - FAT12/FAT16,
								// 		32 - FAT32
	uint8_t		NumFATs;		// The count of FAT data structures on the volume.
								// 		This field should always contain the value 2
	uint16_t	RootEntCnt; 	// Number of Root Directory Entries in multiple of 32 bytes.
								// 		512 - for FAT16, 0 - for FAT32
	uint16_t	TotSec16;		// This field is the old 16-bit total count of sectors on the volume.
								// 		0 for FAT32
	uint8_t		Media;			// 0xF8 - “fixed” (non-removable) media.
								// 0xF0 - For removable media
	uint16_t	FATSz16;		// FAT12/FAT16 : 16-bit count of sectors occupied by ONE FAT
								// FAT32 : must be 0
	uint16_t	SecPerTrk;		// Sectors per track for interrupt 0x13
	uint16_t	NumHeads;		// Number of heads for interrupt 0x13.
	uint32_t	HiddSec;		// Count of hidden sectors preceding the partition that contains
								// this FAT volume. This field is generally only relevant for media
								// visible on interrupt 0x13.
	uint32_t	TotSec32;		// FAT32 total count of sectors on the volume.
	union {
		struct {	// _FAT16_BPB
			uint8_t DrvNum;		// Interrupt 0x13 drive number. Set value to 0x80 or 0x00.
			uint8_t Reserved1;	// Reserved. Set value to 0x0.
			uint8_t BootSig;	// Extended boot signature. Set value to 0x29 if either of
								// the following two fields are non-zero.
			uint32_t VolID;		// Volume serial number. This ID should be generated by simply
								// combining the current date and time into a 32-bit value.
			uint8_t	VolLab[11];	// Volume label. This field matches the 11-byte volume
									// label recorded in the root directory.
			uint8_t FilSysType[8];	// One of the strings “FAT12 ”, “FAT16 ”, or “FAT ”.
									// NOTE: This string is informational only and does not
									// determine the FAT type.
		} Bpb16;
		struct { 	//_FAT32_BPB
			uint32_t	FATSz32;	// This field is the FAT32 32-bit count of sectors occupied
									// by one FAT. Note that BPB_FATSz16 must be 0 for media
									// formatted FAT32.
			uint16_t	ExtFlags;	// Bits0-3 -- Zero-based number of active FAT.
									//			  Only valid if mirroring is disabled.
									// Bits 4-6 -- Reserved.
									// Bit 7 -- 0 means the FAT is mirrored at runtime into all FATs.
									// 			1 means only one FAT is active; it is the one
									//			  referenced in bits 0-3.
									// Bits 8-15 -- Reserved.
			uint16_t	FSVer;		// High byte is major revision number.
											// Low byte is minor revision number.
											// Must be set to 0x0.
			uint32_t	RootClus;	// This is set to the cluster number of the first cluster
									// of the root directory. This value should be 2 or the first
									// usable (not bad) cluster available thereafter.
			uint16_t	FSInfo;		// Sector number of FSINFO structure in the reserved area of
									// the FAT32 volume. Usually 1.
			uint16_t	BkBootSec;	// Set to 0 or 6. If non-zero, indicates the sector number
									// in the reserved area of the volume of a copy of the boot record.
			uint8_t		Reserved[12];	// Reserved. Must be set to 0x0.
			uint8_t		DrvNum;		// Interrupt 0x13 drive number. Set value to 0x80 or 0x00.
			uint8_t		Reserved1;	// Reserved. Set value to 0x0.
			uint8_t		BootSig;	// Extended boot signature. Set value to 0x29 if either of the
									// following two fields are non-zero.
			uint32_t	VolID;		// Volume serial number. This ID should be generated by simply
									// combining the current date and time into a 32-bit value.
			uint8_t		VolLab[11];	// Volume label. This field matches the 11-byte volume label
									// recorded in the root directory.
			uint8_t		FilSysType[8];	// Set to the string:”FAT32 ”.
										// NOTE: This string is informational only and does not
										// determine the FAT type.
		} Bpb32;
	};
	uint8_t	Blank[420];
	uint8_t	Signature_word[2];			// Set to 0x55 (at byte offset 510) and 0xAA (at byte offset 511)
} FATFS_BSBPB;

typedef enum _DIR_Attr {
	FATFS_DIRATTR_READ_ONLY = 1, 		// 0x01 - ATTR_READ_ONLY
	FATFS_DIRATTR_HIDDEN 	= 2,		// 0x02 - ATTR_HIDDEN
	FATFS_DIRATTR_SYSTEM	= 4,		// 0x04	- ATTR_SYSTEM
	FATFS_DIRATTR_VOLUME_ID	= 8,		// 0x08 - ATTR_VOLUME_ID
	FATFS_DIRATTR_DIRECTORY	= 0x10,		// 0x10 - ATTR_DIRECTORY
	FATFS_DIRATTR_ARCHIVE	= 0x20,		// 0x20 - ATTR_ARCHIVE
	FATFS_DIRATTR_LONG_NAME = 			// ATTR_LONG_NAME is defined as follows:
			(FATFS_DIRATTR_READ_ONLY |	// (ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID)
			 FATFS_DIRATTR_HIDDEN 	|
			 FATFS_DIRATTR_SYSTEM	|
			 FATFS_DIRATTR_VOLUME_ID)
} FATFS_DIRATTR;

// FAT32 File System Info
typedef struct _FATFS_FSInfo {
	uint32_t	LeadSig;			// Value = 0x41615252. This lead signature is used to validate
									// the beginning of the FSInfo structure in the sector.
	uint8_t		Reserved1[480];		// Reserved. Must be set to 0.
	uint32_t	StrucSig;			// Value = 0x61417272. An additional signature validating the
									// integrity of the FSInfo structure.
	uint32_t	Free_Count;			// Contains the last known free cluster count on the volume.
									// The value 0xFFFFFFFF indicates the free count is not known.
	uint32_t	Nxt_Free;			// Contains the cluster number of the first available (free)
									// cluster on the volume.
	uint8_t		Reserved2[12];		// Reserved. Must be set to 0.
	uint32_t	TrailSig;			// Value = 0xAA550000. . This trail signature is used to validate
									// the integrity of the data in the sector containing the FSInfo structure.
} FATFS_FSINFO;


#define FATFS_DIRENT_LASTLONG		0x40
#define FATFS_DIRENT_DELETED		0xE5

typedef struct _FATFS_ShortName {
	uint8_t		Name[11];		// Short name.
	uint8_t		Attr;			// File attributes:
								// 0x01 - ATTR_READ_ONLY
								// 0x02 - ATTR_HIDDEN
								// 0x04	- ATTR_SYSTEM
								// 0x08 - ATTR_VOLUME_ID
								// 0x10 - ATTR_DIRECTORY
								// 0x20 - ATTR_ARCHIVE
								// ATTR_LONG_NAME is defined as follows:
								// (ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID)
	uint8_t		NTRes;			// Reserved for use by Windows NT. Set value to 0
	uint8_t		CrtTimeTenth;	// Millisecond stamp at file creation time. This field actually
								// contains a count of tenths of a second.
	uint16_t	CrtTime;		// Time file was created.
	uint16_t	CrtDate;		// Date file was created.
	uint16_t	lstAccDate;		// Last access date. Note that there is no last access time, only a date
	uint16_t	FstClusHI;		// High word of this entry’s first cluster number (always 0 for a FAT12 or FAT16 volume).
	uint16_t	WrtTime;		// Time of last write. Note that file creation is considered a write.
	uint16_t	WrtDate;		// Date of last write. Note that file creation is considered a write.
	uint16_t	FstClusLO;		// Low word of this entry’s first cluster number.
	uint32_t	FileSize;		// 32-bit DWORD holding this file’s size in bytes.
} FATFS_SHORTNAME;

typedef struct _FATFS_LongName {
	uint8_t		Ord;			// The order of this entry in the sequence of long dir entries
								// associated with the short dir entry at the end of the long dir set.
								// If masked with 0x40 (LAST_LONG_ENTRY), this indicates the entry
								// is the last long dir entry in a set of long dir entries.
	uint8_t 	Name1[10];		// Characters 1-5 of the long-name sub-component in this dir entry.
	uint8_t		Attr;			// Attributes - must be ATTR_LONG_NAME
	uint8_t		Type;			// If zero, indicates a directory entry that is a sub-component
								// of a long name. NOTE: Other values reserved for future extensions.
								// Non-zero implies other dirent types.
	uint8_t		Chksum;			// Checksum of name in the short dir entry at the end of the long dir set.
	uint8_t		Name2[12];		// Characters 6-11 of the long-name sub-component in this dir entry.
	uint16_t	FstClusLO;		// Must be ZERO. This is an artifact of the FAT "first cluster"
	uint8_t		Name3[4];		// Characters 12-13 of the long-name sub-component in this dir entry.		} LongName;

} FATFS_LONGNAME;

typedef union _FATFS_DirEntry {
	FATFS_SHORTNAME ShortName;
	FATFS_LONGNAME LongName;
} FATFS_DIR;

#pragma pack(pop)

#pragma pack(push, 4)

typedef struct {
	char 		VolName[12];	// Volume name
//	FATFS_TYPE	FatType;
	int 		SectSize;
	uint32_t 	DataSize;
	const FATFS_DIR *pRootDir;
	const uint16_t *pFat1;
} FATFS_VDISKCFG;

typedef struct {
	FATFS_TYPE	FatType;
	uint32_t	TotalSectors;
	uint32_t	RootDirSect;
	uint32_t	DataStartSector;
	uint32_t	Fat1Sector;
	FATFS_BSBPB BootSect;
	FATFS_FSINFO Fat32Info;
	const FATFS_DIR *pRootDir;
	const uint16_t *pFat1;
} FATFS_VDISK;

// File descriptor
typedef struct {
	void 		*pFs;			// Pointer to file system object
	DIR 		DirEntry;		// File directory entry
	uint32_t	CurClus;		// Current data cluster
	uint32_t 	SectIdx;		// Sector index in CurClus
	uint32_t	SectOff;		// Current file pos : offset in sector
//	uint8_t 	SectData[512];	// Current sector data
} FATFS_FD;

#pragma pack(pop)

#ifdef __cplusplus
#include <memory>
#include "diskio.h"

class FatFS {
public:
	FatFS() { }//memset(vOpenFiles, 0, sizeof(vOpenFiles)); }
	virtual ~FatFS() {}
	bool Init(DiskIO *pDiskIO);
	bool Find(char *pPathName, DIR *pDir);
	int Open(char *pPathName, int Flags, int Mode);
	int Close(int fd);
	int Read(int Fd, uint8_t *pBuff, size_t Len);
	int Write(int Fd, uint8_t *pBuf, size_t Len);

protected:
	// Calc sector number for cluster
	uint32_t ClusToSect(uint32_t ClusNo);

private:
	FATFS_TYPE 	vType;				// FAT type
	uint32_t 	vClusterSize;		// Cluster size inm nb of sector
	uint32_t 	vPartStartSect;		// Partition start sector
	uint32_t 	vFatSize;			// FAT table size
	uint32_t 	vTotalSect;			// Total partition size
	uint32_t 	vFATStartSect;		// FAT Table start sector
	uint32_t 	vDataStartSect;		// Data start sector
	uint32_t 	vRootDirSect;		// Root dir start sector
	DIR			vCurDir;			// Current directory
	//std::shared_ptr<DiskIO> vDiskIO;	// Disk object
	DiskIO		*vDiskIO;
	DISKPART 	vPartData;			// Partition data
	FATFS_FD 	vOpenFiles[MAX_FILE];	// Keep list of open files
	//uint8_t 	vSectData[512];		// Temp sector data
	//uint32_t 	vSectNo;			// Absolute sector # of the current temp sector data
};

extern "C" {
#endif
// FAT Filesystem, stdio overwrite
bool FATFSInit(void *pDiskIO);
int FATFSOpen(void *pDevObj, const char *pPathName, int Flags, int Mode);
int FATFSClose(void *pDevObj, int Fd);
int FATFSSeek(void *pDevObj, int Fd, int Offset);
int FATFSRead(void *pDevObj, int Fd, uint8_t *pBuff, size_t Len);
int FATFSWrite(void *pDevObj, int Fd, uint8_t *pBuff, size_t Len);

// Virtual Disk functions
uint32_t FATFSVDiskGetClusterSector(FATFS_VDISK *pVDisk, uint32_t ClusterNo);
uint8_t *FATFSVDiskGetSectorData(FATFS_VDISK *pVDisk, uint32_t SectNo);
bool FATFSVDiskInit(FATFS_VDISK *pVDisk, const FATFS_VDISKCFG *pCfg);

#ifdef __cplusplus
}
#endif

#endif // __FATFS_H__
