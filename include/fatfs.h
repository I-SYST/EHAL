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

#pragma pack(push, 1)

typedef enum _FATFS_FAT_Entry_Value {
	FATFS_FATENTRY_FREE	= 0,
	FATFS_FATENTRY_BAD = 0xFFFFFF7,
	FATFS_FATENTRY_ALLOCATED = 0xFFFFFFFF
} FATFS_FATENTRY;

typedef struct _FATFS_BootSector_BPB {
	uint8_t 	BS_jmpBoot[3]; 	// Jump instruction to boot code. This field has two allowed forms:
								// jmpBoot[0] = 0xEB, jmpBoot[1] = 0x??, jmpBoot[2] = 0x90
								// and jmpBoot[0] = 0xE9, jmpBoot[1] = 0x??, jmpBoot[2] = 0x??
								// 0x?? indicates that any 8-bit value is allowed in that byte.
								// What this forms is a three-byte Intel x86 unconditional
								// branch (jump) instruction that jumps to the start of the operating
								// system bootstrap code.
	uint8_t 	BS_OEMName[8];	// "MSWIN4.1" is the recommended setting, because it is the setting
								// least likely to cause compatibility problems.
	uint16_t 	BPB_BytsPerSec;	// Count of bytes per sector. 512, 1024, 2048 or 4096. If maximum
								// compatibility 512 is recommended
	uint8_t		BPB_SecPerClus;	// Number of sectors per allocation unit. Must be a power
								// of 2 that is greater than 0 where SecPerClus * BytesPerSec <= 32KB
	uint16_t	BPB_RsvdSecCnt; // Number of reserved sectors in the Reserved.
								// 		1 - FAT12/FAT16,
								// 		32 - FAT32
	uint8_t		BPB_NumFATs;	// The count of FAT data structures on the volume.
								// 		This field should always contain the value 2
	uint16_t	BPB_RootEntCnt; // Number of Root Directory Entries in multiple of 32 bytes.
								// 		512 - for FAT16, 0 - for FAT32
	uint16_t	BPB_TotSec16;	// This field is the old 16-bit total count of sectors on the volume.
								// 		0 for FAT32
	uint8_t		BPB_Media;		// 0xF8 - “fixed” (non-removable) media.
								// 0xF0 - For removable media
	uint16_t	BPB_FATSz16;	// FAT12/FAT16 : 16-bit count of sectors occupied by ONE FAT
								// FAT32 : must be 0
	uint16_t	BPB_SecPerTrk;	// Sectors per track for interrupt 0x13
	uint16_t	BPB_NumHeads;	// Number of heads for interrupt 0x13.
	uint32_t	BPB_HiddSec;	// Count of hidden sectors preceding the partition that contains
								// this FAT volume. This field is generally only relevant for media
								// visible on interrupt 0x13.
	uint32_t	BPB_TotSec32;	// FAT32 total count of sectors on the volume.
	union {
		struct _FAT16_BPB {
			uint8_t BS_DrvNum;	// Interrupt 0x13 drive number. Set value to 0x80 or 0x00.
			uint8_t BS_Reserved1;	// Reserved. Set value to 0x0.
			uint8_t BS_BootSig;	// Extended boot signature. Set value to 0x29 if either of
								// the following two fields are non-zero.
			uint32_t BS_VolID;	// Volume serial number. This ID should be generated by simply
								// combining the current date and time into a 32-bit value.
			uint8_t	BS_VolLab[11];	// Volume label. This field matches the 11-byte volume
									// label recorded in the root directory.
			uint8_t BS_FilSysType[8];	// One of the strings “FAT12 ”, “FAT16 ”, or “FAT ”.
										// NOTE: This string is informational only and does not
										// determine the FAT type.
		} Bpb16;
		struct _FAT32_BPB {
			uint32_t	BPB_FATSz32;	// This field is the FAT32 32-bit count of sectors occupied
										// by one FAT. Note that BPB_FATSz16 must be 0 for media
										// formatted FAT32.
			uint16_t	BPB_ExtFlags;	// Bits0-3 -- Zero-based number of active FAT.
										//			  Only valid if mirroring is disabled.
										// Bits 4-6 -- Reserved.
										// Bit 7 -- 0 means the FAT is mirrored at runtime into all FATs.
										// 			1 means only one FAT is active; it is the one
										//			  referenced in bits 0-3.
										// Bits 8-15 -- Reserved.
			uint16_t	BPB_FSVer;		// High byte is major revision number.
										// Low byte is minor revision number.
										// Must be set to 0x0.
			uint32_t	BPB_RootClus;	// This is set to the cluster number of the first cluster
										// of the root directory. This value should be 2 or the first
										// usable (not bad) cluster available thereafter.
			uint16_t	BPB_FSInfo;		// Sector number of FSINFO structure in the reserved area of
										// the FAT32 volume. Usually 1.
			uint16_t	BPB_BkBootSec;	// Set to 0 or 6. If non-zero, indicates the sector number
										// in the reserved area of the volume of a copy of the boot record.
			uint8_t		BPB_Reserved[12];	// Reserved. Must be set to 0x0.
			uint8_t		BS_DrvNum;		// Interrupt 0x13 drive number. Set value to 0x80 or 0x00.
			uint8_t		BS_Reserved1;	// Reserved. Set value to 0x0.
			uint8_t		BS_BootSig;		// Extended boot signature. Set value to 0x29 if either of the
										// following two fields are non-zero.
			uint32_t	BS_VolID;		// Volume serial number. This ID should be generated by simply
										// combining the current date and time into a 32-bit value.
			uint8_t		BS_VolLab[11];	// Volume label. This field matches the 11-byte volume label
										// recorded in the root directory.
			uint8_t		BS_FilSysType[8];	// Set to the string:”FAT32 ”.
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
	uint32_t	FSI_LeadSig;			// Value = 0x41615252. This lead signature is used to validate
										// the beginning of the FSInfo structure in the sector.
	uint8_t		FSI_Reserved1[480];		// Reserved. Must be set to 0.
	uint32_t	FSI_StrucSig;			// Value = 0x61417272. An additional signature validating the
										// integrity of the FSInfo structure.
	uint32_t	FSI_Free_Count;			// Contains the last known free cluster count on the volume.
										// The value 0xFFFFFFFF indicates the free count is not known.
	uint32_t	FSI_Nxt_Free;			// Contains the cluster number of the first available (free)
										// cluster on the volume.
	uint8_t		FSI_Reserved2[12];		// Reserved. Must be set to 0.
	uint32_t	FSI_TrailSig;			// Value = 0xAA550000. . This trail signature is used to validate
										// the integrity of the data in the sector containing the FSInfo structure.
} FATFS_FSINFO;

typedef struct _FATFS_DirEntry {
	union {
		struct _ShortName {
			uint8_t		DIR_Name[11];		// Short name.
			uint8_t		DIR_Attr;			// File attributes:
											// 0x01 - ATTR_READ_ONLY
											// 0x02 - ATTR_HIDDEN
											// 0x04	- ATTR_SYSTEM
											// 0x08 - ATTR_VOLUME_ID
											// 0x10 - ATTR_DIRECTORY
											// 0x20 - ATTR_ARCHIVE
											// ATTR_LONG_NAME is defined as follows:
											// (ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID)
			uint8_t		DIR_NTRes;			// Reserved for use by Windows NT. Set value to 0
			uint8_t		DIR_CrtTimeTenth;	// Millisecond stamp at file creation time. This field actually
											// contains a count of tenths of a second.
			uint16_t	DIR_CrtTime;		// Time file was created.
			uint16_t	DIR_CrtDate;		// Date file was created.
			uint16_t	DIR_lstAccDate;		// Last access date. Note that there is no last access time, only a date
			uint16_t	DIR_FstClusHI;		// High word of this entry’s first cluster number (always 0 for a FAT12 or FAT16 volume).
			uint16_t	DIR_WrtTime;		// Time of last write. Note that file creation is considered a write.
			uint16_t	DIR_WrtDate;		// Date of last write. Note that file creation is considered a write.
			uint16_t	DIR_FstClusLO;		// Low word of this entry’s first cluster number.
			uint32_t	DIR_FileSize;		// 32-bit DWORD holding this file’s size in bytes.
		} ShortName;
		struct _LongName {
			uint8_t		LDIR_Ord;			// The order of this entry in the sequence of long dir entries
											// associated with the short dir entry at the end of the long dir set.
											// If masked with 0x40 (LAST_LONG_ENTRY), this indicates the entry
											// is the last long dir entry in a set of long dir entries.
			uint8_t 	LDIR_Name1[10];		// Characters 1-5 of the long-name sub-component in this dir entry.
			uint8_t		LDIR_Attr;			// Attributes - must be ATTR_LONG_NAME
			uint8_t		LDIR_Type;			// If zero, indicates a directory entry that is a sub-component
											// of a long name. NOTE: Other values reserved for future extensions.
											// Non-zero implies other dirent types.
			uint8_t		LDIR_Chksum;		// Checksum of name in the short dir entry at the end of the long dir set.
			uint8_t		LDIR_Name2[12];		// Characters 6-11 of the long-name sub-component in this dir entry.
			uint16_t	LDIR_FstClusLO;		// Must be ZERO. This is an artifact of the FAT "first cluster"
			uint8_t		LDIR_Name3[4];		// Characters 12-13 of the long-name sub-component in this dir entry.		} LongName;

		} LongName;
	};
} FATFS_DIR;

#pragma pack(pop)

#pragma pack(push, 4)

typedef struct {
	char 		VolName[12];	// Volume name
	FATFS_TYPE	FatType;
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

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

// Virtual Disk functions
uint32_t FATFSVDiskGetClusterSector(FATFS_VDISK *pVDisk, uint32_t ClusterNo);
uint8_t *FATFSVDiskGetSectorData(FATFS_VDISK *pVDisk, uint32_t SectNo);
bool FATFSVDiskInit(FATFS_VDISK *pVDisk, const FATFS_VDISKCFG *pCfg);

#ifdef __cplusplus
}
#endif

#endif // __FATFS_H__
