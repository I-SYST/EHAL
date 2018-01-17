/**-------------------------------------------------------------------------
@file	sdcard.h

@brief	SD card driver


@author	Hoang Nguyen Hoan
@date	June 9, 2011

@license

Copyright (c) 2011, I-SYST, all rights reserved

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

----------------------------------------------------------------------------*/
#ifndef __SDCARD_H__
#define __SDCARD_H__

#include <stdint.h>

#include "device_intrf.h"
#include "diskio.h"

#define SDCARD_CACHE_MAX		2

#pragma pack(push, 4)

// CSD register
typedef struct {
	uint32_t Vers:2;
	uint32_t Rsvd1:6;
	uint32_t Taac:8;
	uint32_t Nsac:8;
} SDCSD;

typedef struct {
	int Rate;
	DEVINTRF *pSerIntrf;
} SDCFG;

typedef struct {
	int SectSize;
	int TotalSect;
	SDCSD Csd;
	DEVINTRF *pSerIntrf;
} SDDEV;

#pragma pack(pop)

// Define SD device handle.  This is a pointer to SDCard class for C
// function wrapper for use with C application code.
// Internal implementation will cast this to proper class pointer
typedef void*		HSDDEV;


#ifdef __cplusplus

#include <memory>

class SDCard : public DiskIO {
public:
	SDCard();
	virtual ~SDCard();

	virtual bool Init(DeviceIntrf *pDevInterf, uint8_t *pCacheMem = NULL, int CacheMemSize = 0);
	virtual bool Init(DeviceIntrf *pDevInterf, DISKIO_CACHE_DESC *pCacheBlk = NULL, int NbCacheBlk = 0);
	int Cmd(uint8_t Cmd, uint32_t param);
	int GetResponse(uint8_t *pBuff, int BuffLen);
	int ReadData(uint8_t *pBuff, int BuffLen);
	int WriteData(uint8_t *pData, int Len);
	int GetSectSize(void);
	uint32_t GetNbSect(void);
	// @return size in KB
	uint64_t GetSize(void);
	int ReadSingleBlock(uint32_t Addr, uint8_t *pData, int Len);
	int WriteSingleBlock(uint32_t Addr, uint8_t *pData, int Len);
	bool SectRead(uint32_t SectNo, uint8_t *pData) {
		return ReadSingleBlock(SectNo, pData, vDev.SectSize) == vDev.SectSize;
	}
	bool SectWrite(uint32_t SectNo, uint8_t *pData) {
		return WriteSingleBlock(SectNo, pData, vDev.SectSize) == vDev.SectSize;
	}
	//operator SDDEV *() { return &vDev; };

protected:
private:
	//std::shared_ptr<SerialIntrf> vpInterf;
	DeviceIntrf *vpInterf;
	SDDEV vDev;
	int NbCacheBlk;
	DISKIO_CACHE_DESC vCacheDesc[SDCARD_CACHE_MAX];
};

extern "C" {
#endif

bool SDInit(SDDEV *pDev, SDCFG *pCfg);
int SDCmd(SDDEV *pDev, uint8_t Cmd, uint32_t param);
int SDGetResponse(SDDEV *pDev, uint8_t *pData, int len);
int SDReadData(SDDEV *pDev, uint8_t *pBuff, int BuffLen);
int SDWriteData(SDDEV *pDev, uint8_t *pData, int DataLen);
int SDReadSingleBlock(SDDEV *pDev, uint32_t Addr, uint8_t *pData, int len);
int SDWriteSingleBlock(SDDEV *pDev, uint32_t Addr, uint8_t *pData, int len);
uint32_t SDGetSize(SDDEV *pDev);

#ifdef __cplusplus
}
#endif

#endif	// __SDCARD_H__

