/*--------------------------------------------------------------------------
File   : sdcard.h

Author : Hoang Nguyen Hoan          June 9, 2011

Desc   : SD card driver

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#ifndef __SDCARD_H__
#define __SDCARD_H__

#include <stdint.h>
#include "serialintrf.h"

#pragma pack(push, 4)

typedef struct {
	int Rate;
	SERINTRFDEV *pSerIntrf;
} SDCFG;

typedef struct {
	SERINTRFDEV *pSerIntrf;
} SDDEV;

#pragma pack(pop)

// Define SD device handle.  This is a pointer to SDCard class for C
// function wrapper for use with C application code.
// Internal implementation will cast this to proper class pointer
typedef void*		HSDDEV;


#ifdef __cplusplus

#include <memory>
//#include <tr1/shared_ptr.h>

class SDCard {
public:
	SDCard();
	virtual ~SDCard();

	virtual bool Init(SerialIntrf *pSerInterf);
	int Cmd(uint8_t Cmd, uint32_t param);
	int GetResponse(uint8_t *pBuff, int BuffLen);
	int ReadData(uint8_t *pBuff, int BuffLen);
	int WriteData(uint8_t *pData, int Len);
	uint32_t GetSize(void);
	int ReadSingleBlock(uint32_t Addr, uint8_t *pData, int Len);
	int WriteSingleBlock(uint32_t Addr, uint8_t *pData, int Len);
	operator SDDEV *() { return &vDev; };

protected:
private:
	std::shared_ptr<SerialIntrf> vpInterf;
	SDDEV vDev;
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

