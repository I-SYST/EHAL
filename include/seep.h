/*--------------------------------------------------------------------------
File   : seep.h

Author : Hoang Nguyen Hoan          Sept. 15, 2011

Desc   : Serial EEPROM device implementation

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
#ifndef __SEEP_H__
#define __SEEP_H__

#include <stdbool.h>

#include "serialintrf.h"

#define SEEP_NB_IOPIN	2

#pragma pack(push,4)

typedef struct {
	int DevAddr;
	int AddrLen;
	int PageSize;
	SERINTRFDEV	*pSerIntrf;
} SEEPDEV;

#pragma pack(pop)

#ifdef __cplusplus

#include <memory>

class Seep {
public:
	Seep();
	virtual ~Seep();
	Seep(Seep&); 	// copy ctor not allowed

	virtual bool Init(int DevAddr, int PageSize, int AddrLen, SerialIntrf *pInterf);
	virtual void Set(int DevAddr, int PageSize, int AddrLen) {
		vDevAddr = DevAddr;
		vPageSize = PageSize;
		vAddrLen = AddrLen;
	}
	virtual int Read(int Addr, uint8_t *pData, int Len);
	virtual int Write(int Addr, uint8_t *pData, int Len);

private:
	std::shared_ptr<SerialIntrf> vpInterf;
	int vDevAddr;		// SEEP I2C address
	int vPageSize;
	int vAddrLen;
};

extern "C" {
#endif
// C prototypes
bool SeepInit(SEEPDEV *pDev, int DevAddr, int PageSize, int AddrLen, SERINTRFDEV *pIntrf);
int SeepRead(SEEPDEV *pDev, int Addr, uint8_t *pData, int len);
int SeepWrite(SEEPDEV *pDev, int Addr, uint8_t *pData, int len);

#ifdef __cplusplus
}
#endif

#endif	// __SEEP_H__

