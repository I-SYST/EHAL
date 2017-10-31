/*--------------------------------------------------------------------------
File   : device.h

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : Generic device base class

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#ifndef __DEVICE_H__
#define __DEVICE_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "device_intrf.h"
#include "iopincfg.h"

#pragma pack(push, 4)

#pragma pack(pop)

#ifdef __cplusplus

class Device {
public:
	Device() : vDevAddr(0), vpIntrf(NULL) {}

	virtual bool Enable() = 0;
	virtual void Disable() = 0;
	virtual void Reset() = 0;
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen) {
		if (vpIntrf) {
			return vpIntrf->Read(vDevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
		}
		return 0;
	}

	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen) {
		if (vpIntrf) {
			return vpIntrf->Write(vDevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
		}
		return 0;
	}

	virtual uint8_t ReadReg8(uint8_t *pRegAddr, int RegAddrLen) {
		uint8_t val = 0;
		Read(pRegAddr, RegAddrLen, &val, 1);
		return val;
	}

	virtual uint16_t ReadReg16(uint8_t *pRegAddr, int RegAddrLen) {
		uint16_t val = 0;
		Read(pRegAddr, RegAddrLen,(uint8_t*) &val, 2);
		return val;
	}

	virtual uint32_t ReadReg32(uint8_t *pRegAddr, int RegAddrLen) {
		uint32_t val = 0;
		Read(pRegAddr, RegAddrLen, (uint8_t*)&val, 4);
		return val;
	}

	virtual bool WriteReg(uint8_t *pRegAddr, int RegAddrLen, uint8_t Data) {
		return Write(pRegAddr, RegAddrLen, &Data, 1) > 0;
	}

	virtual bool WriteReg(uint8_t *pRegAddr, int RegAddrLen, uint16_t Data) {
		return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 2) > 1;
	}

	virtual bool WriteReg(uint8_t *pRegAddr, int RegAddrLen, uint32_t Data) {
		return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 1) > 3;
	}

protected:

	void SetInterface(DeviceIntrf *pIntrf) { vpIntrf = pIntrf; }
	DeviceIntrf *GetInterface() { return vpIntrf; }
	void SetDeviceAddess(uint32_t Addr) { vDevAddr =  Addr; }

	uint32_t vDevAddr;
	DeviceIntrf *vpIntrf;
};

extern "C" {
#endif	// __cplusplus


#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __DEVICE_H__
