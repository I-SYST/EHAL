/*--------------------------------------------------------------------------
File   : device_bridge.h

Author : Hoang Nguyen Hoan          			May 14, 2017

Desc   : Implementation of a bridge device.  This device route data from
		 one device interface to othe device interface

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
#ifndef __DEVICE_BRIDGE_H__
#define __DEVICE_BRIDGE_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "device.h"
#include "iopincfg.h"

#pragma pack(push, 4)

#pragma pack(pop)

#ifdef __cplusplus

class DeviceBridge : public Device {
public:
	virtual bool Enable() {}
	virtual void Disable() {}
	virtual void Reset() {}

	bool Init(DeviceIntrf *pIntf1, DeviceIntrf *pIntrf2);

protected:

	void SetInterface(DeviceIntrf *pIntrf) { vpIntrf = pIntrf; }
	void SetDeviceAddess(uint32_t Addr) { vDevAddr =  Addr; }

	uint32_t vDevAddr;
	DeviceIntrf *vpIntrf;
	uint32_t vDevAddr2;
	DeviceIntrf *vpIntrf2;
};

extern "C" {
#endif	// __cplusplus


#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __DEVICE_BRIDGE_H__
