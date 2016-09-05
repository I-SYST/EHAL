/*---------------------------------------------------------------------------
File : usbhiddev.h

Author : Hoang Nguyen Hoan          Jan. 16, 2009

Desc :   Generic class of Host side USB HID device access

Copyright (c) 2009, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose 
with or without fee is hereby granted, provided that the above copyright 
notice and this permission notice appear in all copies, and none of the 
names : I-SYST, TidalStream or its contributors may be used to endorse or 
promote products derived from this software without specific prior written 
permission.

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
Modified by          		Date              Description
Hoan						June 2009			Ekomini
----------------------------------------------------------------------------*/

#ifndef __USBHIDDEV_H__
#define __USBHIDDEV_H__

#include <string>
#include <stdint.h>

class UsbHidDevice {

public:
//    UsbHidDevice::UsbHidDevice() {}
	
	// Must be implemented per target implementation
	static UsbHidDevice *Create();

	virtual int ReadInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans = true)=0;
	virtual bool WriteOutputReport(int RepNo, uint8_t *pBuf, uint32_t Bufsize, bool CtrlTrans = true)=0;
	virtual int ReadFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize)=0;
	virtual bool WriteFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize) = 0;
	virtual bool GetSerialNumber(std::string &sn)=0;
	virtual bool ProcessInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize) = 0;
    virtual bool WaitResponse(int TimeOutSec) = 0;
    
protected:
	virtual void ReadDeviceInfo() = 0;
	uint32_t vMaxInRepSize;
	uint32_t vMaxOutRepSize;
	uint32_t vMaxFeatRepSize;
	
private:
};

#endif   // __USBHIDDEV_H__
