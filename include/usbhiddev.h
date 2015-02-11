/*---------------------------------------------------------------------------
File : usbhiddev.h

Author : Hoang Nguyen Hoan          Jan. 16, 2009

Desc :   Generic class of USB HID device access

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
Modified by          Date              Description

----------------------------------------------------------------------------*/

#ifndef __USBHIDDEV_H__
#define __USBHIDDEV_H__

class UsbHidDevice {
public:
	UsbHidDevice();
	virtual ~UsbHidDevice();

	virtual bool FindDevice(int VendId, int ProdId)=0;
   virtual bool RegisterNotifications(HWND hWnd)=0;
   virtual void GetDeviceCapabilities()=0;
   virtual void PrepareOverlappedTransfer()=0;
   virtual int ReadInputReport(int RepNo, char *pBuf, int BufSize, bool CtrlTrans = true)=0;
   virtual bool WriteOutputReport(int RepNo, char *pBuf, int Bufsize, bool CtrlTrans = true)=0;
   virtual int ReadFeatureReport(int RepNo, char *pBuf, int BufSize)=0;
   virtual bool WriteFeatureReport(int RepNo, char *pBuf, int BufSize)=0;
};

#endif   // __USBHIDDEV_H__
