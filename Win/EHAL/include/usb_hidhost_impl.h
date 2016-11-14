/*---------------------------------------------------------------------------
File : usb_hidhost_implt.h

Author : Hoang Nguyen Hoan          Oct 9, 2008

Desc :	Generic class of Host side USB HID device access
		Implementation class for USB device on Windows

Copyright (c) 2008-16, I-SYST, all rights reserved

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
----------------------------------------------------------------------------*/
#ifndef __USB_HIDHOST_IMPL_H__
#define __USB_HIDHOST_IMPL_H__

#include <string>
#include "Windows.h"
#include <wchar.h>

extern "C" {
#include "hidsdi.h"
#include "hidpi.h"
}

#include "usb_hidhost.h"

typedef struct {

} HIDDEV_DATA;

class UsbHidDevice_Impl : public UsbHidDevice {
public:
	UsbHidDevice_Impl();
	virtual ~UsbHidDevice_Impl();

	bool Init(HANDLE hDev);
	bool RegisterWindowNotifications(HWND hWnd);
	bool RegisterServiceNotifications(SERVICE_STATUS_HANDLE hService);
	void GetDeviceCapabilities();
	int ReadInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans = true);
	bool WriteOutputReport(int RepNo, uint8_t *pBuf, uint32_t Bufsize, bool CtrlTrans = true);
	int ReadFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
	bool WriteFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
	bool WaitResponse(int TimeoutSec);
	bool GetSerialNumber(std::wstring &sn);
	bool GetSerialNumber(std::string &sn);
	bool GetProductString(std::wstring &pd);
	virtual bool ProcessInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize) { return true; }

protected:
	virtual void ReadDeviceInfo() {}

private:
	HANDLE		vhDevice;
	HANDLE		vhDevNotification;
	HIDP_CAPS   vCaps;
	HANDLE		vhEvent;
	OVERLAPPED	vRdOverlapped;
	OVERLAPPED	vWrOverlapped;
	int   vTempBufSize;
	char *vpTempBuf;
};

typedef bool(*USBHIDFOUNDCB)(HANDLE HidDev);

int UsbHidFindDevice(int VendId, int ProdId, USBHIDFOUNDCB Callback);

#endif // __USB_HIDHOST_IMPL_H
