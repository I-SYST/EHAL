// Copyright (c) 2008, I-SYST, All rights reserved
//---------------------------------------------------------------------------

#ifndef __USBHIDDEV_IMPL_H__
#define __USBHIDDEV_IMPL_H__

#include <string>
#include "Windows.h"
#include <wchar.h>

extern "C" {
#include "hidsdi.h"
#include "hidpi.h"
}

#include "UsbHidDev.h"

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
#endif
