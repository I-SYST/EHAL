/*---------------------------------------------------------------------------
File : usb_hidhost_implt.cpp

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

#include <windows.h>
#include <algorithm>
#include <stdbool.h>

#include <setupapi.h>
#include <string>
//#include <initguid.h>
#include <dbt.h>
#pragma hdrstop
#pragma comment (lib, "Setupapi.lib")
#pragma comment (lib, "hid.lib")

#include "usb_hidhost_impl.h"

using namespace std;
//---------------------------------------------------------------------------

//#pragma package(smart_init)

UsbHidDevice_Impl::UsbHidDevice_Impl() :
	vhDevice(INVALID_HANDLE_VALUE),
	vhDevNotification(INVALID_HANDLE_VALUE),
	vhEvent(INVALID_HANDLE_VALUE),
	vpTempBuf(NULL),
	vTempBufSize(0)
{
	memset(&vRdOverlapped, 0, sizeof(OVERLAPPED));
	memset(&vWrOverlapped, 0, sizeof(OVERLAPPED));
}

UsbHidDevice_Impl::~UsbHidDevice_Impl()
{
	if (vhDevNotification != INVALID_HANDLE_VALUE)
		UnregisterDeviceNotification(vhDevNotification);
	if (vhDevice != INVALID_HANDLE_VALUE)
		CloseHandle(vhDevice);
	if (vhEvent != INVALID_HANDLE_VALUE)
		CloseHandle(vhEvent);
	if (vpTempBuf)
		delete[] vpTempBuf;
}

bool UsbHidDevice_Impl::Init(HANDLE hDev)
{
	if (hDev == NULL || hDev == INVALID_HANDLE_VALUE)
		return false;

	vhDevice = hDev;

	PHIDP_PREPARSED_DATA	PreparsedData;

	HidD_GetPreparsedData(vhDevice, &PreparsedData);

	HidP_GetCaps(PreparsedData, &vCaps);

	if (vpTempBuf)
		delete[] vpTempBuf;
	vTempBufSize = std::max<int>(vCaps.InputReportByteLength, vCaps.OutputReportByteLength);
	vTempBufSize = std::max<int>(vTempBufSize, vCaps.FeatureReportByteLength);
	vpTempBuf = new char[vTempBufSize];

	HidD_FreePreparsedData(PreparsedData);

	if (vRdOverlapped.hEvent == 0)
	{
		vRdOverlapped.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	}

	if (vWrOverlapped.hEvent == 0)
	{
		vWrOverlapped.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
	}

	ReadDeviceInfo();

	return true;
}

bool UsbHidDevice_Impl::RegisterWindowNotifications(HWND hWnd)
{
   DEV_BROADCAST_DEVICEINTERFACE DevBroadcastDeviceInterface;
//   HDEVNOTIFY DeviceNotificationHandle;

   DevBroadcastDeviceInterface.dbcc_size = sizeof(DevBroadcastDeviceInterface);
   DevBroadcastDeviceInterface.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
//   DevBroadcastDeviceInterface.dbcc_classguid = vHidGuid;

   vhDevNotification = RegisterDeviceNotification(hWnd,
                                                  &DevBroadcastDeviceInterface,
                                                  DEVICE_NOTIFY_WINDOW_HANDLE);

   return vhDevNotification != NULL;
}

bool UsbHidDevice_Impl::RegisterServiceNotifications(SERVICE_STATUS_HANDLE hService)
{
   DEV_BROADCAST_DEVICEINTERFACE DevBroadcastDeviceInterface;
   HDEVNOTIFY DeviceNotificationHandle;

   DevBroadcastDeviceInterface.dbcc_size = sizeof(DevBroadcastDeviceInterface);
   DevBroadcastDeviceInterface.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
//   DevBroadcastDeviceInterface.dbcc_classguid = vHidGuid;

   vhDevNotification = RegisterDeviceNotification(hService,
                                                  &DevBroadcastDeviceInterface,
                                                  DEVICE_NOTIFY_SERVICE_HANDLE);

   return vhDevNotification != NULL;
}

void UsbHidDevice_Impl::GetDeviceCapabilities()
{
	//Get the Capabilities structure for the device.
	if (vhDevice == NULL)
		return;

	PHIDP_PREPARSED_DATA	PreparsedData;

	HidD_GetPreparsedData(vhDevice, &PreparsedData);

	HidP_GetCaps(PreparsedData, &vCaps);

	if (vpTempBuf)
		delete[] vpTempBuf;
	vTempBufSize = std::max<int>(vCaps.InputReportByteLength, vCaps.OutputReportByteLength);
	vTempBufSize = std::max<int>(vTempBufSize, vCaps.FeatureReportByteLength);
	vpTempBuf = new char[vTempBufSize];

	HidD_FreePreparsedData(PreparsedData);
}

int UsbHidDevice_Impl::ReadInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans)
{
	unsigned long retval = 0;
	DWORD res = FALSE;

	if (vhDevice == NULL || vhDevice == INVALID_HANDLE_VALUE)
		return 0;

	//The first byte is the report number.
	//memset(vpTempBuf, 0, vTempBufSize);
	vpTempBuf[0] = RepNo;

	if (CtrlTrans)
	{
		//Read a report from the device using a control transfer.
		res = HidD_GetInputReport(vhDevice, vpTempBuf, vCaps.InputReportByteLength);
		if (res)
		{
			memcpy(pBuf, vpTempBuf + 1, std::min<int>(BufSize, vTempBufSize));
			retval = vCaps.InputReportByteLength - 1;
		}
	}
	else
	{
		res = ReadFile(vhDevice, vpTempBuf, vCaps.InputReportByteLength,
					   &retval, (LPOVERLAPPED)&vRdOverlapped);

		res = WaitForSingleObject(vRdOverlapped.hEvent, 6000);

		switch (res)
		{
			case WAIT_OBJECT_0:
				res = GetOverlappedResult(vhDevice, &vRdOverlapped, &retval, true);

				//memcpy(pBuf, vpTempBuf + 1, std::min<int>(BufSize, vTempBufSize));
				ProcessInputReport(RepNo, (uint8_t*)vpTempBuf + 1, std::min<int>(BufSize, vTempBufSize));
				break;
			case WAIT_TIMEOUT:

				res = CancelIo(vhDevice);

				break;
			default:
			;
		}

		//ResetEvent(vhEvent);

	}

	return retval;
}

bool UsbHidDevice_Impl::WriteOutputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans)
{
	//Send a report to the device.

	DWORD	BytesWritten = 0;
	INT		Index =0;
	ULONG	res;
	bool retval = false;

	if (vhDevice == NULL || vhDevice == INVALID_HANDLE_VALUE)
		return false;

	//The first byte is the report number.
	vpTempBuf[0] = RepNo;
	int len = BufSize;//std::min<int>(BufSize, vCaps.OutputReportByteLength);
	memcpy(vpTempBuf + 1, pBuf, len);

	if (CtrlTrans)
	{
		// Send a control report to the device.
		res = HidD_SetOutputReport(vhDevice, vpTempBuf, vCaps.OutputReportByteLength);
		if (res)
		{
			retval = true;
		}
	}
	else
	{
		res = WriteFile(vhDevice, vpTempBuf, vCaps.OutputReportByteLength,
			&BytesWritten, (LPOVERLAPPED)&vWrOverlapped);
		res = WaitForSingleObject(vWrOverlapped.hEvent, 6000);

		switch (res)
		{
			case WAIT_OBJECT_0:
				retval = true;
				break;
			case WAIT_TIMEOUT:
				res = CancelIo(vhDevice);

				break;
			default:
				;
		}
		//ResetEvent(vhEvent);

	}

	return retval;
}

int UsbHidDevice_Impl::ReadFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize)
{
	// Read a Feature report from the device.
	bool res = false;
   int retval = 0;

	//The first byte is the report number.
   memset(vpTempBuf, 0, vTempBufSize);
	vpTempBuf[0] = RepNo;

	//Read a report from the device.

   /*
   HidD_GetFeature
   Returns:
   True on success
   Requires:
   A device handle returned by CreateFile.
   A buffer to hold the report.
   The report length returned by HidP_GetCaps in Capabilities.InputReportByteLength.
   */

   if (vhDevice != INVALID_HANDLE_VALUE)
      res = HidD_GetFeature(vhDevice, vpTempBuf, vCaps.FeatureReportByteLength);
   if (res)
   {
		memcpy(pBuf, vpTempBuf + 1, min(vCaps.FeatureReportByteLength, (unsigned short)BufSize));
      retval = vCaps.FeatureReportByteLength;
   }
   else
   {
      //The read attempt failed, so close the handles, display a message,
      //and set MyDeviceDetected to FALSE so the next attempt will look for the device.

   }

   return retval;
}

bool UsbHidDevice_Impl::WriteFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize)
{
	//Send a report to the device.
	DWORD	BytesWritten = 0;
	INT		Index =0;
	ULONG	res = 0;

	//The first byte is the report number.
   memset(vpTempBuf, 0, vTempBufSize);
	vpTempBuf[0] = RepNo;

   memcpy(&vpTempBuf[1], pBuf, min(BufSize, (uint32_t)vCaps.FeatureReportByteLength));

	//Send a report to the device.

	/*
	HidD_SetFeature
	Sends a report to the device.
	Returns: success or failure.
	Requires:
	A device handle returned by CreateFile.
	A buffer that holds the report.
	The Output Report length returned by HidP_GetCaps,
	*/

	if (vhDevice != INVALID_HANDLE_VALUE)
		res = HidD_SetFeature(vhDevice, vpTempBuf, vCaps.FeatureReportByteLength);

	if (!res)
	{
		//The write attempt failed, so close the handles, display a message,
		//and set MyDeviceDetected to FALSE so the next attempt will look for the device.

	}
	else
	{
	}

   return res;
}

bool UsbHidDevice_Impl::WaitResponse(int TimeoutSec)
{
	return true;
}

bool UsbHidDevice_Impl::GetSerialNumber(std::wstring &sn)
{
   wchar_t buf[256];
   bool retval = HidD_GetSerialNumberString(vhDevice, buf, 255*2);
   if (retval)
   {
      sn = buf;
   }

   return retval;
}

bool UsbHidDevice_Impl::GetSerialNumber(std::string &sn)
{
   wchar_t buf[256];
   bool retval = HidD_GetSerialNumberString(vhDevice, buf, 255*2);
   if (retval)
   {
      sn.assign(&buf[0], &buf[wcslen(buf)]);
   }

   return retval;
}

bool UsbHidDevice_Impl::GetProductString(std::wstring &pd)
{
   wchar_t buf[256];
   bool retval = HidD_GetProductString(vhDevice, buf, 255*2);
   if (retval)
   {
      pd = buf;
   }

   return retval;
}



