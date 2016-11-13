/*---------------------------------------------------------------------------
 File : usb_hidhost_implt.h
 
 Author : Hoang Nguyen Hoan          July 9, 2009
 
 Desc : Generic class of Host side USB HID device access
        Implementation class for USB device on MAC OS X
 
 Copyright (c) 2009-16, I-SYST, all rights reserved
 Copyright (c) 2009, Ekomini Inc, all rights reserved
 
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

#include <IOKit/hid/IOHIDLib.h>
#include <IOKit/hid/IOHIDDevice.h>

#include "usb_hidhost.h"

typedef void (*HIDINTRPTINCB)(UsbHidDevice *pDev, char *pBuf, uint32_t DataLen);

class UsbHidDevice_Impl : public UsbHidDevice {
public:
	UsbHidDevice_Impl(IOHIDDeviceRef HidDev = NULL);
//	UsbHidDevice_Impl(IONotificationPortRef pNotPort, io_iterator_t HidDevIter);
	virtual ~UsbHidDevice_Impl();
	virtual void ReadDeviceInfo() {}
	virtual bool Init(IOHIDDeviceRef HidDev);
	virtual void GetDeviceCapabilities();
	virtual int ReadInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans = true);
	virtual bool WriteOutputReport(int RepNo, uint8_t *pBuf, uint32_t Bufsize, bool CtrlTrans = true);
	virtual int ReadFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
	virtual bool WriteFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
	virtual bool GetSerialNumber(std::string &sn);
	virtual bool ProcessInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
    virtual bool WaitResponse(int TimeOutSec);
	
protected:

	static void IOHIDDeviceReportCallback(void *inContext,     // context from IOHIDDeviceRegisterInputReportCallback
										  IOReturn inResult,   // completion result for the input report operation
										  void *inSender,      // IOHIDDeviceRef of the device this report is from
										  IOHIDReportType inType,             // the report type
										  uint32_t        inReportID,         // the report ID
										  uint8_t *       inReport,           // pointer to the report data
										  CFIndex         InReportLength);     // the actual size of the input report

//	char	*vpBuffer;

private:
	IOHIDDeviceRef      vHidDev;	// USB HID device handle
	//CFMutableDictionaryRef	vpHIDMatchDict;
	//IONotificationPortRef	vpNotifyPort;
	//io_iterator_t				vIOIter;
	//CFRunLoopRef				vpRunLoop;
	//IOHIDDeviceInterface122 **vpDeviceInterface;
	//io_object_t					vNotification;
	//io_object_t					vDevice;
	//mach_port_t					vAsyncPort;
	//CFRunLoopSourceRef		vEventSource;
	uint8_t *vpInRepData;
};

#endif   // __USB_HIDHOST_IMPL_H__
