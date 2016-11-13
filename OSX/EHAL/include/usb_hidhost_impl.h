/*--------------------------------------------------------------------------
 File   : usbhiddev_impl.h
 
 Author : Hoang Nguyen Hoan          July 9, 2009
 
 Desc   : Implementation class for USB device on MAC OS X
 
 Copyright (c) 2009, Ekomini Inc, all rights reserved
 ----------------------------------------------------------------------------
 Modified by          Date              Description
 ----------------------------------------------------------------------------*/
#ifndef __USBHIDDEV_IMPL_H__
#define __USBHIDDEV_IMPL_H__

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

#endif   // __USBHIDDEV_IMPL_H__
