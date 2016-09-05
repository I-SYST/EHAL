/*--------------------------------------------------------------------------
 File   : usbhiddev_impl.h
 
 Author : Hoang Nguyen Hoan          July 9, 2009
 
 Desc   : Implementation class for USB device on MAC OS X
 
 Copyright (c) 2009, Ekomini Inc, all rights reserved
 ----------------------------------------------------------------------------
 Modified by          Date              Description
 ----------------------------------------------------------------------------*/
#include "usbhiddev_impl.h"

void UsbHidDevice_Impl::IOHIDDeviceReportCallback(void *inContext, // context from IOHIDDeviceRegisterInputReportCallback
												  IOReturn inResult, // completion result for the input report operation
												  void *inSender,    // IOHIDDeviceRef of the device this report is from
												  IOHIDReportType inType, // the report type
												  uint32_t        inReportID, // the report ID
												  uint8_t *       inReport,   // pointer to the report data
												  CFIndex         InReportLength) // the actual size of the input report
{
	if (inContext)
	{
		UsbHidDevice_Impl *hiddev = (UsbHidDevice_Impl *)inContext;
		hiddev->ProcessInputReport(inReportID, inReport, (uint32_t)InReportLength);
		CFRunLoopStop(CFRunLoopGetCurrent());
	}
	
}

UsbHidDevice_Impl::UsbHidDevice_Impl(IOHIDDeviceRef HidDev):
	vpInRepData(NULL)
{
	if (HidDev)
	{
		Init(HidDev);
	}
}

UsbHidDevice_Impl::~UsbHidDevice_Impl()
{
	if (vHidDev)
		CFRelease(vHidDev);
	
	if (vpInRepData)
		delete[] vpInRepData;
}

bool UsbHidDevice_Impl::Init(IOHIDDeviceRef HidDev)
{
	if (HidDev)
	{
		vHidDev = HidDev;
		
		CFNumberRef numb = (CFNumberRef)IOHIDDeviceGetProperty(vHidDev, CFSTR(kIOHIDMaxInputReportSizeKey));
		CFNumberGetValue(numb, kCFNumberSInt32Type, &vMaxInRepSize);
		numb = (CFNumberRef)IOHIDDeviceGetProperty(vHidDev, CFSTR(kIOHIDMaxOutputReportSizeKey));
		CFNumberGetValue(numb, kCFNumberSInt32Type, &vMaxOutRepSize);
		numb = (CFNumberRef)IOHIDDeviceGetProperty(vHidDev, CFSTR(kIOHIDMaxFeatureReportSizeKey));
		CFNumberGetValue(numb, kCFNumberSInt32Type, &vMaxFeatRepSize);
		
		//std::string sn = CFStringGetCStringPtr(tref, kCFStringEncodingUTF8);
		
		if (vpInRepData)
			delete vpInRepData;
		vpInRepData = new uint8_t[vMaxInRepSize];
		IOHIDDeviceRegisterInputReportCallback(vHidDev,           // IOHIDDeviceRef for the HID device
											   (uint8_t *)vpInRepData,             // pointer to the report data (uint8_t's)
											   vMaxInRepSize,         // number of bytes in the report (CFIndex)
											   IOHIDDeviceReportCallback,   // the callback routine
											   this);           // context passed to callback
		IOHIDDeviceScheduleWithRunLoop(HidDev, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
		ReadDeviceInfo();
	}
	
	return true;
}

bool UsbHidDevice_Impl::WaitResponse(int TimeoutSec)
{
    SInt32 res;
    
    do {
        res = CFRunLoopRunInMode(kCFRunLoopDefaultMode, TimeoutSec, false);
    } while(res == kCFRunLoopRunHandledSource);
    
    return (res != kCFRunLoopRunTimedOut);
}

bool UsbHidDevice_Impl::ProcessInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize)
{
	return true;
}

void UsbHidDevice_Impl::GetDeviceCapabilities()
{
}

int UsbHidDevice_Impl::ReadInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans)
{
	SInt32 res;

	do {
		res = CFRunLoopRunInMode(kCFRunLoopDefaultMode, 10, false);
	} while(res == kCFRunLoopRunHandledSource);
	
	return (res != kCFRunLoopRunTimedOut);
/*
	if (vHidDev == NULL)
		return 0;
	
	//uint8_t b[100];
	CFIndex c = 100;
	IOReturn ress = IOHIDDeviceGetReport(vHidDev, kIOHIDReportTypeInput, RepNo, pBuf, (CFIndex*)&c);
	if (ress != kIOReturnSuccess)
		return 0;
	ProcessInputReport(RepNo, pBuf, (uint32_t)c);
	return c;*/
}

bool UsbHidDevice_Impl::WriteOutputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans)
{
	if (vHidDev == NULL)
		return false;

	IOReturn  res = IOHIDDeviceSetReport(vHidDev, kIOHIDReportTypeOutput, RepNo,
											(const uint8_t *)pBuf, BufSize);

    if (res != kIOReturnSuccess)
        printf("WriteReport Error %x\n", res);
	return res == kIOReturnSuccess;

}

int UsbHidDevice_Impl::ReadFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize)
{
	if (vHidDev == NULL)
		return false;

	IOHIDDeviceGetReport(vHidDev, kIOHIDReportTypeFeature, RepNo, pBuf, (CFIndex*)&BufSize);
	
	return BufSize;
}

bool UsbHidDevice_Impl::WriteFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize)
{
	if (vHidDev == NULL)
		return false;
	
	IOReturn res = IOHIDDeviceSetReport(vHidDev, kIOHIDReportTypeFeature, RepNo, pBuf, BufSize);

	return res == kIOReturnSuccess;
}

bool UsbHidDevice_Impl::GetSerialNumber(std::string &sn)
{
	CFStringRef tref = (CFStringRef)IOHIDDeviceGetProperty(vHidDev, CFSTR(kIOHIDSerialNumberKey));
	sn = CFStringGetCStringPtr(tref, kCFStringEncodingUTF8);
	
	return false;
}

