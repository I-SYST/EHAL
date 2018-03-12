/*---------------------------------------------------------------------------
 File : usb_hidhost_implt.h
 
 Author : Hoang Nguyen Hoan          Mar. 9, 2018
 
 Desc : Generic class of Host side USB HID device access
        Implementation class for USB device on Linux
 
 NOTE:
 Linux require rules set for device permission.
 create a a rule file in /etc/udev/rules.d
 that contains

 KERNEL=="hidraw*", ATTRS{busnum}=="1", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="003f", MODE="0666"

  then reloaded with $ sudo udevadm control --reload-rules

 Copyright (c) 2018, I-SYST, all rights reserved
 
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

#include <libudev.h>

#include "usb/usb_hidhost.h"

typedef struct udev_device	HIDDEV;

class UsbHidDevice_Impl : public UsbHidDevice {
public:
	UsbHidDevice_Impl();
	virtual ~UsbHidDevice_Impl();
	virtual void ReadDeviceInfo() {}
	virtual bool Init(HIDDEV *pDev);
	virtual void GetDeviceCapabilities();
	virtual int ReadInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize, bool CtrlTrans = true);
	virtual bool WriteOutputReport(int RepNo, uint8_t *pBuf, uint32_t Bufsize, bool CtrlTrans = true);
	virtual int ReadFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
	virtual bool WriteFeatureReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
	virtual bool GetSerialNumber(std::string &sn);
	virtual bool ProcessInputReport(int RepNo, uint8_t *pBuf, uint32_t BufSize);
    virtual bool WaitResponse(int TimeOutSec);
	
protected:

private:
	HIDDEV *vpDev;
	int vDevHandle;
	uint8_t *vpInRepData;
};

#endif   // __USB_HIDHOST_IMPL_H__
