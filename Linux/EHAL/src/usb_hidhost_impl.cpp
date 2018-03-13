/*---------------------------------------------------------------------------
 File : usb_hidhost_implt.cpp
 
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
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/utsname.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>


#include <linux/hidraw.h>

#include "usb_hidhost_impl.h"

UsbHidDevice_Impl::UsbHidDevice_Impl()
{
	vpDev = NULL;
	vpInRepData = NULL;
	vDevHandle = -1;
}

UsbHidDevice_Impl::~UsbHidDevice_Impl()
{
	if (vDevHandle > 0)
	{
		close(vDevHandle);
	}

	if (vpInRepData)
	{
		delete[] vpInRepData;
	}

	if (vpDev)
	{
		udev_device_unref(vpDev);
	}
}

bool UsbHidDevice_Impl::Init(HIDDEV *pDev)
{
	if (pDev == NULL)
		return false;

	const char *devpath = udev_device_get_devnode(pDev);

	vDevHandle = open(devpath, O_RDWR);

	if (vDevHandle <= 0)
	{
		// failed open
		return false;
	}

	vpDev = udev_device_ref(pDev);

	int res, desc_size = 0;

	struct hidraw_report_descriptor rpt_desc;

	memset(&rpt_desc, 0x0, sizeof(rpt_desc));

	/* Get Report Descriptor Size */
	res = ioctl(vDevHandle, HIDIOCGRDESCSIZE, &desc_size);
	//if (res < 0)
	//	perror("HIDIOCGRDESCSIZE");


	/* Get Report Descriptor */
	rpt_desc.size = desc_size;
	res = ioctl(vDevHandle, HIDIOCGRDESC, &rpt_desc);
	//if (res < 0) {
	//	perror("HIDIOCGRDESC");
	//} else {
		/* Determine if this device uses numbered reports. */
	//	dev->uses_numbered_reports =
	//		uses_numbered_reports(rpt_desc.value,
	//		                      rpt_desc.size);
	//}

	ReadDeviceInfo();

	return true;
}

bool UsbHidDevice_Impl::WaitResponse(int TimeoutSec)
{
	if (vpDev == NULL)
		return false;

	struct pollfd fds;

	fds.fd = vDevHandle;
	fds.events = POLLIN;
	fds.revents = 0;
	int ret = poll(&fds, 1, TimeoutSec * 1000);
	if (ret <= 0 || fds.revents & (POLLERR | POLLHUP | POLLNVAL))
	{
		/* Error or timeout */
		return false;
	}

	return true;
}

bool UsbHidDevice_Impl::ProcessInputReport(int RepNo, uint8_t *pData, uint32_t DataLen)
{
	if (vpDev == NULL)
		return false;

	return true;
}

void UsbHidDevice_Impl::GetDeviceCapabilities()
{
}

int UsbHidDevice_Impl::ReadInputReport(int RepNo, uint8_t *pBuff, uint32_t BuffSize, bool CtrlTrans)
{
	if (vpDev == NULL)
		return 0;

	struct pollfd fds;

	fds.fd = vDevHandle;
	fds.events = POLLIN;
	fds.revents = 0;
	int ret = poll(&fds, 1, 10 * 1000);
	if (ret <= 0 || fds.revents & (POLLERR | POLLHUP | POLLNVAL))
	{
		/* Error or timeout */
		return 0;
	}

	int count = read(vDevHandle, pBuff, BuffSize);

	if (count > 0)
	{
		count = ProcessInputReport(pBuff[0], &pBuff[0], count - 1);
	}
	return count;
}

bool UsbHidDevice_Impl::WriteOutputReport(int RepNo, uint8_t *pData, uint32_t DataLen, bool CtrlTrans)
{
	if (vpDev == NULL)
		return false;

	uint8_t buf[HIDRAW_BUFFER_SIZE + 1];

	buf[0] = RepNo;
	memcpy(&buf[1], pData, DataLen);

	int count = write(vDevHandle, buf, DataLen + 1);

	return count > 0;
}

int UsbHidDevice_Impl::ReadFeatureReport(int RepNo, uint8_t *pBuff, uint32_t BuffSize)
{
	int count = ioctl(vDevHandle, HIDIOCGFEATURE(BuffSize), pBuff);

	return count;
}

bool UsbHidDevice_Impl::WriteFeatureReport(int RepNo, uint8_t *pData, uint32_t DataLen)
{
	if (vpDev == NULL)
		return false;
	
	uint8_t buf[HIDRAW_BUFFER_SIZE + 1];

	buf[0] = RepNo;
	memcpy(&buf[1], pData, DataLen);

	int count = ioctl(vDevHandle, HIDIOCSFEATURE(DataLen + 1), buf);

	return count > 0;
}

bool UsbHidDevice_Impl::GetSerialNumber(std::string &sn)
{
	if (vpDev == NULL)
		return false;
	
	return false;
}

