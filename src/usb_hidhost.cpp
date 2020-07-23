/*---------------------------------------------------------------------------
 File : usb_hidhost.cpp
 
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
 Modified by			Date					Description
 Hoan						juin 2009			Ekomini
 ----------------------------------------------------------------------------*/
#include "usb/usb_hidhost_impl.h"

UsbHidDevice *UsbHidDevice::Create() 
{ 
	//return new UsbHidDevice_Impl; 
	return NULL;
} 
