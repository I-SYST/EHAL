/**-------------------------------------------------------------------------
@file	usb_hiddef.h

@brief	Generic USB HID Class definitions

@author	Hoang Nguyen Hoan
@date	Nov. 11, 2014

@license

Copyright (c) 2014, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

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

----------------------------------------------------------------------------*/
#ifndef __USB_HIDDEF_H__
#define __USB_HIDDEF_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

/** @addtogroup USB
  * @{
  */

typedef enum __USB_HID_Subclass {
	USB_HID_SUBCLASS_NONE = 0,
	USB_HID_SUBCLASS_BOOT = 1,		//!< Boot Interface Subclass
} USB_HID_SUBCLASS;

typedef enum __USB_HID_Protocol {
	USB_HID_PROT_NONE		= 0,
	USB_HID_PROT_KEYBOARD	= 1,
	USB_HID_PROT_MOUSE		= 2
} USB_HID_PROT;

typedef enum __USB_HID_Request_Codes {
	USB_HID_REQ_GET_REPORT 		= 1,
	USB_HID_REQ_GET_IDLE 		= 2,
	USB_HID_REQ_GET_PROTOCOL 	= 3,
	USB_HID_REQ_SET_REPORT 		= 9,
	USB_HID_REQ_SET_IDLE 		= 10,
	USB_HID_REQ_SET_PROTOCOL 	= 11
} USB_HID_REQ;

// The wValue field in the USB_SETUP_DATA specifies the Report Type in the high
// byte and the Report ID in the low byte. Set Report ID to 0 (zero) if Report IDs
// are not used.
#define USB_HID_REPID_MASK		0xFF
#define USB_HID_REPTYPE_MASK	0xFF00

typedef enum __USB_HID_Report_type {
	USB_HID_REPTYPE_INPUT 	= (1 << 8),
	USB_HID_REPTYPE_OUTPUT 	= (2 << 8),
	USB_HID_REPTYPE_FEATURE = (3 << 8)
} USB_HID_REPTYPE;

typedef enum __USB_Usage_Page {
	USB_HID_USAGEPAGE_UNDEF		= 0,
	USB_HID_USAGEPAGE_DESKTOP	= 1,	//!< Generic Desktop Controls
	USB_HID_USAGEPAGE_SIM		= 2,	//!< Simulation Controls
	USB_HID_USAGEPAGE_VR		= 3,	//!< VR Controls
	USB_HID_USAGEPAGE_SPORT		= 4,	//!< Sport Controls
	USB_HID_USAGEPAGE_GAME		= 5,	//!< Game Controls
	USB_HID_USAGEPAGE_DEVICE	= 6,	//!< Generic Device Controls
	USB_HID_USAGEPAGE_KEYBOARD	= 7,	//!< Keyboard/Keypad
	USB_HID_USAGEPAGE_LED		= 8,	//!< LEDs
	USB_HID_USAGEPAGE_BUTTON	= 9,	//!< Button
	USB_HID_USAGEPAGE_ORDINAL	= 0xA,	//!< Ordinal
	USB_HID_USAGEPAGE_TEL		= 0xB,	//!< Telephony
	USB_HID_USAGEPAGE_CONSUMER	= 0xC,	//!< Consumer
	USB_HID_USAGEPAGE_DIG		= 0xD,	//!< Digitizer
	USB_HID_USAGEPAGE_PID		= 0xF,	//!< PID Page USB Physical Interface Device
										//!< definitions for force feedback and related devices.
	USB_HID_USAGEPAGE_UNICODE	= 0x10,	//!< Unicode
	USB_HID_USAGEPAGE_ALPHADISPLAY	= 0x14,	//!< Alphanumeric Display
	USB_HID_USAGEPAGE_MEDICAL	= 0x40,		//!< Medical Instruments
	USB_HID_USAGEPAGE_MONITOR	= 0x80,	//!< 0x80-0x83 Monitor pages USB Device
										//!< Class Definition for Monitor Devices
	USB_HID_USAGEPAGE_POWER		= 0x84,	//!< 0x84-0x87 Power pages USB Device Class
										//!< Definition for Power Devices
	USB_HID_USAGEPAGE_BARCODE	= 0x8C,	//!< Bar Code Scanner page
	USB_HID_USAGEPAGE_SCALE		= 0x8D,	//!< Scale page
	USB_HID_USAGEPAGE_MSR		= 0x8E,	//!< Magnetic Stripe Reading (MSR) Devices
	USB_HID_USAGEPAGE_POS		= 0x8F,	//!< Reserved Point of Sale pages
	USB_HID_USAGEPAGE_CAM		= 0x90,	//!< Camera Control Page USB Device Class Definition
	USB_HID_USAGEPAGE_ARCADE	= 0x91,	//!< Arcade Page
										//!< OAAF Definitions for arcade and coinop related Devices
	USB_HID_USAGEPAGE_VENDOR	= 0xFF00//!< 0xFF00-0xFFFF Vendor-defined
} USB_HID_USAGEPAGE;

#pragma pack(push, 1)

typedef struct __USB_HID_Report_Descriptor {
	uint8_t bDescriptorType;		//!< type of class descriptor. See Section 7.1.2:
									//!< Set_Descriptor Request for a table of class
									//!< descriptor constants.
	uint16_t wDescriptorLength;		//!< total size of the Report descriptor.
} USB_HID_REP_DESC;

typedef struct __USB_HID_Descriptor {
	uint8_t bLength;				//!< Total size of the HID descriptor.
	uint8_t bDescriptorType;		//!< type of HID descriptor.
	uint16_t bcdHID;				//!< HID Class Specification release.
	uint8_t bCountryCode;			//!< country code of the localized hardware.
	uint8_t bNumDescriptors;		//!< number of class descriptors (always at
									//!< least one i.e. Report descriptor.)
	USB_HID_REP_DESC RepDesc[1];	//!< Array of HID report descriptor class
} USB_HID_DESC;


#pragma pack(pop)

/** @} end group USB */

#endif	// __USB_HIDDEF_H__
