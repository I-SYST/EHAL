/**-------------------------------------------------------------------------
@file	usb_def.h

@brief	Generic USB definitions

@author	Hoang Nguyen Hoan
@date	Nov. 1, 2014

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
#ifndef __USB_DEF_H__
#define __USB_DEF_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

/** @addtogroup USB
  * @{
  */

typedef enum __USB_Device_Class {
	USB_DEVCLASS_NONE 		= 0,		//!< Use class information in the Interface Descriptors
	USB_DEVCLASS_CDC  		= 2,		//!< Communications and CDC Control
	USB_DEVCLASS_HUB 		= 9,		//!< Hub
	USB_DEVCLASS_BBOARD 	= 17,		//!< Billboard Device Class
	USB_DEVCLASS_DIAG 		= 0xDC,		//!< Diagnostic Device
	USB_DEVCLASS_MISC 		= 0xEF,		//!< Miscellaneous
	USB_DEVCLASS_VENDOR 	= 0xFF		//!< Vendor Specific
} USB_DEVCLASS;

typedef enum __USB_Interface_Class {
	USB_INTRFCLASS_AUDIO 	= 1,		//!< Audio
	USB_INTRFCLASS_CDC 		= 2,		//!< Communication interface class
	USB_INTRFCLASS_HID		= 3,		//!< HID class
	USB_INTRFCLASS_PHY 		= 5,		//!< Physical
	USB_INTFRCLASS_IMG 		= 6,		//!< Image
	USB_INTRFCLASS_PRT 		= 7,		//!< Printer
	USB_INTRFCLASS_MSC 		= 8,		//!< Mass Storage
	USB_INTRFCLASS_HUB 		= 9,		//!< Hub
	USB_INTRFCLASS_DATA 	= 10,		//!< Data interface class
	USB_INTRFCLASS_SMC 		= 11,		//!< Smart Card
	USB_INTRFCLASS_MMC 		= 11,		//!< Smart Card
	USB_INTRFCLASS_CSEC 	= 13,		//!< Content Security
	USB_INTRFCLASS_VIDEO 	= 14,		//!< Video
	USB_INTRFCLASS_HEALTH 	= 15,		//!< Personal Healthcare
	USB_INTRFCLASS_AUDVID 	= 16,		//!< Audio/Video Devices
	USB_INTRFCLASS_DIAG 	= 0xDC,		//!< Diagnostic Device
	USB_INTRFCLASS_WIRELESS = 0xE0,		//!< Wireless Controller
	USB_INTRFCLASS_MISC 	= 0xEF,		//!< Miscellaneous
	USB_INTRFCLASS_APP 		= 0xFE,		//!< Application Specific
	USB_INTRFCLASS_VENDOR 	= 0xFF		//!< Vendor Specific
} USB_INTRFCLASS;


typedef enum __USB_Device_State {
	USB_DEVSTATE_ATTACHED = 1,
	USB_DEVSTATE_POWERED = 2,
	USB_DEVSTATE_DEFAULT = 4,
	USB_DEVSTATE_ADDRESS = 8,
	USB_DEVSTATE_CONFIGURED = 0x10,
	USB_DEVSTATE_SUSPENDED = 0x20
} USB_DEVSTATE;

typedef enum __USB_Standard_Request_Codes {
	USB_REQ_GET_STATUS = 0,				//!< returns status for the specified recipient.
	USB_REQ_CLEAR_FEATURE = 1,			//!< clear or disable a specific feature
	USB_REQ_SET_FEATURE = 3,			//!< set or enable a specific feature.
	USB_REQ_SET_ADDRESS = 5,			//!< sets the device address for all future device accesses.
	USB_REQ_GET_DESCRIPTOR = 6,			//!< returns the specified descriptor if the
										//!< descriptor exists.
	USB_REQ_SET_DESCRIPTOR = 7,			//!< optional and may be used to update existing
										//!< descriptors or new descriptors may be added.
	USB_REQ_GET_CONFIGURATION = 8,		//!< returns the current device configuration value
	USB_REQ_SET_CONFIGURATION = 9,		//!< sets the device configuration.
	USB_REQ_GET_INTERFACE = 10,			//!< returns the selected alternate setting for
										//!< the specified interface.
	USB_REQ_SET_INTERFACE = 11,			//!< allows the host to select an alternate setting
										//!< for the specified interface.
	USB_REQ_SYNCH_FRAME = 12			//!< set and then report an endpoint’s synchronization frame.
} USB_REQ;

typedef enum __USB_Descriptor_Type {
	USB_DESCTYPE_DEVICE 			= 1,
	USB_DESCTYPE_CONFIGURATION 		= 2,
	USB_DESCTYPE_STRING 			= 3,
	USB_DESCTYPE_INTERFACE 			= 4,
	USB_DESCTYPE_ENDPOINT 			= 5,
	USB_DESCTYPE_DEVICE_QUALIFIER 	= 6,
	USB_DESCTYPE_OSC 				= 7,	//!< Other Speed Configuration
	USB_DESCTYPE_INTERFACE_POWER 	= 8,
	USB_DESCTYPE_OTG 				= 9,
	USB_DESCTYPE_DEBUG 				= 10,
	USB_DESCTYPE_IA 				= 11,	//!< Interface Association
	USB_DESCTYPE_HID				= 0x21,
	USB_DESCTYPE_HID_REPORT			= 0x22,
	USB_DESCTYPE_PHYSICAL			= 0x23
} USB_DESCTYPE;

typedef enum __USB_Standard_Feature_Selectors {
	USB_FEATSEL_ENDPOINT_HALT = 0,
	USB_FEATSEL_DEVICE_REMOTE_WAKEUP = 1,	//
	USB_FEATSEL_TEST_MODE = 2
} USB_FEATSEL;


typedef enum __USB_Config_Attribute {
	USB_CONFATT_REMOTE_WAKEUP = 0xA0,
	USB_CONFATT_SELF_POWERED = 0xC0,
	USB_CONFATT_BUS_POWERED = 0x80
} USB_CONFATT;

#pragma pack(push, 1)

typedef enum __USB_Request_Type {
	USB_REQTYPE_DEVICE 		= 0,
	USB_REQTYPE_INTERFACE 	= 1,
	USB_REQTYPE_ENDPOINT 	= 2,
	USB_REQTYPE_OTHER 		= 3,
	USB_REQTYPE_STANDARD	= (0 << 5),
	USB_REQTYPE_CLASS 		= (1 << 5),
	USB_REQTYPE_VEND		= (2 << 5),
	USB_REQTYPE_RSVD		= (3 << 5),
	USB_REQTYPE_DIRDEV		= (0 << 8),
	USB_REQTYPE_DIRHOST		= (1 << 8)
} USB_REQTYPE;

typedef enum __USB_Request_Type_Mask {
	USB_REQTYPE_MASK_RECEIPT 	= 0x1F,
	USB_REQTYPE_MASK_TYPE		= (3 << 5),
	USB_REQTYPE_MASK_DIR		= 0x80
} USB_REQTYPE_MASK;

typedef struct __USB_Setup_Data {
	uint8_t bmRequestType;			//!< Characteristics of request:
									//!< - D7: Data transfer direction
									//!<		- 0 = Host-to-device
									//!<		- 1 = Device-to-host
									//!< - D6...5: Type
									//!<		- 0 = Standard
									//!<		- 1 = Class
									//!<		- 2 = Vendor
									//!<		- 3 = Reserved
									//!< - D4...0: Recipient
									//!<		- 0 = Device
									//!<		- 1 = Interface
									//!<		- 2 = Endpoint
									//!<		- 3 = Other
									//!<		- 4...31 = Reserved
	uint8_t bRequest;				//!< Specific request (refer to Table 9-3)
	uint16_t wValue;				//!< Word-sized field that varies according to request
	uint16_t wIndex;				//!< Index or Offset
									//!< Word-sized field that varies according to request;
									//!< typically used to pass an index or offset
	uint16_t wLength;				//!< Number of bytes to transfer if there is a Data stage
} USB_SETUP_DATA;


typedef struct __USB_Device_Descriptor {
	uint8_t		bLength;			//!< Size of the Descriptor in Bytes (18 bytes)
	uint8_t		bDescriptorType;	//!< Device Descriptor (0x01)
	uint16_t	bcdUSB;				//!< BCD USB Specification Number which device complies too.
	uint8_t 	bDeviceClass;		//!< Class Code (Assigned by USB Org)
									//!< - If equal to Zero, each interface specifies it’s own class code
									//!< - If equal to 0xFF, the class code is vendor specified.
									//!< - Otherwise field is valid Class Code.
	uint8_t		bDeviceSubClass;	//!< Subclass Code (Assigned by USB Org)
	uint8_t 	bDeviceProtocol;	//!< Protocol Code (Assigned by USB Org)
	uint8_t		bMaxPacketSize;		//!< Maximum Packet Size for Zero Endpoint.
									//!< Valid Sizes are 8, 16, 32, 64
	uint16_t	idVendor;			//!< Vendor ID (Assigned by USB Org)
	uint16_t	idProduct;			//!< Product ID (Assigned by Manufacturer)
	uint16_t	bcdDevice;			//!< Device Release Number
	uint8_t		iManufacturer;		//!< Index of Manufacturer String Descriptor
	uint8_t		iProduct;			//!< Index of Product String Descriptor
	uint8_t		iSerialNumber;		//!< Index of Serial Number String Descriptor
	uint8_t		bNumConfigurations;	//!< Number of Possible Configurations
} USB_DEVICE_DESC;

typedef struct __USB_Device_Qualifier_Descriptor {
	uint8_t bLength;				//!< Size of descriptor
	uint8_t bDescriptorType;		//!< Device Qualifier Type
	uint16_t bcdUSB;				//!< USB specification version number (e.g., 0200H for V2.00 )
	uint8_t bDeviceClass;			//!< Class Code
	uint8_t bDeviceSubClass;		//!< SubClass Code
	uint8_t bDeviceProtocol;		//!< Protocol Code
	uint8_t bMaxPacketSize0;		//!< Maximum packet size for other speed
	uint8_t bNumConfigurations;		//!< Number of Other-speed Configurations
	uint8_t bReserved;				//!< Reserved for future use, must be zero
} USB_DEVICE_QUAL;

typedef struct __USB_Config_Descriptor {
	uint8_t		bLength;			//!< Size of Descriptor in Bytes (9 bytes)
	uint8_t		bDescriptorType;	//!< Configuration Descriptor (0x02)
	uint16_t	wTotalLength;		//!< Total length in bytes of data returned
	uint8_t		bNumInterfaces;		//!< Number of Interfaces
	uint8_t		bConfigurationValue;//!< Value to use as an argument to select this configuration
	uint8_t		iConfiguration;		//!< Index of String Descriptor describing this configuration
	uint8_t		bmAttributes;		//!< - D7 Reserved, set to 1. (USB 1.0 Bus Powered)
									//!< - D6 Self Powered
									//!< - D5 Remote Wakeup
									//!< - D4..0 Reserved, set to 0.
	uint8_t		bMaxPower;			//!< Maximum Power Consumption in 2mA units
} USB_CONFIG_DESC;

typedef struct __USB_Other_Speed_Configuration_Descriptor {
	uint8_t bLength;				//!< Size of descriptor
	uint8_t bDescriptorType;		//!< Other_speed_Configuration Type
	uint16_t wTotalLength;			//!< Total length of data returned
	uint8_t bNumInterfaces;			//!< Number of interfaces supported by this speed configuration
	uint8_t bConfigurationValue;	//!< Value to use to select configuration
	uint8_t iConfiguration;			//!< Index of string descriptor
	uint8_t bmAttributes;			//!< Same as Configuration descriptor
	uint8_t bMaxPower;				//!< Same as Configuration descriptor
} USB_OSPEED_CONFIG_DESC;

typedef struct __USB_Interface_Descriptor {
	uint8_t		bLength;			//!< Size of Descriptor in Bytes (9 Bytes)
	uint8_t		bDescriptorType;	//!< Interface Descriptor (0x04)
	uint8_t		bInterfaceNumber;	//!< Zero based Number of this Interface
	uint8_t		bAlternateSetting;	//!< Value used to select alternative setting
	uint8_t		bNumEndpoints;		//!< Number of Endpoints used for this interface
	uint8_t		bInterfaceClass;	//!< Class Code (defined by USB Org), 0xff - vendor specific
	uint8_t		bInterfaceSubClass;	//!< Subclass Code (defined by USB Org)
	uint8_t		bInterfaceProtocol;	//!< Protocol Code (defined by USB Org)
	uint8_t		iInterface;			//!< Index of String Descriptor describing this interface
} USB_INTERFACE_DESC;

#define USB_ENDPADDR_DIRIN(addr)	((addr & 0xF) | 0x80)
#define USB_ENDPADDR_DIROUT(addr)	(addr & 0xF)

typedef enum __USB_Endpoint_Attribute {
	USB_ENDPATT_TRANS_CONTROL	= 0,		//!< Control transfer
	USB_ENDPATT_TRANS_ISO		= 1,		//!< Isochronous transfer
	USB_ENDPATT_TRANS_BULK		= 2,		//!< Bulk transfer
	USB_ENDPATT_TRANS_INT 		= 3,		//!< Interrupt transfer
	USB_ENDPATT_ISO_NOSYNC		= 0,		//!< Iso transfer no sync
	USB_ENDPATT_ISO_ASYNC		= 4,		//!< Iso transfer async
	USB_ENDPATT_ISO_ADAP		= 8,		//!< Iso transfer adaptive
	USB_ENDPATT_ISO_SYNC		= 0xC,		//!< Iso transfer sync
	USB_ENDPATT_ISO_DATA		= 0,		//!< Iso mode data
	USB_ENDPATT_ISO_FB			= 0x10,		//!< Iso mode feedback
	USB_ENDPATT_ISO_EXPFB		= 0x20,		//!< Iso mode explicite feedback
} USB_ENDPATT;

typedef struct __USB_Endpoint_Descriptor {
	uint8_t		bLength;			//!< Size of Descriptor in Bytes (7 bytes)
	uint8_t		bDescriptorType;	//!< Endpoint Descriptor (0x05)
	uint8_t		bEndpointAddress;	//!< Endpoint Address
									//!< - Bits 0..3b Endpoint Number.
									//!< - Bits 4..6b Reserved. Set to Zero
									//!< - Bits 7 Direction 0 = Out, 1 = In (Ignored for Control Endpoints)
	uint8_t		bmAttributes;		//!< - Bits 0..1 Transfer Type
									//!<		- 00 = Control
									//!<		- 01 = Isochronous
									//!<		- 10 = Bulk
									//!<		- 11 = Interrupt
									//!< - Bits 2..7 are reserved.
									//!<
									//!< If Isochronous endpoint,
									//!< - Bits 3..2 = Synchronisation Type (Iso Mode)
									//!<		- 00 = No Synchonisation
									//!<		- 01 = Asynchronous
									//!<		- 10 = Adaptive
									//!<		- 11 = Synchronous
									//!< - Bits 5..4 = Usage Type (Iso Mode)
									//!<		- 00 = Data Endpoint
									//!<		- 01 = Feedback Endpoint
									//!<		- 10 = Explicit Feedback Data Endpoint
									//!<		- 11 = Reserved
	uint16_t	wMaxPacketSize;		//!< Maximum Packet Size this endpoint is capable of sending or receiving
	uint8_t		bInterval;			//!< Interval for polling endpoint data transfers. Value in frame counts.
									//!< Ignored for Bulk & Control Endpoints. Isochronous must equal 1 and
									//!< field may range from 1 to 255 for interrupt endpoints.
} USB_ENDPOINT_DESC;

typedef struct __USB_Interface_Association_Descriptor {
	uint8_t bLength;				//!< Size of this descriptor in bytes.
	uint8_t bDescriptorType;		//!< INTERFACE ASSOCIATION Descriptor.
	uint8_t bFirstInterface;		//!< Interface number of the first interface
									//!< that is associated with this function.
	uint8_t bInterfaceCount;		//!< Number of contiguous interfaces that are
									//!< associated with this function.
	uint8_t bFunctionClass;			//!< Class code (assigned by USB-IF).
									//!< A value of zero is not allowed in this descriptor.
									//!< - If this field is FFH, the function class is
									//!< vendor- specific.
									//!< - All other values are reserved for assignment
									//!< by the USB-IF.
	uint8_t bFunctionSubClass;		//!< Subclass code (assigned by USB-IF).
									//!< - If the bFunctionClass field is not set to
									//!< - FFH all values are reserved for assignment
									//!< by the USB- IF.
	uint8_t bFunctionProtocol;		//!< Protocol code (assigned by USB-IF). These
									//!< codes are qualified by the values of the
									//!< bFunctionClass and bFunctionSubClass fields.
	uint8_t iFunction;				//!< Index of string descriptor describing this function.
} USB_IAD_DESC;

typedef struct __USB_OTG_Descriptor {
	uint8_t bLength;				//!< Size of Descriptor
	uint8_t bDescriptorType;		//!< OTG type = 9
	uint8_t bmAttributes;			//!< Attribute Fields
									//!< - D7...3: Reserved (reset to zero)
									//!< - D2: ADP support
									//!< - D1: HNP support
									//!< - D0: SRP support
	uint16_t bcdOTG;				//!< OTG
									//!< OTG and EH supplement release number in binary-coded
									//!< decimal (i.e. 2.0 is 0200H). This field identifies
									//!< the release of the OTG and EH supplement with
									//!< which the device and its descriptors are compliant.
} USB_OTG_DESC;

typedef struct __USB_String_Language_Descriptor {
	uint8_t		bLength;			//!< Size of Descriptor in Bytes
	uint8_t		bDescriptorType;	//!< String Descriptor (0x03)
	uint16_t	wLangId[1];			//!< Array of supported language Code Zero (e.g. 0x0409 English - United States)
} USB_STRLANG_DESC;

typedef struct __USB_String_Descriptor {
	uint8_t		bLength;			//!< Size of Descriptor in Bytes
	uint8_t		bDescriptorType;	//!< String Descriptor (0x03)
	uint16_t	wStr[1];			//!< Unicode String
} USB_STR_DESC;


#pragma pack(pop)

/** @} end group USB */

#endif	// __USB_DEF_H__
