/*--------------------------------------------------------------------------
File   : usb_cdcdef.h

Author : Hoang Nguyen Hoan          Nov. 11, 2014

Desc   : Generic USB Communication Class definitions

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#ifndef __USB_CDCDEF_H__
#define __USB_CDCDEF_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

typedef enum _USB_Communication_Class_Subclass {
	USB_CDC_SUBCLASS_NONE	= 0,	// Reserved
	USB_CDC_SUBCLASS_DLCM 	= 1,	// Direct Line Control Model
	USB_CDC_SUBCLASS_ACM 	= 2,	// Abstract Control Model
	USB_CDC_SUBCLASS_TCM 	= 3,	// Telephone Control Model
	USB_CDC_SUBCLASS_MCCM 	= 4,	// Multi-Channel Control Model
	USB_CDC_SUBCLASS_CCM 	= 5,	// CAPI Control Model
	USB_CDC_SUBCLASS_ECM	= 6,	// Ethernet Networking Control Model
	USB_CDC_SUBCLASS_ATM 	= 7, 	// ATM Networking Control Model
	USB_CDC_SUBCLASS_WMCM 	= 8,	// Wireless Handset Control Model
	USB_CDC_SUBCLASS_DMM 	= 9,	// Device Management Model
	USB_CDC_SUBCLASS_MDLM 	= 10,	// Mobile Direct Line Model
	USB_CDC_SUBCLASS_OBEX 	= 11,	// OBEX Model
	USB_CDC_SUBCLASS_EEM 	= 12, 	// Ethernet Emulation Model
	USB_CDC_SUBCLASS_NCM 	= 13,	// Network Control Model
	USB_CDC_SUBCLASS_MBIM 	= 14,	// Mobile Broadband Interface Model
	USB_CDC_SUBCLASS_VENDOR = 0x80	// Vendor specific start here
} USB_CDC_SUBCLASS;

typedef enum _USB_Communication_Class_Protocol {
	USB_CDC_PROT_NONE 		= 0,	// No protocol
	USB_CDC_PROT_ITUTV250 	= 1, 	// ITU-T V.250 AT Commands: V.250 etc
	USB_CDC_PROT_PCCA101 	= 2,	// PCCA-101 AT Commands defined by PCCA-101
	USB_CDC_PROT_PCCA101A0 	= 3, 	// PCCA-101 AT Commands defined by PCCA-101 & Annex O
	USB_CDC_PROT_GSM 		= 4,	// GSM 7.07 AT Commands defined by GSM 07.07
	USB_CDC_PROT_3GPP 		= 5, 	// 3GPP 27.07 AT Commands defined by 3GPP 27.007
	USB_CD_PROT_CS0017 		= 6,	// C-S0017-0 AT Commands defined by TIA for CDMA
	USB_CDC_PROT_EEM 		= 7,	// USB EEM Ethernet Emulation Model
	USB_CDC_PROT_EXTRN 		= 0xFE,	// External Protocol: Commands defined by
									// Command Set Functional Descriptor
	USB_CDC_PROT_VENDOR = 0xFF		// Vendor specific
} USB_CDC_PROT;

typedef enum _USB_CDC_Data_Class_Protocol {
	USB_CDCDATA_PROT_NONE	= 0,	// None
	USB_CDCDATA_PROT_NCM 	= 1,	// Network Transfer Block
	USB_CDCDATA_PROT_MBIM 	= 2,	// Network Transfer Block (IP + DSS)
	USB_CDCDATA_PROT_I430	= 0x30,	// I.430 Physical interface protocol for ISDN BRI
	USB_CDCDATA_PROT_HDLC 	= 0x31,	// ISO/IEC 3309-1993 HDLC
	USB_CDCDATA_PROT_TRANS 	= 0x32,	// Transparent
	USB_CDCDATA_PROT_Q921M 	= 0x50,	// Q.921M Management protocol for Q.921 data link protocol
	USB_CDCDATA_PROT_Q921 	= 0x51,	// Q.921 Data link protocol for Q.931
	USB_CDCDATA_PROT_Q921TM	= 0x52,	// Q921TM TEI-multiplexor for Q.921 data link protocol
	USB_CDCDATA_PROT_V42BIS	= 0x90,	// V.42bis Data compression procedures
	USB_CDCDATA_PROT_Q931 	= 0x91,	// Q.931/Euro- ISDN Euro-ISDN protocol control
	USB_CDCDATA_PROT_V120 	= 0x92,	// V.120 V.24 rate adaptation to ISDN
	USB_CDCDATA_PROT_CAPI20	= 0x93,	// CAPI2.0 CAPI Commands
	USB_CDCDATA_PROT_HOST 	= 0xFD,	// Host based driver.
									// Note: This protocol code should only be
									// used in messages between host and device
									// to identify the host driver portion of a
									// protocol stack.
	USB_CDCDATA_PROT_CDC 	= 0xFE,	// CDC Specification The protocol(s) are described
									// using a Protocol Unit Functional Descriptors on
									// Communications Class Interface
	USB_CDCDATA_PROT_VENDOR = 0xFF	// Vendor specific
} USB_CDCDATA_PROT;

typedef enum _USB_Functional_Descriptor_Type {
	USB_FUNCTYPE_CS_INTERFACE = 0x24,
	USB_FUNCTYPE_CS_ENDPOINT = 0x25
} USB_FUNCTYPE;


typedef enum _USB_Communication_Fonctional_Descriptor_Subtype {
	USB_CDC_FSUBTYPE_HEADER = 0,	// Header Functional Descriptor, which marks the
									// beginning of the concatenated set of functional
									// descriptors for the interface.
	USB_CDC_FSUBTYPE_CM 	= 1,	// Call Management Functional Descriptor.
	USB_CDC_FSUBTYPE_ACM 	= 2,	// Abstract Control Management Functional Descriptor.
	USB_CDC_FSUBTYPE_DLM 	= 3,	// Direct Line Management Functional Descriptor.
	USB_CDC_FSUBTYPE_RINGER = 4,	// Telephone Ringer Functional Descriptor.
	USB_CDC_FSUBTYPE_CLS 	= 5,	// Telephone Call and Line State Reporting Capabilities
									// Functional Descriptor.
	USB_CDC_FSUBTYPE_UNION 	= 6, 	// Union Functional Descriptor
	USB_CDC_FSUBTYPE_COUNTRY = 7,	// Country Selection Functional Descriptor
	USB_CDC_FSUBTYPE_OPMODE	= 8,	// Telephone Operational Modes Functional Descriptor
	USB_CDC_FSUBTYPE_USBTERM = 9,	// USB Terminal Functional Descriptor
	USB_CDC_FSUBTYPE_NCTERM = 10,	// Network Channel Terminal Descriptor
	USB_CDC_FSUBTYPE_PROTOCOL = 11,	// Protocol Unit Functional Descriptor
	USB_CDC_FSUBTYPE_EXT 	= 12,	// Extension Unit Functional Descriptor
	USB_CDC_FSUBTYPE_MCM 	= 13,	// Multi-Channel Management Functional Descriptor
	USB_CDC_FSUBTYPE_CAPI 	= 14,	// CAPI Control Management Functional Descriptor
	USB_CDC_FSUBTYPE_ETH 	= 15,	// Ethernet Networking Functional Descriptor
	USB_CDC_FSUBTYPE_ATM 	= 16,	// ATM Networking Functional Descriptor
	USB_CDC_FSUBTYPE_WHCM 	= 17, 	// Wireless Handset Control Model Functional Descriptor
	USB_CDC_FSUBTYPE_MDLM 	= 18,	// Mobile Direct Line Model Functional Descriptor
	USB_CDC_FSUBTYPE_MDLMDET = 19,	// MDLM Detail Functional Descriptor
	USB_CDC_FSUBTYPE_DMM 	= 20,	// Device Management Model Functional Descriptor
	USB_CDC_FSUBTYPE_OBEX 	= 21,	// OBEX Functional Descriptor
	USB_CDC_FSUBTYPE_CMD 	= 22,	// Command Set Functional Descriptor
	USB_CDC_FSUBTYPE_CMDDET = 23,	// Command Set Detail Functional Descriptor
	USB_CDC_FSUBTYPE_TCM 	= 24,	// Telephone Control Model Functional Descriptor
	USB_CDC_FSUBTYPE_OBEXSID = 25,	// OBEX Service Identifier Functional Descriptor
	USB_CDC_FSUBTYPE_NCM 	= 26,	// NCM Functional Descriptor
	USB_CDC_FSUBTYPE_MBIM 	= 27,	// MBIM Functional Descriptor
	USB_CDC_FSUBTYPE_MBIMEXT = 28,	// MBIM Extended Functional Descriptor
	USB_CDC_FSUBTYPE_VENDOR = 0x80	// Vendor specific start here
} USB_CDC_FUNCSUBTYPE;

typedef enum _USB_Data_Functional_Descriptor_Subtype {
	USB_CDCDATA_FSUBTYPE_HEADER = 0,	// Header Functional Descriptor, which
										// marks the beginning of the concatenated
										// set of functional descriptors for the
										// interface.
	USB_CDCDATA_FSUBTYPE_VENDOR = 0x80	// Vendor specific start here
} USB_CDCDATA_FSUBTYPE;

typedef enum _USB_Communication_Request_Code {
	USB_CDC_REQ_SEND_ENCAPS_CMD 	= 0,
	USB_CDC_REQ_GET_ENCAPS_RESP 	= 1,
	USB_CDC_REQ_SET_COMM_FEAT		= 2,
	USB_CDC_REQ_GET_COMM_FEAT		= 3,
	USB_CDC_REQ_CLEAR_COMM_FEAT		= 4,
	USB_CDC_REQ_SET_AUX_LINE_STATE 	= 0x10,
	USB_CDC_REQ_SET_HOOK_STATE		= 0x11,
	USB_CDC_REQ_PULSE_SETUP			= 0x12,
	USB_CDC_REQ_SEND_PULSE			= 0x13,
	USB_CDC_REQ_SET_PULSE_TIME		= 0x14,
	USB_CDC_REQ_RING_AUX_JACK		= 0x15,
	USB_CDC_REQ_SET_LINE_CODING		= 0x20,
	USB_CDC_REQ_GET_LINE_CODING		= 0x21,
	USB_CDC_REQ_SET_CTRL_LINE_STATE = 0x22,
	USB_CDC_REQ_SEND_BREAK			= 0x23,
	USB_CDC_REQ_SET_RINGER_PARMS	= 0x30,
	USB_CDC_REQ_GET_RINGER_PARMS	= 0x31,
	USB_CDC_REQ_SET_OPER_PARMS 		= 0x32,
	USB_CDC_REQ_GET_OPER_PARMS		= 0x33,
	USB_CDC_REQ_SET_LINE_PARMS		= 0x34,
	USB_CDC_REQ_GET_LINE_PARMS		= 0x35,
	USB_CDC_REQ_DIAL_DIGITS			= 0x36,
	USB_CDC_REQ_SET_UNIT_PARAMETER	= 0x37,
	USB_CDC_REQ_GET_UNIT_PARAMETER	= 0x38,
	USB_CDC_REQ_CLEAR_UNIT_PARAMETER= 0x39,
	USB_CDC_REQ_GET_PROFILE			= 0x3A,
	USB_CDC_REQ_SET_ETH_MCAST_FLTR	= 0x40,
	USB_CDC_REQ_SET_ETH_PWR_MGNT_PAT_FLTR	= 0x41,
	USB_CDC_REQ_GET_ETH_PWR_MGNT_PAT_FLTR	= 0x42,
	USB_CDC_REQ_SET_ETH_PACKET_FLTR	= 0x43,
	USB_CDC_REQ_GET_ETH_STATS		= 0x44,
	USB_CDC_REQ_SET_ATM_DATA_FORMAT	= 0x50,
	USB_CDC_REQ_GET_ATM_DEV_STATS	= 0x51,
	USB_CDC_REQ_SET_ATM_DFLT_VC		= 0x52,
	USB_CDC_REQ_GET_ATM_VC_STATS	= 0x53,
	USB_CDC_REQ_MDLM				= 0x60,	// Start here Semantic-Model specific Requests
	USB_CDC_REQ_GET_NTB_PARAM		= 0x80,
	USB_CDC_REQ_GET_NET_ADDRESS		= 0x81,
	USB_CDC_REQ_SET_NET_ADDRESS		= 0x82,
	USB_CDC_REQ_GET_NTB_FORMAT		= 0x83,
	USB_CDC_REQ_SET_NTB_FORMAT		= 0x84,
	USB_CDC_REQ_GET_NTB_INPUT_SIZE	= 0x85,
	USB_CDC_REQ_SET_NTB_INPUT_SIZE	= 0x86,
	USB_CDC_REQ_GET_MAX_DATAGRAM_SIZE = 0x87,
	USB_CDC_REQ_SET_MAX_DATAGRAM_SIZE = 0x88,
	USB_CDC_REQ_GET_CRC_MODE		= 0x89,
	USB_CDC_REQ_SET_CRC_MODE		= 0x8A
} USB_CDC_REQ;

typedef enum _USB_CDC_Notification_Code {
	USB_CDC_NOTIFY_NET_CONN					= 0,
	USB_CDC_NOTIFY_RESP_AVAIL				= 1,
	USB_CDC_NOTIFY_AUX_JACK_HOOK_STATE 		= 8,
	USB_CDC_NOTIFY_RING_DETECT				= 9,
	USB_CDC_NOTIFY_SERIAL_STATE 			= 0x20,
	USB_CDC_NOTIFY_CALL_STATE_CHANGE 		= 0x28,
	USB_CDC_NOTIFY_LINE_STATE_CHANGE 		= 0x29,
	USB_CDC_NOTIFY_CONNECTION_SPEED_CHANGE 	= 0x2A,
	USB_CDC_NOTIFY_MDML_SEMANTIC			= 0x40	// MDML SEMANTIC-MODEL-SPECIFIC NOTIFICATION start here
} USB_CDC_NOTIFY;

// USB_CDC_REQ_SET_CTRL_LINE_STATE
#define USB_CDC_CTRL_LINE_STATE_DTR		(1<<0)	// Data terminal ready
#define USB_CDC_CTRL_LINE_STATE_RTS		(1<<1)	// Ready to send

// USB_CDC_NOTIFY_LINE_STATE_CHANGE
#define	USB_CDC_LINE_STATE_DCD			(1<<0) // Data carrier detect. This is asserted when a connection has been established with remote equipment
#define	USB_CDC_LINE_STATE_DSR			(1<<1) // Data Set Ready. This is asserted to indicate an active connection. (CTS)
#define	USB_CDC_LINE_STATE_BRK			(1<<2) // Break detection state
#define	USB_CDC_LINE_STATE_RI			(1<<3) // Ring indicator state.
#define	USB_CDC_LINE_STATE_FE			(1<<4) // Framing error detected.
#define	USB_CDC_LINE_STATE_PE			(1<<5) // Parity error detected.
#define	USB_CDC_LINE_STATE_OE			(1<<6) // Overrun error

#pragma pack(push, 1)

typedef struct _USB_Functional_Descriptor {
	uint8_t bFunctionLength;		// Size of this descriptor.
	uint8_t bDescriptorType;		// CS_INTERFACE, as defined in Table 12.
	uint8_t bDescriptorSubtype;		// Identifier (ID) of functional descriptor.
									// For a list of the supported values, see Table 13.
	uint8_t bData[1];				// Data array. These fields will vary depending
									// on the functional descriptor being represented.
} USB_FCTDESC;

typedef struct _USB_Header_Functional_Descriptor {
	uint8_t bFunctionLength;		// Size of this descriptor in bytes.
	uint8_t bDescriptorType;		// CS_INTERFACE descriptor type.
	uint8_t bDescriptorSubtype;		// Header functional descriptor subtype as defined in Table 13.
	uint16_t bcdCDC;				// USB Class Definitions for Communications
									// Devices Specification release number in binary-coded decimal.
} USB_CDC_HEADER_DESC;

typedef struct _USB_Union_Functional_Descriptor {
	uint8_t bFunctionLength;		// Size of this functional descriptor, in bytes.
	uint8_t bDescriptorType;		// CS_INTERFACE
	uint8_t bDescriptorSubtype;		// Union Functional Descriptor SubType as defined in Table 13.
	uint8_t bControlInterface;		// The interface number of the Communications or Data Class
									// interface, designated as the controlling interface for the union.*
	uint8_t bSubordinateInterf[1];	// Interface number of first subordinate interface in the union. *
} USB_CDC_UNION_DESC;

typedef struct _USB_Country_Functional_Descriptor {
	uint8_t bFunctionLength;		// Size of this functional descriptor, in bytes.
	uint8_t bDescriptorType;		// CS_INTERFACE
	uint8_t bDescriptorSubtype;		// Country Selection Functional Descriptor Subtype
									// as defined in Table 13.
	uint8_t iCountryCodeRelDate;	// Index of a string giving the release date for
									// the implemented ISO 3166 Country Codes.
									// Date shall be presented as ddmmyyyy with dd=day,
									///mm=month, and yyyy=year.
	uint16_t wCountryCode[1];		// Country code in the format as defined in [ISO3166],
									// release date as specified in offset 3 for the first
									// supported country.
} USB_CDC_COUNTRY_DESC;

typedef struct _USB_Call_Management_Functional_Descriptor {
	uint8_t bFunctionLength;		// Size of this functional descriptor, in bytes.
	uint8_t bDescriptorType;		// CS_INTERFACE
	uint8_t bDescriptorSubtype;		// ￼Call Management functional descriptor subtype,
									// as defined in [USBCDC1.2].
	uint8_t bmCapabilities;			// The capabilities that this configuration supports:
									// D7..D2: RESERVED (Reset to zero)
									// D1: 0 - Device sends/receives call management
									//		   information only over the Communications
									//		   Class interface.
									//	   1 - Device can send/receive call management
									//	  	   information over a Data Class interface.
									// D0: 0 - Device does not handle call management itself.
									//	   1 - Device handles call management itself.
									// The previous bits, in combination, identify which call
									// management scenario is used. If bit D0 is reset to 0,
									// then the value of bit D1 is ignored. In this case,
									// bit D1 is reset to zero for future compatibility.
	uint8_t bDataInterface;			// Interface number of Data Class interface optionally
									// used for call management.
} USB_CDC_CM_DESC;

typedef struct _USB_Abstract_Control_Management_Functional_Descriptor {
	uint8_t bFunctionLength;		// Size of this functional descriptor, in bytes.
	uint8_t bDescriptorType;		// CS_INTERFACE
	uint8_t bDescriptorSubtype;		// Abstract Control Management functional descriptor
									// subtype as defined in [USBCDC1.2].
	uint8_t bmCapabilities;			// The capabilities that this configuration supports.
									// (A bit value of zero means that the request is
									// not supported.)
									// D7..D4: RESERVED (Reset to zero)
									// D3: 1 - Device supports the notification Network_Connection.
									// D2: 1 - Device supports the request Send_Break
									// D1: 1 - Device supports the request combination of
									//		   Set_Line_Coding, Set_Control_Line_State,
									//		   Get_Line_Coding, and the notification Serial_State.
									// D0: 1 - Device supports the request combination of
									//		   Set_Comm_Feature, Clear_Comm_Feature, and
									//			Get_Comm_Feature.
									// The previous bits, in combination, identify which
									// requests/notifications are supported by a
									// CommunicationsClass interface with the SubClass code
									// of Abstract Control Model.
} USB_CDC_ACM_DESC;

typedef struct _USB_Direct_Line_Management_Functional_Descriptor {
	uint8_t bFunctionLength;		// Size of this functional descriptor, in bytes.
	uint8_t bDescriptorType;		// CS_INTERFACE
	uint8_t bDescriptorSubtype;		// Direct Line Management functional descriptor
									// subtype, as defined in [USBCDC1.2].
	uint8_t bmCapabilities;			// The capabilities that this configuration supports.
									// (A value of zero means that the request or
									// notification is not supported.)
									// D7..D3: RESERVED (Reset to zero)
									// D2: 1 - Device requires extra Pulse_Setup request
									// during pulse dialing sequence to disengage holding
									// circuit. (see Section 6.3.6)
									// D1: 1 - Device supports the request combination
									// of Set_Aux_Line_State, Ring_Aux_Jack, and notific
} USB_CDC_DLM_DESC;

#pragma pack(pop)

#endif	// __USB_CDCDEF_H__
