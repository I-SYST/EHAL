/*--------------------------------------------------------------------------
File   : usb_mscdef.h

Author : Hoang Nguyen Hoan          Nov. 11, 2014

Desc   : Generic USB Mass Storage Class definitions

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
#ifndef __USB_MSCDEF_H__
#define __USB_MSCDEF_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

typedef enum _USB_MSC_Subclass {
	USB_MSC_SUBCLASS_SCSIX 	= 0,	// SCSI command set not reported, De facto use
	USB_MSC_SUBCLASS_RBC	= 1,	// Allocated by USB-IF for RBC. RBC is defined
									// outside of USB.
	USB_MSC_SUBCLASS_MMC5	= 2, 	// Allocated by USB-IF for MMC-5. MMC-5 is defined
									// outside of USB.
	USB_MSC_SUBCLASS_UFI	= 4,	// Specifies how to interface Floppy Disk Drives to USB
	USB_MSC_SUBCLASS_SCSI	= 6, 	// SCSI transparent command set, allocated by USB-IF
									// for SCSI. SCSI standards are defined outside of USB.
	USB_MSC_SUBCLASS_LSDFS	= 7,	// LSDFS specifies how host has to negotiate access
									// before trying SCSI
	USB_MSC_SUBCLASS_IEEE1667 = 8,	// Allocated by USB-IF for IEEE 1667. IEEE 1667 is
									// defined outside of USB.
	USB_MSC_SUBCLASS_VENDOR	= 0xFF	// Specific to device vendor
} USB_MSC_SUBCLASS;

typedef enum _USB_MSC_Protocol {
	USB_MSC_PROT_CBII		= 0,	// USB Mass Storage Class Control/Bulk/Interrupt
									// (CBI) Transport (with command completion interrupt)
	USB_MSC_PROT_CBI		= 1,	//  USB Mass Storage Class Control/Bulk/Interrupt
									// (CBI) Transport (with no command completion interrupt)
	USB_MSC_PROT_BULK		= 0x50,	// USB Mass Storage Class Bulk-Only (BBB) Transport
	USB_MSC_PROT_UAS		= 0x62,	// Allocated by USB-IF for UAS. UAS is defined outside of USB.
	USB_MSC_PROT_VENDOR		= 0xFF	// Specific to device vendor.
} USB_MSC_PROT;

typedef enum _USB_MSC_Request_Code {
	USB_MSC_REQCODE_ADSC	= 0,	// Accept Device-Specific Command (ADSC)
									// Assigned in context by USB Mass Storage Class
									// Control/Bulk/Interrupt (CBI) Transport, also aliases
									// core USB request 00h Get Status.
	USB_MSC_REQCODE_GET		= 0xFC,	// Get Requests. Assigned by Lockable Storage Devices
									// Feature Specification
	USB_MSC_REQCODE_PUT		= 0xFD,	// Put Requests. Assigned by Lockable Storage Devices
									// Feature Specification
	USB_MSC_REQCODE_MAXLUN	= 0xFE,	// Get Max LUN (GML). Assigned by USB Mass Storage
									// Class Bulk-Only (BBB) Transport
	USB_MSC_REQCODE_BOMSR	= 0xFF	// Bulk-Only Mass Storage Reset (BOMSR)
									// Assigned by USB Mass Storage Class Bulk-Only (BBB) Transport
} USB_MSC_REQCODE;

#define USB_MSC_CBW_SIGNATURE		0x43425355
#define USB_MSC_CSW_SIGNATURE		0x53425355

typedef enum _USB_MSC_Command_Status {
	USB_MSC_CMDSTATUS_PASS	= 0,
	USB_MSC_CMDSTATUS_FAILED	= 1,
	USB_MSC_CMDSTATUS_PHASE_ERR	= 2
} USB_MSC_CMDSTATUS;

#pragma pack(push, 1)

typedef struct _USB_Msc_Command_Block_Wrapper {
	uint32_t dCBWSignature;			// The signature field shall contain the value
									// 43425355h (little endian), indicating a CBW.
	uint32_t dCBWTag;				// A Command Block Tag sent by the host. The
									// device shall echo the contents of this field
									// back to the host in the dCSWTag field of the
									// associated CSW
	uint32_t dCBWDataTransferLength;// The number of bytes of data that the host
									// expects to transfer on the Bulk-In or
									// Bulk-Out endpoint (as indicated by the
									// Direction bit) during the execution of this
									// command.
	uint8_t bmCBWFlags;				// The bits of this field are defined as follows:
									//	Bit 7 : Direction - the device shall ignore
									//			this bit if the dCBWDataTransferLength
									//			field is zero, otherwise:
									//				0 = Data-Out from host to the device,
									//				1 = Data-In from the device to the host.
									// 	Bit 6 : Obsolete. The host shall set this bit to zero.
									//	Bits 5..0 : Reserved - the host shall set these bits to zero
	uint8_t bCBWLUN;				// The device Logical Unit Number (LUN) to which
									// the command block is being sent.
	uint8_t bCBWCBLength;			// The valid length of the CBWCB in bytes
	uint8_t CBWCB[16];				// The command block to be executed by the device.
} USB_MSC_CBW;

typedef struct _USB_Msc_Command_Status_Wrapper {
	uint32_t dCSWSignature;			// Signature that helps identify this data packet
									// as a CSW. The signature field shall contain the
									// value 53425355h (little endian), indicating CSW
	uint32_t dCSWTag;				// The device shall set this field to the value
									// received in the dCBWTag of the associated CBW
	uint32_t dCSWDataResidue;		// See USB MSC Bulk only docs for details
	uint8_t	bCSWStatus;				// bCSWStatus indicates the success or failure
} USB_MSC_CSW;

#pragma pack(pop)

#endif	// __USB_MSCDEF_H__
