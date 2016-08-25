/*
 * lpc11Uxx_usbmsc.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPC11UXX_USBMSC_H__
#define __LPC11UXX_USBMSC_H__

#include "usb_mscdef.h"
#include "usbd_core_lpcxx.h"

#pragma pack(push, 1)

typedef struct {
	uint32_t 	mem_base;	// Base memory location from where the stack can
							// allocate data and buffers.
	uint32_t 	mem_size;	// The size of memory buffer which stack can use.
	uint8_t * 	InquiryStr;	// Pointer to the 28 character string. This string
							// is sent in response to the SCSI Inquiry command.
	uint32_t 	BlockCount;	// Number of blocks present in the mass storage device
	uint32_t 	BlockSize;	// Block size in number of bytes
	uint32_t 	MemorySize;	// Memory size in number of bytes
	uint8_t 	*intf_desc;	// Pointer to the interface descriptor within the
							// descriptor array (high_speed_desc) passed to Init()
							// through USB_CORE_DESCS_T structure. The stack assumes
							// both HS and FS use same BULK endpoints.

	//MSC Write callback function.
	//
	// This function is provided by the application software. This function
	// gets called when host sends a write command.
	//
	// Parameters:
	//		[in]	 offset	Destination start address.
	//		[in,out] src	Pointer to a pointer to the source of data.
	//						Pointer-to-pointer is used to implement zero-copy buffers. See Zero-Copy Data Transfer model for more details on zero-copy concept.
	//		[in]	 length	Number of bytes to be written.
	// Returns:
	// 		Nothing.
	void (*MSC_Write )(uint32_t offset, uint8_t **src, uint32_t length, uint32_t high_offset);

	// MSC Read callback function.
	//
	// This function is provided by the application software. This function
	// gets called when host sends a read command.
	//
	// Parameters:
	//		[in]	 offset	Source start address.
	//		[in,out] dst	Pointer to a pointer to the source of data.
	//						The MSC function drivers implemented in stack
	//						are written with zero-copy model. Meaning the
	//						stack doesn't make an extra copy of buffer before
	//						writing/reading data from USB hardware FIFO. Hence
	//						the parameter is pointer to a pointer containing
	//						address buffer (uint8_t** dst). So that the user
	//						application can update the buffer pointer instead
	//						of copying data to address pointed by the parameter.
	//						note The updated buffer address should be accessible
	//						by USB DMA master. If user doesn't want to use
	//						zero-copy model, then the user should copy data to
	//						the address pointed by the passed buffer pointer
	//						parameter and shouldn't change the address value.
	//						See Zero-Copy Data Transfer model for more details
	//						on zero-copy concept.
	//		[in]	 length	Number of bytes to be read.
	// Returns:
	//		Nothing.
	void (*MSC_Read )(uint32_t offset, uint8_t **dst, uint32_t length, uint32_t high_offset);

	// MSC Verify callback function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a verify command. The callback function should
	// compare the buffer with the destination memory at the requested offset and
	//
	// Parameters:
	//		[in]	offset	Destination start address.
	//		[in]	buf	Buffer containing the data sent by the host.
	//		[in]	length	Number of bytes to verify.
	// Returns:
	//		LPC_OK	If data in the buffer matches the data at destination
	//		ERR_FAILED	At least one byte is different.
	uint32_t (*MSC_Verify)(uint32_t offset, uint8_t buf[], uint32_t length, uint32_t high_offset);

	// Optional callback function to optimize MSC_Write buffer transfer.
	//
	// This function is provided by the application software. This function gets
	// called when host sends SCSI_WRITE10/SCSI_WRITE12 command. The callback
	// function should update the buff_adr pointer so that the stack transfers
	// the data directly to the target buffer. /note The updated buffer address
	// should be accessible by USB DMA master. If user doesn't want to use
	// zero-copy model, then the user should not update the buffer pointer. See
	// Zero-Copy Data Transfer model for more details on zero-copy concept.
	//
	// Parameters:
	//		[in]	offset	Destination start address.
	//		[in,out]	buf	Buffer containing the data sent by the host.
	//		[in]	length	Number of bytes to write.
	// Returns:
	//		Nothing.
	void (*MSC_GetWriteBuf)(uint32_t offset, uint8_t **buff_adr, uint32_t length);

	// Optional user override-able function to replace the default MSC class
	// handler.
	//
	// The application software could override the default EP0 class handler
	// with their own by providing the handler function address as this data
	// member of the parameter structure. Application which like the default
	// handler should set this data member to zero before calling the
	// USBD_MSC_API::Init().
	// Note:
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	// Return values:
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*MSC_Ep0_Hdlr)(USBD_HANDLE_T hUsb, void *data, uint32_t event);
} USBD_MSC_INIT_PARAM_T;

typedef struct {
	// Function to determine the memory required by the MSC function driver module.
	//
	// This function is called by application layer before calling pUsbApi->msc->Init(),
	// to allocate memory used by MSC function driver module. The application should
	// allocate the memory which is accessible by USB controller/DMA controller.
	//
	// Note:
	//	Some memory areas are not accessible by all bus masters.
	// Parameters:
	//		[in]	param	Structure containing MSC function driver module
	//						initialization parameters.
	// Returns:
	//		Returns the required memory size in bytes.
	uint32_t (*GetMemSize )(USBD_MSC_INIT_PARAM_T *param);

	// Function to initialize MSC function driver module.
	//
	// This function is called by application layer to initialize MSC function
	// driver module.
	//
	// Parameters:
	//		[in]		hUsb	Handle to the USB device stack.
	//		[in,out]	param	Structure containing MSC function driver module
	//							initialization parameters.
	// Returns:
	//		Returns ErrorCode_t type to indicate success or error condition.
	//		Return values:
	//		LPC_OK	On success
	//		ERR_USBD_BAD_MEM_BUF	Memory buffer passed is not 4-byte aligned or
	//								smaller than required.
	//		ERR_API_INVALID_PARAM2	Either MSC_Write() or MSC_Read() or MSC_Verify()
	//								callbacks are not defined.
	//		ERR_USBD_BAD_INTF_DESC	Wrong interface descriptor is passed.
	//		ERR_USBD_BAD_EP_DESC	Wrong endpoint descriptor is passed.
	uint32_t (*init )(USBD_HANDLE_T hUsb, USBD_MSC_INIT_PARAM_T *param, void *);
} USBD_MSC_API_T;

//
// Undocumented structure require by API
//
typedef struct _MSC_CTRL_T
{
	/* If it's a USB HS, the max packet is 512, if it's USB FS,
	the max packet is 64. Use 512 for both HS and FS. */
	uint8_t  BulkBuf[USB_MAX_BULK_PACKET]; /* Bulk In/Out Buffer */
	USB_MSC_CBW CBW;                   /* Command Block Wrapper */
	USB_MSC_CSW CSW;                   /* Command Status Wrapper */

	USB_CORE_CTRL_T*  pUsbCtrl;

	uint64_t Offset;                  /* R/W Offset */
	uint32_t Length;                  /* R/W Length */
	uint32_t BulkLen;                 /* Bulk In/Out Length */
	uint8_t* rx_buf;

	uint8_t BulkStage;               /* Bulk Stage */
	uint8_t if_num;                  /* interface number */
	uint8_t epin_num;                /* BULK IN endpoint number */
	uint8_t epout_num;               /* BULK OUT endpoint number */
	uint32_t MemOK;                  /* Memory OK */

	uint8_t*  InquiryStr;
	uint32_t  BlockCount;
	uint32_t  BlockSize;
	uint64_t  MemorySize;
	/* user defined functions */
	void (*MSC_Write)( uint32_t offset, uint8_t** src, uint32_t length, uint32_t high_offset);
	void (*MSC_Read)( uint32_t offset, uint8_t** dst, uint32_t length, uint32_t high_offset);
	uint32_t (*MSC_Verify)( uint32_t offset, uint8_t src[], uint32_t length, uint32_t high_offset);
	/* optional call back for MSC_Write optimization */
	void (*MSC_GetWriteBuf)( uint32_t offset, uint8_t** buff_adr, uint32_t length, uint32_t high_offset);
} USB_MSC_CTRL_T;

#pragma pack(pop)

#endif // __LPC11UXX_USBMSC_H__ 
