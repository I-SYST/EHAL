/*
 * lpc11Uxx_usbdfu.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPC11UXX_USBDFU_H__
#define __LPC11UXX_USBDFU_H__

#include "usbd_core_lpcxx.h"

#pragma pack(push, 1)

typedef struct {
	uint32_t 	mem_base;		// Base memory location from where the stack can
								// allocate data and buffers.
	uint32_t 	mem_size;		// The size of memory buffer which stack can use.
	uint16_t 	wTransferSize;	// DFU transfer block size in number of bytes.
								// This value should match the value set in DFU
								// descriptor provided as part of the descriptor
								// array (high_speed_desc) passed to Init() through
								// USB_CORE_DESCS_T structure.
	uint16_t 	pad;
	uint8_t 	*intf_desc;		// Pointer to the DFU interface descriptor within
								// the descriptor array (high_speed_desc) passed
								// to Init() through USB_CORE_DESCS_T structure.

	// DFU Write callback function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a write command. For application using zero-copy
	// buffer scheme this function is called for the first time with length
	// parameter set to 0. The application code should update the buffer pointer.
	//
	// Parameters:
	//		[in]	block_num		Destination start address.
	//		[in,out] src		Pointer to a pointer to the source of data.
	//							Pointer-to-pointer is used to implement zero-copy
	//							buffers. See Zero-Copy Data Transfer model for more
	//							details on zero-copy concept.
	//		[out]	bwPollTimeout	Pointer to a 3 byte buffer which the callback implementer should fill with the amount of minimum time, in milliseconds, that the host should wait before sending a subsequent DFU_GETSTATUS request.
	//		[in]	length			Number of bytes to be written.
	// Returns:
	//		Returns DFU_STATUS_ values defined in mw_usbd_dfu.h.
	uint8_t (*DFU_Write )(uint32_t block_num, uint8_t **src, uint32_t length, uint8_t *bwPollTimeout);

	// DFU Read callback function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a read command.
	//
	// Parameters:
	//		[in]	block_num	Destination start address.
	//		[in,out] dst	Pointer to a pointer to the source of data.
	//						Pointer-to-pointer is used to implement zero-copy buffers.
	//						See Zero-Copy Data Transfer model for more details on
	//						zero-copy concept.
	//		[in]	length	Amount of data copied to destination buffer.
	// Returns:
	//		DFU_STATUS_ values defined in mw_usbd_dfu.h to return error conditions.
	//		0 if there is no more data to be read. Stack will send EOF frame and
	//		set DFU state-machine to dfuIdle state.
	//		length of the data copied, should be greater than or equal to 16. If
	//		the data copied is less than DFU wTransferSize the stack will send EOF
	//		frame and goes to dfuIdle state.
	uint32_t (*DFU_Read )(uint32_t block_num, uint8_t **dst, uint32_t length);

	// DFU done callback function.
	//
	// This function is provided by the application software. This function gets
	// called after firmware download completes.
	//
	// Returns:
	//		Nothing.
	void (*DFU_Done )(void);

	// DFU detach callback function.
	//
	// This function is provided by the application software. This function gets
	// called after USB_REQ_DFU_DETACH is received. Applications which set
	// USB_DFU_WILL_DETACH bit in DFU descriptor should define this function. As
	// part of this function application can call Connect() routine to disconnect
	// and then connect back with host. For application which rely on WinUSB based
	// host application should use this feature since USB reset can be invoked
	// only by kernel drivers on Windows host. By implementing this feature host
	// doen't have to issue reset instead the device has to do it automatically by
	// disconnect and connect procedure.
	//
	// Parameters:
	//		[in]	hUsb	Handle DFU control structure.
	// Returns:
	//		Nothing.
	void (*DFU_Detach)(USBD_HANDLE_T hUsb);

	// Optional user override-able function to replace the default DFU class handler.
	//
	// The application software could override the default EP0 class handler with
	// their own by providing the handler function address as this data member of
	// the parameter structure. Application which like the default handler should
	// set this data member to zero before calling the USBD_DFU_API::Init().
	// Note:
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	// 		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*DFU_Ep0_Hdlr)(USBD_HANDLE_T hUsb, void *data, uint32_t event);
} USBD_DFU_INIT_PARAM_T;

typedef struct {

	// Function to determine the memory required by the DFU function driver module.
	//
	// This function is called by application layer before calling pUsbApi->dfu->Init(),
	// to allocate memory used by DFU function driver module. The application should
	// allocate the memory which is accessible by USB controller/DMA controller.
	//
	// Note:
	//		Some memory areas are not accessible by all bus masters.
	// Parameters:
	//		[in]	param	Structure containing DFU function driver module initialization parameters.
	// Returns:
	//		Returns the required memory size in bytes.
	uint32_t(* 	GetMemSize )(USBD_DFU_INIT_PARAM_T *param);

	// Function to initialize DFU function driver module.
	//
	// This function is called by application layer to initialize DFU function
	// driver module.
	//
	// Parameters:
	//		[in]	 hUsb	Handle to the USB device stack.
	//		[in,out] param	Structure containing DFU function driver module
	//						initialization parameters.
	// Returns:
	//		LPC_OK	On success
	//		ERR_USBD_BAD_MEM_BUF	Memory buffer passed is not 4-byte aligned or
	//								smaller than required.
	//		ERR_API_INVALID_PARAM2	Either DFU_Write() or DFU_Done() or DFU_Read()
	//								call-backs are not defined.
	//		ERR_USBD_BAD_DESC
	//		USB_DFU_DESCRIPTOR_TYPE is not defined immediately after interface descriptor.
	//								wTransferSize in descriptor doesn't match the value
	//								passed in param->wTransferSize.
	//								DFU_Detach() is not defined while USB_DFU_WILL_DETACH
	//								is set in DFU descriptor.
	//		ERR_USBD_BAD_INTF_DESC	Wrong interface descriptor is passed.
	uint32_t (*init)(USBD_HANDLE_T hUsb, USBD_DFU_INIT_PARAM_T *param, uint32_t init_state);
} USBD_DFU_API_T;

#pragma pack(pop)

#endif // __LPC11UXX_USBDFU_H__ 
