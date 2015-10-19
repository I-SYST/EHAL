/*
 * lpc11Uxx_usbhid.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPC11UXX_USBHID_H__
#define __LPC11UXX_USBHID_H__

#include "usb_hiddef.h"
#include "lpcusbd_core.h"

#pragma pack(push, 1)

typedef struct {
	uint16_t 	len;		// Size of the report descriptor in bytes.
	uint8_t 	idle_time;	// This value is used by stack to respond to Set_Idle &
							// GET_Idle requests for the specified report ID. The
							// value of this field specified the rate at which
							// duplicate reports are generated for the specified
							// Report ID. For example, a device with two input
							// reports could specify an idle rate of 20 milliseconds
							// for report ID 1 and 500 milliseconds for report ID 2.
	uint8_t 	__pad;
	uint8_t 	*desc;		// Report descriptor.
} USB_HID_REPORT_T;

typedef struct {
	uint32_t 	mem_base;		// Base memory location from where the stack can
								// allocate data and buffers.
								// Note: The memory address set in this field
								//       should be accessible by USB DMA controller.
								//       Also this value should be aligned on 4 byte boundary.
	uint32_t 	mem_size;		// The size of memory buffer which stack can use.
	uint8_t 	max_reports;	// Number of HID reports supported by this instance of
								// HID class driver.
	uint8_t 	pad [3];
	uint8_t * 	intf_desc;		// Pointer to the HID interface descriptor within the
								// descriptor array (high_speed_desc) passed to Init()
								// through USB_CORE_DESCS_T structure.
	USB_HID_REPORT_T * 	report_data;	// Pointer to an array of HID report descriptor
										// data structure (USB_HID_REPORT_T). The number
										// of elements in the array should be same a max_
										// reports value. The stack uses this array to
										// respond to requests received for various HID
										// report descriptor information.
										// Note: This array should be of global scope.

	// HID get report callback function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a HID_REQUEST_GET_REPORT request. The setup packet
	// data (pSetup) is passed to the callback so that application can extract
	// the report ID, report type and other information need to generate the report.
	//
	// Note:
	//		HID reports are sent via interrupt IN endpoint also. This function
	//		is called only when report request is received on control endpoint.
	//		Application should implement HID_EpIn_Hdlr to send reports to host
	//		via interrupt IN endpoint.
	//
	// Parameters:
	//		[in]	hHid	Handle to HID function driver.
	//		[in]	pSetup	Pointer to setup packet received from host.
	//		[in,out]	pBuffer	Pointer to a pointer of data buffer containing report data. Pointer-to-pointer is used to implement zero-copy buffers. See Zero-Copy Data Transfer model for more details on zero-copy concept.
	//		[in]	length	Amount of data copied to destination buffer.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_GetReport)(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t **pBuffer, uint16_t *length);

	// HID set report callback function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a HID_REQUEST_SET_REPORT request. The setup packet
	// data (pSetup) is passed to the callback so that application can extract
	// the report ID, report type and other information need to modify the report.
	// An application might choose to ignore input Set_Report requests as meaningless.
	// Alternatively these reports could be used to reset the origin of a control
	// (that is, current position should report zero).
	//
	// Parameters:
	//		[in]	hHid	Handle to HID function driver.
	//		[in]	pSetup	Pointer to setup packet received from host.
	//		[in,out]	pBuffer	Pointer to a pointer of data buffer containing report data. Pointer-to-pointer is used to implement zero-copy buffers. See Zero-Copy Data Transfer model for more details on zero-copy concept.
	//		[in]	length	Amount of data copied to destination buffer.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_SetReport)(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t **pBuffer, uint16_t length);

	// Optional callback function to handle HID_GetPhysDesc request.
	//
	// The application software could provide this callback HID_GetPhysDesc handler
	// to handle get physical descriptor requests sent by the host. When host
	// requests Physical Descriptor set 0, application should return a special
	// descriptor identifying the number of descriptor sets and their sizes.
	// A Get_Descriptor request with the Physical Index equal to 1 should
	// return the first Physical Descriptor set. A device could possibly have
	// alternate uses for its items. These can be enumerated by issuing subsequent
	// Get_Descriptor requests while incrementing the Descriptor Index. A device
	// should return the last descriptor set to requests with an index greater
	// than the last number defined in the HID descriptor.
	//
	// Note:
	//		Applications which don't have physical descriptor should set this
	//		data member to zero before calling the USBD_HID_API::Init().
	// Parameters:
	//		[in]	hHid	Handle to HID function driver.
	//		[in]	pSetup	Pointer to setup packet received from host.
	//		[in]	pBuf	Pointer to a pointer of data buffer containing physical descriptor data. If the physical descriptor is in USB accessible memory area application could just update the pointer or else it should copy the descriptor to the address pointed by this pointer.
	//		[in]	length	Amount of data copied to destination buffer or descriptor length.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next
	//							in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_GetPhysDesc)(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t **pBuf, uint16_t *length);

	// Optional callback function to handle HID_REQUEST_SET_IDLE request.
	//
	// The application software could provide this callback to handle
	// HID_REQUEST_SET_IDLE requests sent by the host. This callback is
	// provided to applications to adjust timers associated with various
	// reports, which are sent to host over interrupt endpoint. The setup
	// packet data (pSetup) is passed to the callback so that application
	// can extract the report ID, report type and other information need to
	// modify the report's idle time.
	//
	// Note:
	//		Applications which don't send reports on Interrupt endpoint or don't
	//		have idle time between reports should set this data member to zero
	//		before calling the USBD_HID_API::Init().
	// Parameters:
	//		[in]	hHid	Handle to HID function driver.
	//		[in]	pSetup	Pointer to setup packet received from host.
	//		[in]	idleTime	Idle time to be set for the specified report.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to
	//							next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_SetIdle)(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t idleTime);

	// Optional callback function to handle HID_REQUEST_SET_PROTOCOL request.
	//
	// The application software could provide this callback to handle
	// HID_REQUEST_SET_PROTOCOL requests sent by the host. This callback is
	// provided to applications to adjust modes of their code between boot
	// mode and report mode.
	//
	// Note:
	//		Applications which don't support protocol modes should set this data
	//		member to zero before calling the USBD_HID_API::Init().
	// Parameters:
	//		[in]	hHid	Handle to HID function driver.
	//		[in]	pSetup	Pointer to setup packet received from host.
	//		[in]	protocol	Protocol mode. 0 = Boot Protocol
	//										   1 = Report Protocol
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next
	//							in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_SetProtocol)(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t protocol);

	// Optional Interrupt IN endpoint event handler.
	//
	// The application software could provide Interrupt IN endpoint event handler.
	// Application which send reports to host on interrupt endpoint should provide
	// an endpoint event handler through this data member. This data member is
	// ignored if the interface descriptor intf_desc doesn't have any IN interrupt
	// endpoint descriptor associated.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Handle to HID function driver.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should return ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next
	//							in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_EpIn_Hdlr)(USBD_HANDLE_T hUsb, void *data, uint32_t event);

	// Optional Interrupt OUT endpoint event handler.
	//
	// The application software could provide Interrupt OUT endpoint event handler.
	// Application which receives reports from host on interrupt endpoint should
	// provide an endpoint event handler through this data member. This data member
	// is ignored if the interface descriptor intf_desc doesn't have any OUT
	// interrupt endpoint descriptor associated.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Handle to HID function driver.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should return ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next
	//							in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_EpOut_Hdlr)(USBD_HANDLE_T hUsb, void *data, uint32_t event);

	// Optional user override-able function to replace the default HID_GetReportDesc handler.
	//
	// The application software could override the default HID_GetReportDesc handler
	// with their own by providing the handler function address as this data member
	// of the parameter structure. Application which like the default handler should
	// set this data member to zero before calling the USBD_HID_API::Init() and also
	// provide report data array report_data field.
	// Note:
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or error
	//		condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*HID_GetReportDesc)(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t **pBuf, uint16_t *length);

	// Optional user override-able function to replace the default HID class handler.
	//
	// The application software could override the default EP0 class handler with
	// their own by providing the handler function address as this data member of
	// the parameter structure. Application which like the default handler should
	// set this data member to zero before calling the USBD_HID_API::Init().
	// Note:
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or error
	//		condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next
	//							in line.
	//		ERR_USBD_xxx	For other error conditions.

	uint32_t (*HID_Ep0_Hdlr)(USBD_HANDLE_T hUsb, void *data, uint32_t event);
} USBD_HID_INIT_PARAM_T;

typedef struct {
	// Function to determine the memory required by the HID function driver module.
	//
	// This function is called by application layer before calling pUsbApi->hid->Init(),
	// to allocate memory used by HID function driver module. The application should
	// allocate the memory which is accessible by USB controller/DMA controller.
	//
	// Note:
	//		Some memory areas are not accessible by all bus masters.
	// Parameters:
	//		[in]	param	Structure containing HID function driver module
	//						initialization parameters.
	// Returns:
	//		Returns the required memory size in bytes.
	uint32_t (*GetMemSize )(USBD_HID_INIT_PARAM_T *param);

	// Function to initialize HID function driver module.
	//
	// This function is called by application layer to initialize HID function
	// driver module. On successful initialization the function returns a handle
	// to HID function driver module in passed param structure.
	//
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in,out]	param	Structure containing HID function driver module
	//							initialization parameters.
	// Returns:
	//		Returns ErrorCode_t type to indicate success or error condition.
	//		LPC_OK	On success
	//		ERR_USBD_BAD_MEM_BUF	Memory buffer passed is not 4-byte aligned or
	//								smaller than required.
	//		ERR_API_INVALID_PARAM2	Either HID_GetReport() or HID_SetReport()
	//								callback are not defined.
	//		ERR_USBD_BAD_DESC		HID_HID_DESCRIPTOR_TYPE is not defined immediately
	//								after interface descriptor.
	//		ERR_USBD_BAD_INTF_DESC	Wrong interface descriptor is passed.
	//		ERR_USBD_BAD_EP_DESC	Wrong endpoint descriptor is passed.
	uint32_t (*init )(USBD_HANDLE_T hUsb, USBD_HID_INIT_PARAM_T *param);
} USBD_HID_API_T;

//
// Undocumented structure but require by API
//
typedef struct _HID_CTRL_T {
  /* pointer to controller */
  USB_CORE_CTRL_T*  pUsbCtrl;
  /* descriptor pointers */
  uint8_t* hid_desc;
  USB_HID_REPORT_T* report_data;

  uint8_t protocol;
  uint8_t if_num;                  /* interface number */
  uint8_t epin_adr;                /* IN interrupt endpoint */
  uint8_t epout_adr;               /* OUT interrupt endpoint */

  /* user defined functions */
  uint32_t (*HID_GetReport)( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t* length);
  uint32_t (*HID_SetReport)( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t length);
  uint32_t (*HID_GetPhysDesc)( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuf, uint16_t* length);
  uint32_t (*HID_SetIdle)( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t idleTime);
  uint32_t (*HID_SetProtocol)( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t protocol);

  /* virtual overridable functions */
  uint32_t (*HID_GetReportDesc)(USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuf, uint16_t* length);

} USB_HID_CTRL_T;

#pragma pack(pop)

#endif // __LPC11UXX_USBHID_H__ 
