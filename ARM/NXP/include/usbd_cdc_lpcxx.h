/*
 * lpc11Uxx_usbcdc.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPC11UXX_USBCDC_H__
#define __LPC11UXX_USBCDC_H__

#include "usb/usb_cdcdef.h"
#include "usbd_core_lpcxx.h"

#pragma pack(push, 1)

typedef struct {
	uint32_t 	dwDTERate;
	uint8_t 	bCharFormat;
	uint8_t 	bParityType;
	uint8_t 	bDataBits;
} CDC_LINE_CODING;
#pragma pack(pop)


typedef struct {
	uint32_t 	mem_base;		// Base memory location from where the stack can
								// allocate data and buffers.
								// Note: The memory address set in this field
								// should be accessible by USB DMA controller.
								// Also this value should be aligned on 4 byte boundary.
	uint32_t 	mem_size;		// The size of memory buffer which stack can use.
	uint8_t * 	cif_intf_desc;	// Pointer to the control interface descriptor within
								// the descriptor array (high_speed_desc) passed to
								// Init() through USB_CORE_DESCS_T structure. The
								// stack assumes both HS and FS use same BULK endpoints.
	uint8_t * 	dif_intf_desc;	// Pointer to the data interface descriptor within
								// the descriptor array (high_speed_desc) passed to
								// Init() through USB_CORE_DESCS_T structure. The stack
								// assumes both HS and FS use same BULK endpoints.

	// Communication Interface Class specific get request call-back function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends CIC management element get requests.
	//
	// Note:
	//		Applications implementing Abstract Control Model subclass can set this
	//		param to NULL. As the default driver parses ACM requests and calls the
	//		individual ACM call-back routines defined in this structure. For all
	//		other subclasses this routine should be provided by the application.
	//		The setup packet data (pSetup) is passed to the call-back so that
	//		application can extract the CIC request type and other associated data.
	//		By default the stack will assign pBuffer pointer to EP0Buff allocated
	//		at init. The application code can directly write data into this buffer
	//		as long as data is less than 64 byte. If more data has to be sent then
	//		application code should update pBuffer pointer and length accordingly.
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	pSetup	Pointer to setup packet received from host.
	//		[in,out]	pBuffer	Pointer to a pointer of data buffer containing request
	//							data. Pointer-to-pointer is used to implement zero-copy
	//							buffers. See Zero-Copy Data Transfer model for more
	//							details on zero-copy concept.
	//		[in,out]	length	Amount of data to be sent back to host.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next
	//							in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*CIC_GetRequest )(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t **pBuffer, uint16_t *length);

	// Communication Interface Class specific set request call-back function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a CIC management element requests.
	//
	// Note:
	//		Applications implementing Abstract Control Model subclass can set this
	//		param to NULL. As the default driver parses ACM requests and calls the
	//		individual ACM call-back routines defined in this structure. For all
	//		other subclasses this routine should be provided by the application.
	//		The setup packet data (pSetup) is passed to the call-back so that
	//		application can extract the CIC request type and other associated
	//		data. If a set request has data associated, then this call-back is
	//		called twice.
	//		First when setup request is received, at this time application code
	//		could update pBuffer pointer to point to the intended destination.
	//		The length param is set to 0 so that application code knows this is
	//		first time. By default the stack will assign pBuffer pointer to
	//		EP0Buff allocated at init. Note, if data length is greater than 64
	//		bytes and application code doesn't update pBuffer pointer the stack
	//		will send STALL condition to host.
	//		Second when the data is received from the host. This time the length
	//		param is set with number of data bytes received.
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	pSetup	Pointer to setup packet received from host.
	//		[in,out]	pBuffer	Pointer to a pointer of data buffer containing
	//							request data. Pointer-to-pointer is used to
	//							implement zero-copy buffers. See Zero-Copy Data
	//							Transfer model for more details on zero-copy concept.
	//		[in]	length	Amount of data copied to destination buffer.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	// 		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next
	//							in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*CIC_SetRequest)(USBD_HANDLE_T hCdc, USB_SETUP_PACKET *pSetup, uint8_t **pBuffer, uint16_t length);

	// Communication Device Class specific BULK IN endpoint handler.
	//
	// The application software should provide the BULK IN endpoint handler.
	// Applications should transfer data depending on the communication protocol
	// type set in descriptors.
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
	uint32_t (*CDC_BulkIN_Hdlr)(USBD_HANDLE_T hUsb, void *data, uint32_t event);

	// Communication Device Class specific BULK OUT endpoint handler.
	//
	// The application software should provide the BULK OUT endpoint handler.
	// Applications should transfer data depending on the communication protocol
	// type set in descriptors.
	// Note:
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*CDC_BulkOUT_Hdlr )(USBD_HANDLE_T hUsb, void *data, uint32_t event);

	// Abstract control model(ACM) subclass specific SEND_ENCAPSULATED_COMMAND
	// request call-back function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a SEND_ENCAPSULATED_COMMAND set request.
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	buffer	Pointer to the command buffer.
	//		[in]	len	Length of the command buffer.
	//	Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*SendEncpsCmd)(USBD_HANDLE_T hCDC, uint8_t *buffer, uint16_t len);

	// Abstract control model(ACM) subclass specific GET_ENCAPSULATED_RESPONSE
	// request call-back function.
	//
	// This function is provided by the application software. This function
	// gets called when host sends a GET_ENCAPSULATED_RESPONSE request.
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in,out]	buffer	Pointer to a pointer of data buffer containing
	//							response data. Pointer-to-pointer is used to
	//							implement zero-copy buffers. See Zero-Copy Data
	//							Transfer model for more details on zero-copy concept.
	//		[in,out]	len	Amount of data to be sent back to host.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*GetEncpsResp )(USBD_HANDLE_T hCDC, uint8_t **buffer, uint16_t *len);

	// Abstract control model(ACM) subclass specific SET_COMM_FEATURE request
	// call-back function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a SET_COMM_FEATURE set request.
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	feature	Communication feature type. See usbcdc11.pdf, section
	//						6.2.4, Table 47.
	//		[in]	buffer	Pointer to the settings buffer for the specified
	//						communication feature.
	//		[in]	len	Length of the request buffer.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*SetCommFeature)(USBD_HANDLE_T hCDC, uint16_t feature, uint8_t *buffer, uint16_t len);

	// Abstract control model(ACM) subclass specific GET_COMM_FEATURE request
	// call-back function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a GET_ENCAPSULATED_RESPONSE request.
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	feature	Communication feature type. See usbcdc11.pdf,
	//						section 6.2.4, Table 47.
	//		[in,out]	buffer	Pointer to a pointer of data buffer containing
	//							current settings for the communication feature.
	//							Pointer-to-pointer is used to implement zero-copy
	//							buffers.
	//		[in,out]	len	Amount of data to be sent back to host.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*GetCommFeature)(USBD_HANDLE_T hCDC, uint16_t feature, uint8_t **pBuffer, uint16_t *len);

	// Abstract control model(ACM) subclass specific CLEAR_COMM_FEATURE request
	// call-back function.
	//
	// This function is provided by the application software. This function gets
	// called when host sends a CLEAR_COMM_FEATURE request. In the call-back the
	// application should Clears the settings for a particular communication feature.
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	feature	Communication feature type. See usbcdc11.pdf, section
	//						6.2.4, Table 47.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success
	//		or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*ClrCommFeature )(USBD_HANDLE_T hCDC, uint16_t feature);

	// Abstract control model(ACM) subclass specific SET_CONTROL_LINE_STATE
	// request call-back function.
	//
	// This function is provided by the application software. This function
	// gets called when host sends a SET_CONTROL_LINE_STATE request. RS-232
	// signal used to tell the DCE device the DTE device is now present
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	state	The state value uses bitmap values defined in
	//						usbcdc11.pdf, section 6.2.14, Table 51.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success
	//		or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*SetCtrlLineState )(USBD_HANDLE_T hCDC, uint16_t state);

	// Abstract control model(ACM) subclass specific SEND_BREAK request
	// call-back function.
	//
	// This function is provided by the application software. This function
	// gets called when host sends a SEND_BREAK request.
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	mstime	Duration of Break signal in milliseconds. If mstime
	//						is FFFFh, then the application should send break
	//						until another SendBreak request is received with the
	//						wValue of 0000h.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success or
	//		error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*SendBreak )(USBD_HANDLE_T hCDC, uint16_t mstime);

	// Abstract control model(ACM) subclass specific SET_LINE_CODING request
	// call-back function.
	//
	// This function is provided by the application software. This function
	// gets called when host sends a SET_LINE_CODING request. The application
	// should configure the device per DTE rate, stop-bits, parity, and
	// number-of-character bits settings provided in command buffer. See
	// usbcdc11.pdf, section 6.2.13, table 50 for detail of the command buffer.
	//
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	line_coding	Pointer to the CDC_LINE_CODING command buffer.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success
	//		or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*SetLineCode )(USBD_HANDLE_T hCDC, CDC_LINE_CODING *line_coding);

	// Optional Communication Device Class specific INTERRUPT IN endpoint handler.
	//
	// The application software should provide the INT IN endpoint handler.
	// Applications should transfer data depending on the communication protocol
	// type set in descriptors.
	// Note:
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success
	//		or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*CDC_InterruptEP_Hdlr )(USBD_HANDLE_T hUsb, void *data, uint32_t event);

	// Optional user override-able function to replace the default CDC class handler.
	//
	// The application software could override the default EP0 class handler with
	// their own by providing the handler function address as this data member of
	// the parameter structure. Application which like the default handler should
	// set this data member to zero before calling the USBD_CDC_API::Init().
	// Note:
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
	// Returns:
	//		The call back should returns ErrorCode_t type to indicate success
	//		or error condition.
	//		LPC_OK	On success.
	//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
	//		ERR_USBD_xxx	For other error conditions.
	uint32_t (*CDC_Ep0_Hdlr)(USBD_HANDLE_T hUsb, void *data, uint32_t event);
} USBD_CDC_INIT_PARAM_T;

typedef struct {
	// Function to determine the memory required by the CDC function driver module.
	//
	// This function is called by application layer before calling pUsbApi->CDC->Init(),
	// to allocate memory used by CDC function driver module. The application should
	// allocate the memory which is accessible by USB controller/DMA controller.
	//
	// Note:
	// 		Some memory areas are not accessible by all bus masters.
	//	Parameters:
	//		[in]	param	Structure containing CDC function driver module
	//						initialization parameters.
	// Returns:
	//		Returns the required memory size in bytes.
	uint32_t(* 	GetMemSize )(USBD_CDC_INIT_PARAM_T *param);

	// Function to initialize CDC function driver module.
	//
	// This function is called by application layer to initialize CDC function
	// driver module.
	//
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in,out]	param	Structure containing CDC function driver module
	//							initialization parameters.
	// Returns:
	//		Returns ErrorCode_t type to indicate success or error condition.
	//		LPC_OK	On success
	//		ERR_USBD_BAD_MEM_BUF	Memory buffer passed is not 4-byte aligned or
	//								smaller than required.
	//		ERR_API_INVALID_PARAM2	Either CDC_Write() or CDC_Read() or CDC_Verify()
	//								callbacks are not defined.
	//		ERR_USBD_BAD_INTF_DESC	Wrong interface descriptor is passed.
	//		ERR_USBD_BAD_EP_DESC	Wrong endpoint descriptor is passed.
	uint32_t (*init )(USBD_HANDLE_T hUsb, USBD_CDC_INIT_PARAM_T *param, USBD_HANDLE_T *phCDC);

	// Function to send CDC class notifications to host.
	//
	// This function is called by application layer to send CDC class notifications
	// to host. See usbcdc11.pdf, section 6.3, Table 67 for various notification
	// types the CDC device can send.
	//
	// Note:
	//		The current version of the driver only supports following notifications
	//		allowed by ACM subclass: CDC_NOTIFICATION_NETWORK_CONNECTION,
	//		CDC_RESPONSE_AVAILABLE, CDC_NOTIFICATION_SERIAL_STATE.
	//		For all other notifications application should construct the notification
	//		buffer appropriately and call hw->USB_WriteEP() for interrupt endpoint
	//		associated with the interface.
	// Parameters:
	//		[in]	hCdc	Handle to CDC function driver.
	//		[in]	bNotification	Notification type allowed by ACM subclass.
	//								Should be CDC_NOTIFICATION_NETWORK_CONNECTION,
	//								CDC_RESPONSE_AVAILABLE or CDC_NOTIFICATION_SERIAL_STATE.
	//								For all other types ERR_API_INVALID_PARAM2 is returned.
	//								See usbcdc11.pdf, section 3.6.2.1, table 5.
	//		[in]	data	Data associated with notification.
	//						For CDC_NOTIFICATION_NETWORK_CONNECTION a non-zero data
	//						value is interpreted as connected state.
	//						For CDC_RESPONSE_AVAILABLE this parameter is ignored.
	//						For CDC_NOTIFICATION_SERIAL_STATE the data should use bitmap
	//						values defined in usbcdc11.pdf, section 6.3.5, Table 69.
	// Returns:
	//		Returns ErrorCode_t type to indicate success or error condition.
	//		LPC_OK	On success
	//		ERR_API_INVALID_PARAM2	If unsupported notification type is passed.
	uint32_t (*SendNotification)(USBD_HANDLE_T hCdc, uint8_t bNotification, uint16_t data);
} USBD_CDC_API_T;

//
// Undocumented structure require by API
//
typedef struct _CDC_CTRL_T
{
  USB_CORE_CTRL_T*  pUsbCtrl;
  /* notification buffer */
  uint8_t notice_buf[12];
  CDC_LINE_CODING line_coding;
  uint8_t pad0;

  uint8_t cif_num;                 /* control interface number */
  uint8_t dif_num;                 /* data interface number */
  uint8_t epin_num;                /* BULK IN endpoint number */
  uint8_t epout_num;               /* BULK OUT endpoint number */
  uint8_t epint_num;               /* Interrupt IN endpoint number */
  uint8_t pad[3];
  /* user defined functions */
  uint32_t (*SendEncpsCmd) (USBD_HANDLE_T hCDC, uint8_t* buffer, uint16_t len);
  uint32_t (*GetEncpsResp) (USBD_HANDLE_T hCDC, uint8_t** buffer, uint16_t* len);
  uint32_t (*SetCommFeature) (USBD_HANDLE_T hCDC, uint16_t feature, uint8_t* buffer, uint16_t len);
  uint32_t (*GetCommFeature) (USBD_HANDLE_T hCDC, uint16_t feature, uint8_t** pBuffer, uint16_t* len);
  uint32_t (*ClrCommFeature) (USBD_HANDLE_T hCDC, uint16_t feature);
  uint32_t (*SetCtrlLineState) (USBD_HANDLE_T hCDC, uint16_t state);
  uint32_t (*SendBreak) (USBD_HANDLE_T hCDC, uint16_t state);
  uint32_t (*SetLineCode) (USBD_HANDLE_T hCDC, CDC_LINE_CODING* line_coding);

  /* virtual functions */
  uint32_t (*CIC_GetRequest)( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t* length);
  uint32_t (*CIC_SetRequest)( USBD_HANDLE_T hCdc, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t length);

} USB_CDC_CTRL_T;


#endif // __LPC11UXX_USBCDC_H__ 
