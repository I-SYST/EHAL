/*
 * lpcusbd_core.h
 *
 * USBD stack Core API functions structure.

This module exposes functions which interact directly with USB device stack's core layer.
The application layer uses this component when it has to implement custom class function
driver or standard class function driver which is not part of the current USB device stack.
The functions exposed by this interface are to register class specific EP0 handlers and
corresponding utility functions to manipulate EP0 state machine of the stack. This
interface also exposes function to register custom endpoint interrupt handler.
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPCUSBD_CORE_H__
#define __LPCUSBD_CORE_H__

#include "usb_def.h"

#define USB_FS_MAX_BULK_PACKET		64
#define USB_HS_MAX_BULK_PACKET		512

#define USB_MAX_BULK_PACKET 		USB_FS_MAX_BULK_PACKET

typedef void* USBD_HANDLE_T;

// USBD setup request and endpoint event handler type.
//
// The application layer should define the custom class's EP0 handler with
// function signature. The stack calls all the registered class handlers on
// any EP0 event before going through default handling of the event. This
// gives the class handlers to implement class specific request handlers and
// also to override the default stack handling for a particular event targeted
// to the interface. If an event is not handled by the callback the function
// should return ERR_USBD_UNHANDLED. For all other return codes the stack
// assumes that callback has taken care of the event and hence will not process
// the event any further and issues a STALL condition on EP0 indicating error
// to the host.
// For endpoint interrupt handler the return value is ignored by the stack.
// Parameters:
//		[in]	hUsb	Handle to the USB device stack.
//		[in]	data	Pointer to the data which will be passed when callback function is called by the stack.
//		[in]	event	Type of endpoint event. See USBD_EVENT_T for more details.
// Returns:
//		The call back should returns ErrorCode_t type to indicate success or error condition.
//		Return values:
//		LPC_OK	On success.
//		ERR_USBD_UNHANDLED	Event is not handled hence pass the event to next in line.
//		ERR_USBD_xxx	For other error conditions.
typedef uint32_t (*USB_EP_HANDLER_T)(USBD_HANDLE_T hUsb, void *data, uint32_t event);

#pragma pack(push, 1)

typedef struct {
	uint8_t bmRequestType;			// Characteristics of request:
									// D7: Data transfer direction
									//		0 = Host-to-device
									//		1 = Device-to-host
									// D6...5: Type
									//		0 = Standard
									//		1 = Class
									//		2 = Vendor
									//		3 = Reserved
									// D4...0: Recipient
									// 		0 = Device
									//		1 = Interface
									//		2 = Endpoint
									//		3 = Other
									//		4...31 = Reserved
	uint8_t bRequest;				// Specific request (refer to Table 9-3)
	uint16_t wValue;				// Word-sized field that varies according to request
	uint16_t wIndex;				// Index or Offset
									// Word-sized field that varies according to request;
									// typically used to pass an index or offset
	uint16_t wLength;				// Number of bytes to transfer if there is a Data stage
} USB_SETUP_PACKET;

typedef struct {
	// Function to register class specific EP0 event handler with USB device stack.
	//
	// The application layer uses this function when it has to register the custom
	// class's EP0 handler. The stack calls all the registered class handlers on any
	// EP0 event before going through default handling of the event. This gives the
	// class handlers to implement class specific request handlers and also to
	// override the default stack handling for a particular event targeted to the
	// interface. Check USB_EP_HANDLER_T for more details on how the callback
	// function should be implemented. Also application layer could use this
	// function to register EP0 handler which responds to vendor specific requests.
	//
	//Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	//		[in]	pfn		Class specific EP0 handler function.
	//		[in]	data	Pointer to the data which will be passed when callback
	//						function is called by the stack.
	// Returns:
	//		Returns ErrorCode_t type to indicate success or error condition.
	// Return values:
	//		LPC_OK	On success
	//		ERR_USBD_TOO_MANY_CLASS_HDLR(0x0004000c)	The number of class handlers registered is greater than the number of handlers allowed by the stack.
	uint32_t (*RegisterClassHandler )(USBD_HANDLE_T hUsb, USB_EP_HANDLER_T pfn, void *data);

	// Function to register interrupt/event handler for the requested endpoint
	// with USB device stack.
	//
	// The application layer uses this function to register the endpoint event handler.
	// The stack calls all the registered endpoint handlers when
	//
	// USB_EVT_OUT or USB_EVT_OUT_NAK events happen for OUT endpoint.
	// USB_EVT_IN or USB_EVT_IN_NAK events happen for IN endpoint. Check USB_EP_HANDLER_T
	// for more details on how the callback function should be implemented.
	// Note:
	// By default endpoint _NAK events are not enabled. Application should call
	// USBD_HW_API_T::EnableEvent for the corresponding endpoint.
	// Parameters:
	//		[in]	hUsb		Handle to the USB device stack.
	//		[in]	ep_index	Endpoint index. Computed as
	// 							For OUT endpoints = 2 * endpoint number eg. for EP2_OUT it is 4.
	// 							For IN endopoints = (2 * endpoint number) + 1 eg. for EP2_IN it is 5.
	//		[in]	pfn			Endpoint event handler function.
	//		[in]	data		Pointer to the data which will be passed when callback function
	//							is called by the stack.
	// Returns:
	//		Returns ErrorCode_t type to indicate success or error condition.
	//		Return values:
	//		LPC_OK	On success
	//		ERR_API_INVALID_PARAM2	ep_index is outside the boundary
	//		( < 2 * USBD_API_INIT_PARAM_T::max_num_ep).
	uint32_t (*RegisterEpHandler )(USBD_HANDLE_T hUsb, uint32_t ep_index,
								   USB_EP_HANDLER_T pfn, void *data);

	// Function to set EP0 state machine in setup state.
	//
	// This function is called by USB stack and the application layer to set the
	// EP0 state machine in setup state. This function will read the setup packet
	// received from USB host into stack's buffer.
	// Note:
	// This interface is provided to users to invoke this function in other scenarios which are not handle by current stack. In most user applications this function is not called directly.Also this function can be used by users who are selectively modifying the USB device stack's standard handlers through callback interface exposed by the stack.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	// Returns:
	//		Nothing.
	void (*SetupStage )(USBD_HANDLE_T hUsb);

	// Function to set EP0 state machine in data_in state.
	//
	// This function is called by USB stack and the application layer to set the
	// EP0 state machine in data_in state. This function will write the data
	// present in EP0Data buffer to EP0 FIFO for transmission to host.
	// Note:
	// 	This interface is provided to users to invoke this function in other
	//	scenarios which are not handle by current stack. In most user applications
	//	this function is not called directly.Also this function can be used by users
	//	who are selectively modifying the USB device stack's standard handlers
	//	through callback interface exposed by the stack.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	// Returns:
	//		Nothing.
	void (*DataInStage )(USBD_HANDLE_T hUsb);

	// Function to set EP0 state machine in data_out state.
	//
	// This function is called by USB stack and the application layer to set the
	// EP0 state machine in data_out state. This function will read the control
	// data (EP0 out packets) received from USB host into EP0Data buffer.
	// Note:
	//	This interface is provided to users to invoke this function in other scenarios which are not handle by current stack. In most user applications this function is not called directly.Also this function can be used by users who are selectively modifying the USB device stack's standard handlers through callback interface exposed by the stack.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	// Returns:
	//		Nothing.
	void (*DataOutStage )(USBD_HANDLE_T hUsb);

	// Function to set EP0 state machine in status_in state.
	//
	// This function is called by USB stack and the application layer to set
	// the EP0 state machine in status_in state. This function will send zero
	// length IN packet on EP0 to host, indicating positive status.
	// Note:
	//	This interface is provided to users to invoke this function in other
	//	scenarios which are not handle by current stack. In most user applications
	//	this function is not called directly.Also this function can be used by
	//	users who are selectively modifying the USB device stack's standard
	//	handlers through callback interface exposed by the stack.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	// Returns:
	//		Nothing.
	void (*StatusInStage )(USBD_HANDLE_T hUsb);

	// Function to set EP0 state machine in status_out state.
	//
	// This function is called by USB stack and the application layer to set
	// the EP0 state machine in status_out state. This function will read the
	// zero length OUT packet received from USB host on EP0.
	// Note:
	//	This interface is provided to users to invoke this function in other
	//	scenarios which are not handle by current stack. In most user applications
	//	this function is not called directly.Also this function can be used by
	//	users who are selectively modifying the USB device stack's standard
	//	handlers through callback interface exposed by the stack.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	// Returns:
	//		Nothing.
	void (*StatusOutStage )(USBD_HANDLE_T hUsb);

	// Function to set EP0 state machine in stall state.
	//
	// This function is called by USB stack and the application layer to generate
	// STALL signaling on EP0 endpoint. This function will also reset the EP0
	// Data buffer.
	// Note:
	//	This interface is provided to users to invoke this function in other
	//	scenarios which are not handle by current stack. In most user applications
	//	this function is not called directly.Also this function can be used by
	//	users who are selectively modifying the USB device stack's standard
	//	handlers through callback interface exposed by the stack.
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	// Returns:
	//		Nothing.
	void (*StallEp0 )(USBD_HANDLE_T hUsb);
} USBD_CORE_API_T;

//
// Undocumented structre require by API
//

#define USB_FULL_SPEED    0
#define USB_HIGH_SPEED    1

#ifndef USB_MAX_EP_NUM
#define USB_MAX_EP_NUM		10
#endif

#ifndef USB_MAX_IF_NUM
#define USB_MAX_IF_NUM		5
#endif

/* USB Endpoint Data Structure */
typedef struct _USB_EP_DATA
{
  uint8_t  *pData;
  uint16_t   Count;
  uint16_t pad0;
} USB_EP_DATA;


/* USB core controller data structure */
typedef struct _USB_CORE_CTRL_T
{
  /* override-able function pointers ~ c++ style virtual functions*/
  uint32_t (*USB_EvtSetupHandler)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_EvtOutHandler)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_ReqVendor)(USBD_HANDLE_T hUsb, uint32_t param1);
  uint32_t (*USB_ReqGetStatus)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_ReqGetDescriptor)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_ReqGetConfiguration)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_ReqSetConfiguration)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_ReqGetInterface)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_ReqSetInterface)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_ReqSetClrFeature)(USBD_HANDLE_T hUsb, uint32_t param1);

  /* USB Device Events Callback Functions */
  uint32_t (*USB_Reset_Event)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_Suspend_Event)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_Resume_Event)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_SOF_Event)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_Power_Event)(USBD_HANDLE_T hUsb, uint32_t param1);
  uint32_t (*USB_Error_Event)(USBD_HANDLE_T hUsb, uint32_t param1);
  uint32_t (*USB_WakeUpCfg)(USBD_HANDLE_T hUsb, uint32_t param1);

  /* USB Core Events Callback Functions */
  uint32_t (*USB_Configure_Event)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_Interface_Event)(USBD_HANDLE_T hUsb);
  uint32_t (*USB_Feature_Event)(USBD_HANDLE_T hUsb);

  /* cache and MMU translation functions */
  uint32_t (* virt_to_phys)(void* vaddr);
  void (* cache_flush)(uint32_t* start_adr, uint32_t* end_adr);

  /* event handlers for endpoints. */
  uint32_t (*ep_event_hdlr[2 * USB_MAX_EP_NUM])(USBD_HANDLE_T hUsb, void* data, uint32_t event);
  void*  ep_hdlr_data[2 * USB_MAX_EP_NUM];

  /* USB class handlers */
  uint32_t (*ep0_hdlr_cb[USB_MAX_IF_NUM])(USBD_HANDLE_T hUsb, void* data, uint32_t event);
  void*  ep0_cb_data[USB_MAX_IF_NUM];
  uint8_t num_ep0_hdlrs;
  /* USB Core data Variables */
  uint8_t max_num_ep; /* max number of endpoints supported by the HW */
  uint8_t device_speed;
  uint8_t  num_interfaces;
  uint8_t  device_addr;
  uint8_t  config_value;
  uint16_t device_status;
  uint8_t *device_desc;
  uint8_t *string_desc;
  uint8_t *full_speed_desc;
  uint8_t *high_speed_desc;
  uint8_t *device_qualifier;
  uint32_t ep_mask;
  uint32_t ep_halt;
  uint32_t ep_stall;
  uint8_t  alt_setting[USB_MAX_IF_NUM];
  /* HW driver data pointer */
  void* hw_data;

  /* USB Endpoint 0 Data Info */
  USB_EP_DATA EP0Data;

  /* USB Endpoint 0 Buffer */
  uint8_t  EP0Buf[64];

  /* USB Setup Packet */
  USB_SETUP_PACKET SetupPacket;

} USB_CORE_CTRL_T;

#pragma pack(pop)

#endif // __LPCUSBD_CORE_H__
