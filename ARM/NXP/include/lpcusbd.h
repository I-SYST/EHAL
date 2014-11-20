/*
 * lpc11Uxx_usb.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPC11UXX_USB_H__
#define __LPC11UXX_USB_H__

#include <stdint.h>

#include "usb_def.h"

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
	uint8_t Recipient:5;
	uint8_t Type:2;
	uint8_t Dir:1;
} BM_T;

typedef union {
	uint8_t B;
	BM_T 	BM;
} REQUEST_TYPE;

typedef struct {
	uint8_t L;
	uint8_t H;
} WB_T;

typedef union {
	uint16_t 	W;
	WB_T 		WB;
} WORD_BYTE;

typedef struct {
	REQUEST_TYPE 	bmRequestType;	// This bitmapped field identifies the
									// characteristics of the specific request.
	uint8_t 	bRequest;			// This field specifies the particular request.
									// The Type bits in the bmRequestType field modify
									// the meaning of this field.
	WORD_BYTE 	wValue;				// Used to pass a parameter to the device,
									// specific to the request.
	WORD_BYTE 	wIndex;				// Used to pass a parameter to the device,
									// specific to the request. The wIndex field
									// is often used in requests to specify an
									// endpoint or an interface.
	uint16_t 	wLength;			// This field specifies the length of the data
									// transferred during the second phase of the
									// control transfer.
} USB_SETUP_PACKET;

#pragma pack(pop)

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

#endif // __LPC11UXX_USB_H__ 
