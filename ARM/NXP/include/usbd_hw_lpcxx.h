/*
 * lpc11Uxx_usbhw.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPC11UXX_USBHW_H__
#define __LPC11UXX_USBHW_H__

#include <stdint.h>
#include "usb_def.h"
#include "usbd_core_lpcxx.h"

enum  USBD_EVENT_T {
  USB_EVT_SETUP			= 1,
  USB_EVT_OUT			= 2,
  USB_EVT_IN			= 3,
  USB_EVT_OUT_NAK		= 4,
  USB_EVT_IN_NAK		= 5,
  USB_EVT_OUT_STALL		= 6,
  USB_EVT_IN_STALL		= 7,
  USB_EVT_OUT_DMA_EOT	= 8,
  USB_EVT_IN_DMA_EOT	= 9,
  USB_EVT_OUT_DMA_NDR	= 10,
  USB_EVT_IN_DMA_NDR	= 11,
  USB_EVT_OUT_DMA_ERR	= 12,
  USB_EVT_IN_DMA_ERR	= 13,
  USB_EVT_RESET			= 14,
  USB_EVT_SOF			= 15,
  USB_EVT_DEV_STATE		= 16,
  USB_EVT_DEV_ERROR		= 17
};

#pragma pack(push, 1)

typedef struct {
	USB_DEVICE_DESC *device_desc; 	// Pointer to USB device descriptor
	uint8_t *string_desc;			// Pointer to array of USB string descriptors
	uint8_t *full_speed_desc;
	uint8_t *high_speed_desc;
	uint8_t *device_qualifier;
} USB_CORE_DESCS_T;

typedef struct {
	uint32_t 	usb_reg_base;			// USB device controller's base register address.
	uint32_t 	mem_base;				// Base memory location from where the stack can
										// allocate data and buffers. Must be 2048 bytes
										// aligned and accessible by USB DMA controller
	uint32_t 	mem_size;				// The size of memory buffer which stack can use.
	uint8_t 	max_num_ep;				// max number of endpoints supported by the USB
										// device controller instance
	uint8_t 	pad0 [3];


	// Event callback for USB interface reset. This event fires when the USB host requests that the
	// device reset its interface. This event fires after the control endpoint has been
	// automatically configured by the library.
	// Note:
	// This event is called from USB_ISR context and hence is time-critical. Having delays
	// in this callback will prevent the device from enumerating correctly or operate properly.
	uint32_t (*USB_Reset_Event)(USBD_HANDLE_T hUsb);

	// Event for USB suspend. This event fires when the USB host suspends the device by
	// halting its transmission of Start Of Frame pulses to the device. This is generally
	// hooked in order to move the device over to a low power state until the host wakes
	// up the device.
	// Note:
	// This event is called from USB_ISR context and hence is time-critical. Having delays
	// in this callback will cause other system issues.
	uint32_t (*USB_Suspend_Event)(USBD_HANDLE_T hUsb);

	// Event for USB wake up or resume. This event fires when a the USB device interface
	// is suspended and the host wakes up the device by supplying Start Of Frame pulses.
	// This is generally hooked to pull the user application out of a low power state and
	// back into normal operating mode.
	// Note:
	// This event is called from USB_ISR context and hence is time-critical. Having delays
	// in this callback will cause other system issues.
	uint32_t (*USB_Resume_Event)(USBD_HANDLE_T hUsb);

	void* 	reserved_sbz;	// Reseved, must be set to 0

	// Event for USB Start Of Frame detection, when enabled. This event fires at the start
	// of each USB frame, once per millisecond in full-speed mode or once per 125
	// microseconds in high-speed mode, and is synchronized to the USB bus.
	//
	// This event is time-critical; it is run once per millisecond (full-speed mode) and
	// thus long handlers will significantly degrade device performance. This event should
	// only be enabled when needed to reduce device wake-ups.
	//
	// Note:
	// This event is not normally active - it must be manually enabled and disabled via the
	// USB interrupt register.
	uint32_t (*USB_SOF_Event)(USBD_HANDLE_T hUsb);

	// Event for remote wake-up configuration, when enabled. This event fires when the USB
	// host request the device to configure itself for remote wake-up capability. The USB
	// host sends this request to device which report remote wake-up capable in their
	// device descriptors, before going to low-power state. The application layer should
	// implement this callback if they have any special on board circuit to trigger remote
	// wake up event. Also application can use this callback to differentiate the following
	// SUSPEND event is caused by cable plug-out or host SUSPEND request. The device can
	// wake-up host only after receiving this callback and remote wake-up feature is
	// enabled by host. To signal remote wake-up the device has to generate resume signaling
	// on bus by calling usapi.hw->WakeUp() routine.
	//
	// Parameters:
	// 		[in]	hUsb	Handle to the USB device stack.
	// 		[in]	param1	When 0 - Clear the wake-up configuration,
	// 						 	 1 - Enable the wake-up configuration.
	// Returns:
	// 		The call back should return ErrorCode_t type to indicate success or error condition.
	uint32_t (*USB_WakeUpCfg)(USBD_HANDLE_T hUsb, uint32_t param1);

	uint32_t (*USB_Power_Event)(USBD_HANDLE_T hUsb, uint32_t param1);	// Reserved parameter should be set to zero.

	// Event for error condition. This event fires when USB device controller detect an
	// error condition in the system.
	//
	// Parameters:
	//		[in]	hUsb	Handle to the USB device stack.
	// 		[in]	param1	USB device interrupt status register.
	// Returns:
	// 		The call back should return ErrorCode_t type to indicate success or error condition.
	uint32_t (*USB_Error_Event)(USBD_HANDLE_T hUsb, uint32_t param1);

	// Event for USB configuration number changed. This event fires when a the USB host changes
	// the selected configuration number. On receiving configuration change request from host,
	// the stack enables/configures the endpoints needed by the new configuration before calling
	// this callback function.
	// Note:
	// This event is called from USB_ISR context and hence is time-critical. Having delays in this
	// callback will prevent the device from enumerating correctly or operate properly.
	uint32_t (*USB_Configure_Event)(USBD_HANDLE_T hUsb);

	// Event for USB interface setting changed. This event fires when a the USB host changes the
	// interface setting to one of alternate interface settings. On receiving interface change
	// request from host, the stack enables/configures the endpoints needed by the new alternate
	// interface setting before calling this callback function.
	// Note:
	// This event is called from USB_ISR context and hence is time-critical. Having delays in
	// this callback will prevent the device from enumerating correctly or operate properly.
	uint32_t (*USB_Interface_Event)(USBD_HANDLE_T hUsb);

	// Event for USB feature changed. This event fires when a the USB host send set/clear feature
	// request. The stack handles this request for USB_FEATURE_REMOTE_WAKEUP, USB_FEATURE_TEST_
	// MODE and USB_FEATURE_ENDPOINT_STALL features only. On receiving feature request from host,
	// the stack handle the request appropriately and then calls this callback function.
	// Note:
	// This event is called from USB_ISR context and hence is time-critical. Having delays in this
	// callback will prevent the device from enumerating correctly or operate properly.
	uint32_t (*USB_Feature_Event)(USBD_HANDLE_T hUsb);

	uint32_t(* 	virt_to_phys )(void *vaddr);	// Reserved parameter for future use. should be set to zero.
	void(* 	cache_flush )(uint32_t *start_adr, uint32_t *end_adr);
} USBD_API_INIT_PARAM_T;

typedef struct {
	uint32_t (*GetMemSize )(USBD_API_INIT_PARAM_T *param);
	uint32_t (*Init )(USBD_HANDLE_T *phUsb, USB_CORE_DESCS_T *pDesc, USBD_API_INIT_PARAM_T *param);
	void(* 	Connect )(USBD_HANDLE_T hUsb, uint32_t con);
	void(* 	ISR )(USBD_HANDLE_T hUsb);
	void(* 	Reset )(USBD_HANDLE_T hUsb);
	void(* 	ForceFullSpeed )(USBD_HANDLE_T hUsb, uint32_t cfg);
	void(* 	WakeUpCfg )(USBD_HANDLE_T hUsb, uint32_t cfg);
	void(* 	SetAddress )(USBD_HANDLE_T hUsb, uint32_t adr);
	void(* 	Configure )(USBD_HANDLE_T hUsb, uint32_t cfg);
	void(* 	ConfigEP )(USBD_HANDLE_T hUsb, USB_ENDPOINT_DESC *pEPD);
	void(* 	DirCtrlEP )(USBD_HANDLE_T hUsb, uint32_t dir);
	void(* 	EnableEP )(USBD_HANDLE_T hUsb, uint32_t EPNum);
	void(* 	DisableEP )(USBD_HANDLE_T hUsb, uint32_t EPNum);
	void(* 	ResetEP )(USBD_HANDLE_T hUsb, uint32_t EPNum);
	void(* 	SetStallEP )(USBD_HANDLE_T hUsb, uint32_t EPNum);
	void(* 	ClrStallEP )(USBD_HANDLE_T hUsb, uint32_t EPNum);
	uint32_t (*SetTestMode )(USBD_HANDLE_T hUsb, uint8_t mode);
	uint32_t (*ReadEP )(USBD_HANDLE_T hUsb, uint32_t EPNum, uint8_t *pData);
	uint32_t (*ReadReqEP )(USBD_HANDLE_T hUsb, uint32_t EPNum, uint8_t *pData, uint32_t len);
	uint32_t (*ReadSetupPkt )(USBD_HANDLE_T hUsb, uint32_t EPNum, uint32_t *pData);
	uint32_t (*WriteEP )(USBD_HANDLE_T hUsb, uint32_t EPNum, uint8_t *pData, uint32_t cnt);
	void (*WakeUp )(USBD_HANDLE_T hUsb);
	uint32_t (*EnableEvent )(USBD_HANDLE_T hUsb, uint32_t EPNum, uint32_t event_type, uint32_t enable);
} USBD_HW_API_T;

#pragma pack(pop)



#endif // __LPC11UXX_USBHW_H__ 
