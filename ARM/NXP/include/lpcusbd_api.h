/*
 * lpcusbd_api.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPCUSBD_API_H__
#define __LPCUSBD_API_H__

#include <stdint.h>

#include "lpcusbd_hw.h"
#include "lpcusbd_core.h"
#include "lpcusbd_msc.h"
#include "lpcusbd_dfu.h"
#include "lpcusbd_hid.h"
#include "lpcusbd_cdc.h"

#pragma pack(push, 1)

typedef struct {
	const USBD_HW_API_T* hw;		// Pointer to USB stack core layer to function table
	const USBD_CORE_API_T* core;	// Pointer to USB device controller hardware function table
	const USBD_MSC_API_T* msc;		// Pointer to Mass Storage Controller function table
	const USBD_DFU_API_T* dfu;		// Pointer to DFU function table
	const USBD_HID_API_T* hid;		// Pointer to HID function table
	const USBD_CDC_API_T* cdc;		// Pointer to CDC function table
	const uint32_t* reserved6;
	const uint32_t version;			// Version identifier of USB ROM stack.
									// The version is defined as 0x0CHDMhCC where each nibble
									// represents version number of the corresponding component.
									// 		CC - 7:0 - 8bit core version number
									// 		h - 11:8 - 4bit hardware interface version number
									// 		M - 15:12 - 4bit MSC class module version number
									//		D - 19:16 - 4bit DFU class module version number
									// 		H - 23:20 - 4bit HID class module version number
									//		C - 27:24 - 4bit CDC class module version number
									// 		H - 31:28 - 4bit reserved
} USBD_API_T;

#pragma pack(pop)

#endif // __LPCUSBD_API_H__
