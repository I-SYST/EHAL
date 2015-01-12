/*
 * lpc11Uxx_usbrom.h
 *
 *  Created on: Nov 9, 2014
 *      Author: hoan
 */

#ifndef __LPC11UXX_USBROM_H__
#define __LPC11UXX_USBROM_H__

#include <stdint.h>

#include "lpcusbd_api.h"


#define USBD_RAM_BASE      0x20004000
#define USBD_RAM_SIZE      0x0800

/* A table of pointers to the USBD functions contained in ROM is located at the address contained at this location */
#define USBD_FUNCTION_TABLE_PTR_ADDR                    (0x1FFF1FF8UL)

#define USBD_API ((USBD_API_T*)(*((USBD_API_T **)(*(void**)USBD_FUNCTION_TABLE_PTR_ADDR))))



#endif // __LPC11UXX_USBROM_H__
