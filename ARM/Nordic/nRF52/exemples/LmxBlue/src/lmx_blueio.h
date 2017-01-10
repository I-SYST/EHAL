/*
 * lmx_blueio.h
 *
 *  Created on: Jan 7, 2017
 *      Author: hoan
 */

#ifndef __LMX_BLUEIO_H__
#define __LMX_BLUEIO_H__

// Base UUID : 00000000-4070-11e4-84d0-0002a5d5c51b
#define LMXBLUE_UUID_BASE	{ 0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0xd0, 0x84, \
							  0xe4, 0x11, 0x70, 0x40, 0x00, 0x00, 0x00, 0x00 }
#define LMXBLUE_UUID_SERVICE 		0x1
#define LMXBLUE_UUID_RDCHAR 		0x2
#define LMXBLUE_UUID_WRCHAR 		0x3

typedef enum {

} LMX;

#pragma pack(push, 1)

typedef struct {
	uint32_t Value;
	union {
		uint32_t Code:8;
		uint32_t Resv:8;
		uint32_t Module:12;
		uint32_t Type:4;
	};
} BLUEIO_PKTHDR;

typedef struct {
	BLUEIO_PKTHDR Hdr;
	uint8_t Data[1];
} BLUEIO_PKT;

#pragma pop()

#endif /* LMX_BLUEIO_H_ */
