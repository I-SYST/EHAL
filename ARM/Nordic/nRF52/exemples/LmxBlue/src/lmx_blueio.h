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
#define LMXBLUE_UUID_MSGCHAR 		0x2

#define LMXBLUE_MAX_MSG_LENGTH		255

typedef enum {
	LMXDISP_STYLE_LEFT,
	LMXDISP_STYLE_CENTER,
	LMXDISP_STYLE_RIGHT,
	LMXDISP_STYLE_SCROLL_LEFT,
	LMXDISP_STYLE_SCROLL_RIGHT
} LMXDISP_STYLE;

#pragma pack(push, 1)

typedef struct {
	int LineNo;				// Display line number
	LMXDISP_STYLE Style;	// Display style
	char Text[LMXBLUE_MAX_MSG_LENGTH + 1];
} LMX_MSG;

#pragma pop()

#endif /* LMX_BLUEIO_H_ */
