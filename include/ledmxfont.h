/*---------------------------------------------------------------------------
File : ledmxfont.h

Author : Hoang Nguyen Hoan          Feb. 28, 2011

Desc : Font definitions for use with IDM-LMX3208 series LED matrix display

Copyright (c) 2011, I-SYST inc, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose 
with or without fee is hereby granted, provided that the above copyright 
notice and this permission notice appear in all copies, and none of the 
names : I-SYST or its contributors may be used to endorse or 
promote products derived from this software without specific prior written 
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY 
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description
----------------------------------------------------------------------------*/
#ifndef __LEDMXFONT_H__
#define __LEDMXFONT_H__

#include <stdint.h>

#if defined(__AVR__)
// ARV does not automaticaly support const data to be located in ROM space.
// It requires PROGMEM attribute
#include <avr/pgmspace.h>
#else
// For other platforms, no PROGMEM require
#define PROGMEM         
#endif

#pragma pack(push, 4)

typedef struct {
	int Width;		// in bits
	uint8_t Data[8];
} LEDMXFONT_BITMAP;

#pragma pack(pop)

extern PROGMEM const LEDMXFONT_BITMAP g_FontBitmap[];
extern PROGMEM const int g_FontBitmapSize;
extern PROGMEM const LEDMXFONT_BITMAP g_FontBitmapAccent[];
extern PROGMEM const int g_FontBitmapAccentSize;

#endif	// __LEDMXFONT_H__
