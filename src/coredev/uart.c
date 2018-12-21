/**-------------------------------------------------------------------------
@file	uart.c

@brief	Generic uart implementation

@author	Hoang Nguyen Hoan
@date	Sept. 16, 2011

@license

Copyright (c) 2011, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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

----------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include "istddef.h"
#include "coredev/uart.h"

extern char s_Buffer[];	// defined in sbuffer.c
extern int s_BufferSize;

void UARTprintf(UARTDEV * const pDev, const char *pFormat, ...)
{
	va_list vl;
    va_start(vl, pFormat);
    UARTvprintf(pDev, pFormat, vl);
    va_end(vl);
}

void UARTvprintf(UARTDEV * const pDev, const char *pFormat, va_list vl)
{
    vsnprintf(s_Buffer, s_BufferSize, pFormat, vl);
    int len = strlen(s_Buffer);
    uint8_t *p = (uint8_t*)s_Buffer;
    int to = 10;

    while (len > 0 && to > 0)
    {
    	int l;
    	l = UARTTx(pDev, p, len);
    	len -= l;
    	p += l;
    	to--;
    }
}

