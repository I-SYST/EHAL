/*--------------------------------------------------------------------------
File   : base64.c

Author : Hoang Nguyen Hoan          Nov. 3, 2012

Desc   : Base64 encode/decode. It is the Base64 binary to ASCII encode/decode

Copyright (c) 2012, I-SYST inc., all rights reserved

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
#include <stdio.h>
#include <stdint.h>

#include "istddef.h"
#include "base64.h"

/*
 * Base64 Encode binary to ASCII
 *
 * @param 	pSrc : Pointer to source binary data
 * 			SrcLen	: Source data length in bytes
 * 			pDest	: Pointer to ASCII destination buffer
 * 			DstLen	: Destination buffer length in bytes
 *
 * 	@return	Number of bytes encoded
 */
int Base64Encode(uint8_t *pSrc, int SrcLen, char *pDest, int DstLen)
{
	int idx = 0;
	uint32_t d = 0;
	uint8_t *p = (uint8_t *)&d;
	int len;
	int cnt = 0;

	while (SrcLen > 0)
	{
		d = 0;
		len = min(SrcLen - 1, 2);
		for (int i = 2; i >= 0 && SrcLen > 0; i--)
		{
			p[i] = *pSrc;
			pSrc++;
			SrcLen--;
		}
		len += 1 + idx;
		for (int i = idx + 3; i >= idx && cnt < DstLen - 1; i--)
		{
			if (i > len)
				pDest[i] = '=';
			else {
				pDest[i] = d & 0x3f;
				if (pDest[i] < 26)
					pDest[i] += 'A';
				else if (pDest[i] < 52)
					pDest[i] += 'a' - 26;
				else if (pDest[i] < 62)
					pDest[i] += '0' - 52;
				else if (pDest[i] == 63)
					pDest[i] = '/';
				else
					pDest[i] = '+';
			}
			d >>= 6;
			cnt++;
		}
		idx += 4;
	}

	pDest[cnt] = '\0';

	return cnt;
}

