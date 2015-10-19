/*--------------------------------------------------------------------------
File   : md5.h

Author : Hoang Nguyen Hoan          Nov. 19, 2012

Desc   : MD5 calculations
		 reference : http://en.wikipedia.org/wiki/MD5

Copyright (c) 2012, I-SYST, all rights reserved

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

#ifndef __MD5_H__
#define __MD5_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Calculate MD5 value
 *
 * @param	pData	: Pointer to source data
 * 			DataLen	: Data length in bytes
 * 			pRes	: Pointer to buffer to store MD5 value
 *
 * 	@return None.
 */
void md5(uint8_t *pData, int DataLen, uint8_t *pRes);

#ifdef __cplusplus
}
#endif


#endif	// __MD5_H__
