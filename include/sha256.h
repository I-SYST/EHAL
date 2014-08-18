/*--------------------------------------------------------------------------
File   : sha256.h

Author : Hoang Nguyen Hoan          Aug. 17, 2014

Desc   : SHA-256 computation.

Copyright (c) 2014, I-SYST inc., all rights reserved

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

#ifndef __SHA256_H__
#define __SHA256_H__

#include <stdint.h>

/*
 * Test cases.
 * Data   : null, zero length
 * SHA256 : e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855
 *
 * Data   : "abc"
 * SHA256 : BA7816BF 8F01CFEA 414140DE 5DAE2223 B00361A3 96177A9C B410FF61 F20015AD
 *
 * Data   : "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
 * SHA256 : 248D6A61 D20638B8 E5C02693 0C3E6039 A33CE459 64FF2167 F6ECEDD4 19DB06C1
 *
 * Data   : repeat 'a' 1000000 times
 * SHA256 : cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0
 *
 * Data   : "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"
 * SHA256 :  cf5b16a7 78af8380 036ce59e 7b049237 0b249b11 e8f07a51 afac4503 7afee9d1
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/*
 * Generate SHA digest code.  Call this function until all data are processed.
 * set bLast parameter to true for last data packet to process.
 *
 * Make sure to have enough memory for returning results.  pRes must have at
 * least 65 bytes.
 *
 * @param 	pSrc 	: Pointer to source data
 * 			SrcLen	: Source data length in bytes
 *			bLast	: set true to indicate last data packet
 * 			pRes	: Pointer to buffer to store resoults of 64 characters
 * 					  if NULL is passed, internal buffer will be used
 *
 * 	@return	Pointer to digest string. If pRes is NULL, internal buffer is returned
 * 			NULL if incomplete
 */
char *Sha256(uint8_t *pData, int DataLen, bool bLast, char *pRes);

#ifdef __cplusplus
}
#endif


#endif // __SHA256_H__
