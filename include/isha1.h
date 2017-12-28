/**-------------------------------------------------------------------------
@file	isha1.h

@brief	SHA-1 computation.

@author	Hoang Nguyen Hoan
@date	Aug. 17, 2014

@license

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

----------------------------------------------------------------------------*/

#ifndef __ISHA1_H__
#define __ISHA1_H__

#include <stdint.h>

/** @addtogroup Utilities
  * @{
  */

/*
 * Test cases
 * Data   : null, zero length
 * SHA1   : da39a3ee 5e6b4b0d 3255bfef 95601890 afd80709
 *
 * Data   : "abc"
 * SHA1 : a9993e36 4706816a ba3e2571 7850c26c 9cd0d89d
 *
 * Data   : "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
 * SHA1 : 84983e44 1c3bd26e baae4aa1 f95129e5 e54670f1
 *
 * Data   : repeat 'a' 1000000 times
 * SHA1 : 34aa973c d4c4daa4 f61eeb2b dbad2731 6534016f
 *
 * Data   : "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"
 * SHA1 :  a49b2446 a02c645b f419f995 b6709125 3a04a259
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief	Generate SHA digest code.
 *
 * Call this function until all data are processed.
 * set bLast parameter to true for last data packet to process.
 *
 * Make sure to have enough memory for returning results.  pRes must have at
 * least 33 bytes.
 *
 * @param 	pSrc 	: Pointer to source data
 * @param	SrcLen	: Source data length in bytes
 * @param	bLast	: set true to indicate last data packet
 * @param	pRes	: Pointer to buffer to store results of 32 characters
 * 					  if NULL is passed, internal buffer will be used
 *
 * 	@return	Pointer to digest string. If pRes is NULL, internal buffer is returned
 * 			NULL if incomplete
 */
char *Sha1(uint8_t *pData, int DataLen, bool bLast, char *pRes);

#ifdef __cplusplus
}
#endif

/** @} End of group Utilities */

#endif // __ISHA1_H__
