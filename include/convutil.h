/**-------------------------------------------------------------------------
@file	convutil.h

@brief	Conversion utilities

@author Hoang Nguyen Hoan
@date	Feb. 8, 2015

@license

Copyright (c) 2015, I-SYST inc., all rights reserved

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

#ifndef __CONVUTIL_H__
#define __CONVUTIL_H__

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/** @addtogroup Utilities
  * @{
  */

#ifdef _MSC_VER
// Microsoft does not support C99 inline
#ifndef inline
#define inline __forceinline
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	16 bits endianess conversion.
 *
 * @param	x : 16 bits number to covert.
 *
 * @return	converted data.
 */
static inline int16_t EndianCvt16(int16_t x) {
	return ((x >> 8) & 0xff) | ((x << 8) & 0xff00);
}

/**
 * @brief	32 bits endianess conversion.
 *
 * @param	x : 32 bits number to covert.
 *
 * @return	converted data.
 */
static inline uint32_t EndianCvt32(uint32_t x) {
	return (((x >> 24UL) & 0xff) | ((x << 24UL) & 0xff000000) |
			((x >> 8UL) & 0xff00) | ((x << 8UL) & 0xff0000));
}

/**
 * @brief	64 bits endianess conversion.
 *
 * @param	x : 64 bits number to covert.
 *
 * @return	converted data.
 */
static inline uint64_t EndianCvt64(uint64_t x) {
	return (((x >> 56ULL) & 0xffULL) | ((x << 56ULL) & 0xff00000000000000ULL) |
			((x >> 40ULL) & 0xff00ULL) | ((x << 40ULL) & 0xff000000000000ULL) |
			((x >> 24ULL) & 0xff0000ULL) | ((x << 24ULL) & 0xff0000000000ULL) |
			((x >> 8ULL) & 0xff000000ULL) | ((x << 8ULL) & 0xff00000000ULL));
}

/**
 * @brief	Convert ASCII hex character to integer.
 *
 * @param	c : Hex character to convert
 *
 * @return 	converted value.  -1 if wrong character
 */
static inline int chex2i(char c) {
	if (c >= 'a')
		return (c - 'a' + 10);
	else if (c >= 'A')
		return (c - 'A' + 10);

	return (c - '0');
}
	
#ifdef __cplusplus
}
#endif

/** @} End of group Utilities */

#endif // __CONVUTIL_H__
