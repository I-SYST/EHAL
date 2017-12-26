/**-------------------------------------------------------------------------
@file	utf8.h

@brief	UTF8 conversion utilities


@author	Hoang Nguyen Hoan
@date	Jun. 20, 2002

@license

Copyright (c) 2002-2008, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose 
with or without fee is hereby granted, provided that the above copyright 
notice and this permission notice appear in all copies, and none of the 
names : I-SYST or its contributors may be used to endorse or 
promote products derived from this software without specific prior written 
permission.

For info or contributing contact : hoan at i-syst dot com

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
#ifndef __UTF8_H__
#define __UTF8_H__

#include <stdio.h>

/** @addtogroup Utilities
  * @{
  */

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief	Get require destination for utf8 to mbcs conversion.
 *
 * @param   pSrc     UTF8 source string
 * @param   SrcSize  UFT8 source length in bytes
 * @param   DestLen  Max destination length in number of characters
 *
 * @return  Converted length in number of characters 
 */
size_t utf8towcs_length(const char *pSrc, size_t SrcSize, size_t DestLen);

/**
 * @brief	Convert UTF8 to wide char.
 * 
 * @param   pSrc     Pointer to UFT8 source string
 * @param   pSrcSize Max input buffer length in bytes, return number of
 *                   bytes converted
 * @param   pDest    Pointer to resulting wide string
 * @param   pDestLen Max buffer length in number of characters, return number
 *                   of wchar_t converted
 * 
 * @return  0 - Conversion completed\n
 *          1 - Partial conversion\n
 *         -1 - Error during conversion
 */ 
int utf8towcs(const char *pSrc, int *pSrcSize, wchar_t *pDest, int *pDestLen);

/**
 * @brief	Convert wide char to UFT8
 * 
 * @param   pSrc        Pointer to wide char source string
 * @param   pSrcLen     Max input string length , return number of
 *                      character converted
 * @param   pDest       Pointer to resulting UFT9 string
 * @param   pDestSize   Max buffer size in bytes,
 *                      return number of bytes converted
 * 
 * @return  0 - Conversion completed\n
 *          1 - Partial conversion\n
 *         -1 - Error during conversion
 */ 
int wcstoutf8(const wchar_t *pSrc, int *pSrcLen, char *pDest, int *pDestSize);

#ifdef __cplusplus
}
#endif

/** @} End of group Utilities */

#endif   // __UTF8_H__
