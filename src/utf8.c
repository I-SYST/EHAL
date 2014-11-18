/*--------------------------------------------------------------------------
File   : utf8.c

Author : Hoang Nguyen Hoan          jun. 20, 2002

Desc   : UTF8 conversion

Copyright (c) 2002-2008 I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose 
with or without fee is hereby granted, provided that the above copyright 
notice and this permission notice appear in all copies, and none of the 
names : I-SYST or its contributors may be used to endorse or 
promote products derived from this software without specific prior written 
permission.

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
Hoan Hoang           Nov. 10, 2006     For codecvt_utf8 which requires 
                                       conversion states
----------------------------------------------------------------------------*/
#include "utf8.h"

int uft8towcs_octetcount(char c)
{
   char mask = (char)0xfc;
   int retval = 6; 

   if ((c & 0x80) == 0)
      return 1;


   while ((c & mask) != mask && retval > 0)
   {
      retval--;
      mask <<= 1;
   }

   return retval;
}

int wcstoutf8_octetcount(int c)
{
   if (c < 0x80)
      return 1;

   if (c < 0x800)
      return 2;

   if (sizeof(c) < 3 || c < 0x10000)
      return 3;

   if (c < 0x200000)
      return 4;

   if (c < 0x4000000)
      return 5;

   return 6;
}

size_t utf8towcs_length(const char *pSrc, size_t SrcSize, size_t DestLen)
{
   size_t retval = 0;
   size_t cnt, len = 0;

   while (len < SrcSize && retval < DestLen)
   {
      cnt = uft8towcs_octetcount(*pSrc);
      if (cnt <= 0 || (len + cnt) > SrcSize)
         break;

      retval++;
      pSrc += cnt;
   }

   return retval;
}

/**
 * Convert UTF8 to wide char
 * 
 * @param   pSrc     pointer to UFT8 source string
 * @param   pSrcSize Max input buffer length in bytes, return number of
 *                   bytes converted
 * @param   pDest    Pointer to resulting wide string
 * @param   pDestLen Max buffer length in number of characters, return number
 *                   of wchar_t converted
 * 
 * @return  0 - Conversion completed
 *          1 - Partial conversion
 *         -1 - Error during conversion
 */ 
int utf8towcs(const char *pSrc, int *pSrcSize, wchar_t *pDest, int *pDestLen)
{
   int retval = 0, len;
   char *srcend;
   wchar_t *destend;
   
   if (pSrc == NULL || pDest == NULL || pSrcSize == NULL || *pSrcSize <= 0 || pDestLen == NULL || *pDestLen <=0)
   {
      if (pSrcSize)
         *pSrcSize = 0;
      if (*pDestLen)
      {
         *pDestLen = 0;
      }
      return -1;
   }

   len = *pSrcSize;
   srcend = (char *)pSrc + *pSrcSize;
   destend = pDest + *pDestLen;
   *pSrcSize = 0;
   *pDestLen = 0;
   while (pSrc < srcend && pDest < destend)
   {
      int cnt = uft8towcs_octetcount(*pSrc);

      if (cnt <= 0)
      {
         return -1;
      }
      if (cnt > len)
         return 1;

      if (cnt > 1)
      {
         char mask = (char)0x7f;
         char *b = (char *)pSrc;
         int i;

         *pDest = (*pSrc & (mask >> cnt));
         pSrc++;
         ++*pSrcSize;

         for (i = 1; i < cnt && pSrc != srcend; i++)
         {
            if ((*pSrc & 0xc0) != 0x80)
            {
               return retval;
            }
            *pDest <<= 6;
            *pDest |= (*pSrc & 0x3f);
            pSrc++;
            ++*pSrcSize;
         }
      }
      else
      {
         *pDest = (*pSrc & 0x7f);
         pSrc++;
         ++*pSrcSize;
      }
      pDest++;
      ++*pDestLen;
      retval++;
      len -= cnt;
   } 
   
   return 0;
}

/**
 * Convert wide char to UFT8
 * 
 * @param   pSrc        Pointer to wide char source string
 * @param   pSrcLen     Max input string length , return number of
 *                      character converted
 * @param   pDest       Pointer to resulting UFT9 string
 * @param   pDestSize   Max buffer size in bytes,
 *                      return number of bytes converted
 * 
 * @return  0 - Conversion completed
 *          1 - Partial conversion
 *         -1 - Error during conversion
 */ 
int wcstoutf8(const wchar_t *pSrc, int *pSrcLen, char *pDest, int *pDestSize)
{
   int retval = 0;
   wchar_t *srcend;
   char *destend;
   char *p, mask;
   wchar_t c;
   int i;

   if (pSrc == NULL || pDest == NULL || pSrcLen == NULL || *pSrcLen <= 0 || pDestSize == NULL || *pDestSize <=0)
   {
      if (pSrcLen)
         *pSrcLen = 0;
      if (pDestSize)
         *pDestSize = 0;

      return -1;
   }

   srcend = (wchar_t *)pSrc + *pSrcLen;
   destend = (char *)pDest + *pDestSize;
   *pSrcLen = 0;
   *pDestSize = 0;

   while (pSrc < srcend && pDest < destend)
   {
      int cnt = wcstoutf8_octetcount(*pSrc);

      if (cnt > 1)
      {
         if ((pDest + cnt) >= destend)
         {
            return 1;
         }

         p = pDest + cnt - 1;
         c = *pSrc;

         for (i = 1; i < cnt; i++)
         {
            *p = 0x80 | (c & 0x3f);
            p--;
            c >>= 6;
         }
         mask = 0xfc << (6 - cnt);
         
         *p = mask | (c & ~mask);
         pDest += cnt;
         (*pDestSize) += cnt;
      }
      else
      {
         *pDest = (char)*pSrc;
         pDest++;
         ++*pDestSize;
      }

      pSrc++;
      ++*pSrcLen;
   }

   return 0;
}
