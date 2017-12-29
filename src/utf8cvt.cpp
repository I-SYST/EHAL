/*--------------------------------------------------------------------------
File   : utf8cvt.cpp

Author : Hoang Nguyen Hoan          Nov. 10, 2006

Desc   : UTF8 codecvt facet

Copyright (c) 2006, TidalStream, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose 
with or without fee is hereby granted, provided that the above copyright 
notice and this permission notice appear in all copies, and none of the 
names : I-SYST, TidalStream or its contributors may be used to endorse or 
promote products derived from this software without specific prior written 
permission.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/

#include "utf8.h"
#include "utf8cvt.h"

namespace std {

codecvt_base::result codecvt_utf8::do_in(mbstate_t &state, const char *from, 
                                         const char *from_end, const char *&from_next, 
                                         wchar_t *to, wchar_t *to_limit, 
                                         wchar_t *&to_next) const
{
   int len = from_end - from;
   int destlen = to_limit - to;
   int res = utf8towcs(from, &len , to, &destlen);
   from_next = from + len;
   to_next = to + destlen;
   if (res < 0)
      return codecvt_base::error;
   else if (res == 1)
      return codecvt_base::partial;

   return codecvt_base::ok;
}

int codecvt_utf8::do_length(const mbstate_t &, const char *from, 
                            const char *from_end, size_t limit) const throw()
{
   return utf8towcs_length(from, from_end - from, limit);
}

codecvt_base::result codecvt_utf8::do_out(mbstate_t &state, const wchar_t *from, 
                                          const wchar_t *from_end, const wchar_t *&from_next,
                                          char *to, char *to_limit, char *&to_next) const
{
   int srclen = from_end - from;
   int destsize = to_limit - to;
   int res = wcstoutf8(from, &srclen, to, &destsize);
   from_next = from + srclen;
   to_next = to + destsize;

   if (res < 0)
   {
      return codecvt_base::error;
   }
   else if (res == 1)
   {

      return codecvt_base::partial;
   }

   return codecvt_base::ok;
}

} // namespace std

