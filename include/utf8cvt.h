/*--------------------------------------------------------------------------
File   : utf8cvt.h

Author : Hoang Nguyen Hoan          Nov. 10, 2006

Desc   : UTF8 codecvt facet

Copyright (c) 2006-2008, I-SYST, all rights reserved

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
#ifndef __UTF8CVT_H__
#define __UTF8CVT_H__

#include <locale>

namespace std {

class codecvt_utf8 : public codecvt<wchar_t, char, mbstate_t>
{
protected:
   virtual codecvt_base::result do_in(mbstate_t &state, 
                                      const char *from, 
                                      const char *from_end, 
                                      const char *&from_next, 
                                      wchar_t *to, 
                                      wchar_t *to_limit, 
                                      wchar_t *&to_next) const;
   virtual int do_length(const mbstate_t &state, const char *from, 
                         const char *from_end, size_t limit) const throw();
   virtual codecvt_base::result do_out(mbstate_t &state, 
                                       const wchar_t *from, 
                                       const wchar_t *from_end, 
                                       const wchar_t *&from_next,
                                       char *to, 
                                       char *to_limit, 
                                       char *&to_next) const;

   // This is stateless so do nothing 
   virtual codecvt_base::result do_unshift(mbstate_t &state, 
                                           char *to, 
                                           char *to_limit, 
                                           char *&to_next) const {
      to_next = to;
      return codecvt_base::ok;
   }
   virtual bool do_always_noconv() const throw() { return false; }
   virtual int do_encoding() const throw() { 
      return 0; // 0 for variable length 
   } 
   virtual int do_max_length() const throw() { 
      return 6; // max length for UTF-8
   }
};

}  // namspace std


#endif   // __UTF8CVT__
