/*--------------------------------------------------------------------------
File   : istddef.h

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : Standard generic defines
 	 	 Mostly for compatibilities

Copyright (c) 2011, I-SYST inc., all rights reserved

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

#ifndef __ISTDDEF_H__
#define __ISTDDEF_H__

#ifndef __cplusplus
#include <stdbool.h>
#endif // __cplusplus

#include <stdint.h>

#ifndef Bool
typedef bool	Bool;
#endif

#ifndef FALSE
#define FALSE		false
#endif
#ifndef TRUE
#define TRUE		true
#endif

#ifndef min
#define min(x, y)	((x) > (y) ? (y) : (x))
#endif

#ifndef max
#define max(x, y)	((x) > (y) ? (x) : (y))
#endif

typedef struct {
	char 		Name[16];
	uint32_t	Vers;
	uint32_t	Build;
} VER;

#endif // __ISTDDEF_H__

