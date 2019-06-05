/*--------------------------------------------------------------------------
File   : system_core_clock.h

Author : Hoang Nguyen Hoan          Aug. 30, 2017

Desc   : Contains core specific default clock and related clock factors
		 for calculating delay loops, etc...

Copyright (c) 2017, I-SYST inc., all rights reserved

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

#ifndef __SYSTEM_CORE_CLOCK_H__
#define __SYSTEM_CORE_CLOCK_H__

#define SYSTEM_CORE_CLOCK				16000000UL		// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(30UL)			// TODO: Adjustment value

#ifdef __cplusplus
extern "C" {
#endif
uint32_t SystemCoreClockSet(uint32_t CoreFreq, int ClkFreq);
#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_CORE_CLOCK_H__
