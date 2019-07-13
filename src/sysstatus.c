/*--------------------------------------------------------------------------
File   : sysstatus.cpp

Author : Hoang Nguyen Hoan          Oct. 16, 1996

Desc   : System status C implementation.
		 Functions to get and set status code.  It allows keeping track of
		 SYSSTATUS_MAXQUE number of status code with a string of description

Copyright (c) 1996-2008, I-SYST, all rights reserved 

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

----------------------------------------------------------------------------*/
//#pragma file_attr("prefersMem=external")
#include <string.h>
#include <stdatomic.h>
#include <signal.h>

#include "sysstatus.h"

// System state.  Only current state is kept
STATUS g_SysState = 0;

// Only for error & warning
SYSSTATUS g_StatusQue[SYSSTATUS_MAXQUE];
int g_StatusQCurrIdx = 0;

STATUS SysStateGet(void)
{
	return g_SysState;
}

const SYSSTATUS *SysStatusGet(void)
{
	return &g_StatusQue[g_StatusQCurrIdx];
}

// Get current STATUS
STATUS SysStatusGetCode(void)
{
	return g_StatusQue[g_StatusQCurrIdx].Code;
}

const char *SysStatusGetDesc(void)
{
	return g_StatusQue[g_StatusQCurrIdx].Desc;
}

// Get previous STATUS from the Que
const SYSSTATUS *SysStatusGetPrev(void)
{
	int idx = g_StatusQCurrIdx - 1;
	if (idx < 0)
		idx = SYSSTATUS_MAXQUE - 1;
	return &g_StatusQue[idx];
}

STATUS SysStatusGetPrevCode(void)
{
	int idx = g_StatusQCurrIdx - 1;
	if (idx < 0)
		idx = SYSSTATUS_MAXQUE - 1;
	return g_StatusQue[idx].Code;
}

// Set current STATUS
STATUS SysStatusSet(STATUS Code, char *pDesc)
{
	int len = 0;

	if ((Code & SYSSTATUS_TYPE_MASK) == SYSSTATUS_TYPE_RNT)
	{
		// Special case for runtime state machine
		atomic_store((sig_atomic_t *)&g_SysState, Code);
	}

	g_StatusQCurrIdx = (g_StatusQCurrIdx + 1) % SYSSTATUS_MAXQUE;

	atomic_store((sig_atomic_t *)g_StatusQue[g_StatusQCurrIdx].Code, Code);
	if (pDesc)
	{
		strncpy(g_StatusQue[g_StatusQCurrIdx].Desc, pDesc, SYSSTATUS_DESC_MAX);
		len = SYSSTATUS_DESC_MAX - 1;
	}
	g_StatusQue[g_StatusQCurrIdx].Desc[len] = '\0';

	// Store special case Runtime State
	return Code;
}

STATUS SysStateSet(uint32_t State)
{
	int code = (State & 0xfffffff) | SYSSTATUS_TYPE_RNT;

	return SysStatusSet(code, NULL);
}

