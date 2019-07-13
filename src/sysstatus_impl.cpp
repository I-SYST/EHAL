/*--------------------------------------------------------------------------
File   : sysstatus.cpp

Author : Hoang Nguyen Hoan          Oct. 16, 1996

Desc   : System status class.

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

#include "atomic.h"
#include "sysstatus.h"

extern STATUS g_SysState;
extern SYSSTATUS g_StatusQue[SYSSTATUS_MAXQUE];
extern int g_StatusQCurrIdx;

SysStatus::SysStatus():vHead(0), vTail(0), vLastAccess(0)
{
   // Init round robin que for status storage
   memset(g_StatusQue, 0, sizeof(g_StatusQue));
}

/*--------------------------------------------------------------------------
 void SysStatus::operator = (STATUS Code)
----------------------------------------------------------------------------
   Set system status with encoded value and reset cursor to current.
 Special case : Runtime (RNT) type code will also be stored in the State
 variable for later retreival.

 Parm.  :   Code : Status value

 Return :   None.
----------------------------------------------------------------------------*/
void SysStatus::operator = (STATUS Code)
{
   vHead = (vHead + 1) % SYSSTATUS_MAXQUE;
   if (vTail == vHead)
      vTail = (vTail + 1) % SYSSTATUS_MAXQUE;

   vStatusQue[vHead].Code = Code;
   vStatusQue[vHead].Desc[0] = '\0';

   vLastAccess = vHead;

   // Store special case Runtime State
   if ((Code & (SYSSTATUS_TYPE_RNT << 28)) == (SYSSTATUS_TYPE_RNT << 28))
   {
      AtomicAssign((sig_atomic_t *)&vState, Code);
   }
}

SysStatus::operator uint32_t ()
{
   vLastAccess = vHead;

   return vStatusQue[vHead].Code;
}

/*--------------------------------------------------------------------------
 STATUS SysStatus::Set(STATUS Code, CHAR *pDesc)
----------------------------------------------------------------------------
   Set system status with encoded value and reset cursor to current.
 Special case : Runtime (RNT) type code will also be stored in the State
 variable for later retreival.

 Parm.  :   Code : Status value
            Desc : Status description string

 Return :   None.
----------------------------------------------------------------------------*/
STATUS SysStatus::SetStatus(STATUS Code, char *pDesc)
{
   vHead = (vHead + 1) % SYSSTATUS_MAXQUE;
   if (vTail == vHead)
      vTail = (vTail + 1) % SYSSTATUS_MAXQUE;

   vStatusQue[vHead].Code = Code;
   if (pDesc)
      strncpy(vStatusQue[vHead].Desc, pDesc, SYSSTATUS_DESC_MAX);
   else
      vStatusQue[vHead].Desc[0] = '\0';

   vLastAccess = vHead;

   // Store special case Runtime State
   if ((Code & (SYSSTATUS_TYPE_RNT << 28)) == (SYSSTATUS_TYPE_RNT << 28))
   {
    AtomicAssign((sig_atomic_t *)&vState, Code);
   }

   return Code;
}  

/*--------------------------------------------------------------------------
 SYSSTATUS *SysStatus::GetStatus(void)
----------------------------------------------------------------------------
   Get current system status

 Parm.  :   None.

 Return :   Pointer to system status structure
----------------------------------------------------------------------------*/
SYSSTATUS *SysStatus::GetStatus(void)
{
   vLastAccess = vHead;

   return &vStatusQue[vHead];
}

/*--------------------------------------------------------------------------
 SYSSTATUS *SysStatus::GetPrevStatus(void)
----------------------------------------------------------------------------
   Get previous system status

 Parm.  :   None.

 Return :   Pointer to system status structure
----------------------------------------------------------------------------*/
SYSSTATUS *SysStatus::GetPrevStatus(void)
{
   vLastAccess = (vLastAccess - 1) % SYSSTATUS_MAXQUE;
   return &vStatusQue[vLastAccess];
}

/*--------------------------------------------------------------------------
 STATUS SysStatus::GetStatusCode(void)
----------------------------------------------------------------------------
   Get current system status code

 Parm.  :   None.

 Return :   Status code
----------------------------------------------------------------------------*/
STATUS SysStatus::GetStatusCode(void)
{
    vLastAccess = vHead;
    return vStatusQue[vHead].Code;
}

/*--------------------------------------------------------------------------
 STATUS SysStatus::GetPrevSstatusCode(void)
----------------------------------------------------------------------------
   Get previous system status code

 Parm.  :   None.

 Return :   Status code
----------------------------------------------------------------------------*/
STATUS SysStatus::GetPrevStatusCode(void)
{
   vLastAccess = (vLastAccess - 1) % SYSSTATUS_MAXQUE;
   return vStatusQue[vLastAccess].Code;
}

/*--------------------------------------------------------------------------
 STATUS SysStatus::GetState(void)
----------------------------------------------------------------------------
   Get system state code

 Parm.  :   None.

 Return :   Status code
----------------------------------------------------------------------------*/
STATUS SysStatus::GetState(void)
{
   return vState;
}

/*--------------------------------------------------------------------------
 STATUS SysStatus::SetState(STATUS State)
----------------------------------------------------------------------------
   Set system state code

 Parm.  :   State : State code.

 Return :   State code
----------------------------------------------------------------------------*/
STATUS SysStatus::SetState(STATUS State)
{
   return SetStatus(State);
}

