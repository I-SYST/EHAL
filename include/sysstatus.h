/*--------------------------------------------------------------------------
File   : sysstatus.h

Author : Hoang Nguyen Hoan          Oct. 16, 1996

Desc   : System status class.
		 The system status contains many different types
		 Runtime state which serve as state machine states is stored separately
		 from other statuses. See SYSSTATUS_TYPE for type definitions

Copyright (c) 1996-2008, I-SYST, All rights reserved

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
Modified by         Date           	Description
Hoan                Mar. 18, 2005	namespace TS
Hoan				Nov. 18, 2014	Reimplementing for new EHAL C based
----------------------------------------------------------------------------*/
#ifndef __SYSSTATUS_H__
#define __SYSSTATUS_H__

#include "sysstatusdef.h"

// Max number of status code queued in system
#define SYSSTATUS_MAXQUE      3

// Max string len for status description
#define SYSSTATUS_DESC_MAX    128

#pragma pack(push, 4)

typedef struct __Sys_Status {
   STATUS Code;   // Status code
   char Desc[SYSSTATUS_DESC_MAX];   // Description string
} SYSSTATUS;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

// C implementation

// Get current status
STATUS SysStateGet(void);
const SYSSTATUS *SysStatusGet(void);
STATUS SysStatusGetCode(void);
const char *SysStatusGetDesc(void);

// Get previous STATUS from the Que
const SYSSTATUS *SysStatusGetPrev(void);
STATUS SysStatusGetPrevCode(void);

// Set current STATUS
STATUS SysStatusSet(STATUS Code, char *pDesc);

#ifdef __cplusplus
}

//
// C++ class implementation
//
class SysStatus {
public:
   SysStatus();
   virtual ~SysStatus() {}

   /**
    * Set system status with encoded value and reset cursor to current.
    * Special case : Runtime (RNT) type code will also be stored in the State
    * variable for later retrieval.
    */
   void operator = (STATUS Code) { SysStatusSet(Code, NULL); }

   // Conversion operator : Get encoded current status
   operator uint32_t () { return SysStatusGetCode(); }

   /**
    * Get current status code
    * 
    * @Return  Pointer to system status structure
    */
   const SYSSTATUS *GetStatus(void) { return SysStatusGet(); }

   /**
    * Get previous system status
    * 
    * @Return  Pointer to system status structure
    */
   const SYSSTATUS *GetPrevStatus(void) { return SysStatusGetPrev(); }

   /**
    * Get current system status code
    * 
    * @Return  Status code
    */
   STATUS GetStatusCode(void) { return SysStatusGetCode(); }

   /**
    * Get previous system status code
    * 
   * @Return  Pointer to system status structure
    */ 
   STATUS GetPrevStatusCode(void) { return SysStatusGetPrevCode(); }

   /**
    * Get system state code
    * 
    * @Return  Status code
    */
   STATUS GetState(void) { return SysStateGet(); }

   /**
    * Set system status with encoded value and reset cursor to current.
    * Special case : Runtime (RNT) type code will also be stored in the State
    * variable for later retreival.
    * 
    * @Param   Code : Status value
    * @Param   Desc : Status description string
    */
   STATUS SetStatus(STATUS status, char *pDesc = NULL) { return SysStatusSet(Code, pDesc); }

   /**
    * Set system state code
    * 
    * @Param   State : State code.
    * 
    * @Return  State code
    */
   STATUS SetState(STATUS State) { return SysStatusSet(Code, NULL); }

private:
//   uint32_t		vHead;
//   uint32_t		vTail;
//   uint32_t		vLastAccess;        // Last que position accessed
   //SYSSTATUS	vStatusQue[SYSSTATUS_MAXQUE];
   //STATUS		vState;
};
#endif



#endif // __SYSSTATUS_H__
