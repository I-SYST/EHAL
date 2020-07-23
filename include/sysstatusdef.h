/*--------------------------------------------------------------------------
File   : sysstatusdef.h

Author : Hoang Nguyen Hoan          Oct. 16, 1996

Desc   : System status definitions & encoding.

         The system status keeps track of all system runtime states.

         it consists of a 32 bits value encoded as follow:

         31           28|27                 16|15                8|7              0
         +--------------+---------------------+-------------------+---------------+
         | Type (4 bits)| Module ID (12 bits) | reserved (8 bits) | Code (8 bits) |
         +--------------+---------------------+-------------------+---------------+

         All subsystems or library modules must reserve its module ID
         in this file.

         The status value (encoded) of 0 is reserved for Ready/No error

         Type : 0000b    Runtime states
                0100b    Warning (need attention)
                1000b    Non fatal error (recoverable)
                1111b    Fatal error (system halt)

         Each Type have a number number of common Codes for all modules
         Those common codes are also defined and maintain here

         Module ID :

            ---- Applications
            0x000 : Main application

            ---- Library modules
            0x100 : OS runtime (Multi-Thread, IPC class, ObjectKernel)
            0x102 : Component class
            0x110 : File I/O class
            0x111 : Multi-processor data transfer class
            0x112 : Audio processor class
            0x113 : Video processor class
            0x114 : Mpeg processor class
            0x115 : USB Device
            0x116 : USB Host

Copyright (c) 1996-2014, I-SYST, all rights reserved

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
Modified by         Date			Description
Hoan				Nov. 18, 2014	Change module IDs
----------------------------------------------------------------------------*/
#ifndef __SYSSTATUSDEF_H__
#define __SYSSTATUSDEF_H__

#include "istddef.h"

#define SYSSTATUS_TYPE_MASK			0xf0000000
#define SYSSTATUS_MODID_MASK		0x0fff0000
#define SYSSTATUS_CODE_MASK			0x0000ffff

/**
 * System status types
 */
typedef enum _System_Status_Type {
   SYSSTATUS_TYPE_RNT   = 0,        		// Runtime status
   SYSSTATUS_TYPE_WRN   = 0x40000000U,     	// Warning
   SYSSTATUS_TYPE_ERR   = 0x80000000U,     	// Non fatal error
   SYSSTATUS_TYPE_FERR  = 0xf0000000U      	// Fatal error
} SYSSTATUS_TYPE;

/**
 * Reserve module Id here
 */

//
// Applications
//
#define SYSSTATUS_MODID_APP           	0		// Top layer application

//
// Library module
// Starts from 0x100
//
#define SYSSTATUS_MODID_OSRTL          	0x100  	// OS runtime
#define SYSSTATUS_MODID_CMPNT          	0x101  	// Component class
#define SYSSTATUS_MODID_ADAPTOR        	0x102  	// Adaptor class
#define SYSSTATUS_MODID_FILE           	0x110  	// File I/O class
#define SYSSTATUS_MODID_MPTRANS        	0x111  	// Multi-processor data transfer class
#define SYSSTATUS_MODID_AUDIO          	0x112  	// Audio processor class
#define SYSSTATUS_MODID_VIDEO          	0x113  	// Video processor class
#define SYSSTATUS_MODID_MPEG           	0x114  	// Mpeg processor class
#define SYSSTATUS_MODID_USB				0x115	// USB

/**
 * Common codes
 */
#define STATUS_OK                      0 
#define SYSSTATUS_OK                   0		// System status normal
#define SYSSTATUS_NOERROR              0
#define SYSSTATUS_SUCCESS              0
#define SYSSTATUS_READY                0

#define SYSSTATUS_IDLE             0
#define SYSSTATUS_STARTED          1
#define SYSSTATUS_STOPED           2
#define SYSSTATUS_RUNNING          3
#define SYSSTATUS_SUSPENDED        4
#define SYSSTATUS_END              5
#define SYSSTATUS_PENDING          6     // Processing pending
#define SYSSTATUS_BLOCKED          7     // Waiting for an event
#define SYSSTATUS_TIMEOUT          8     // Wait time out
#define SYSSTATUS_CANCEL           9     // Operation canceled
#define SYSSTATUS_CONNECTED        10    // Connected state
#define SYSSTATUS_DISCONNECTED     11    // Disconnected state
#define SYSSTATUS_NOTFOUND         12
#define SYSSTATUS_FAILED           13     // Operation failed
#define SYSSTATUS_NOTSUPPORTED     14     // Operation not supported
#define SYSSTATUS_OUTMEM           15     // Out of memory
#define SYSSTATUS_FILECREATE       16     // Can't create file
#define SYSSTATUS_FILEOPEN         17     // Can't open file
#define SYSSTATUS_FILEREAD         18     // Can't read from file
#define SYSSTATUS_FILEWRITE        19     // Can't write to file
#define SYSSTATUS_FILENOTFOUND     20     // File not found
#define SYSSTATUS_THREADCREATE     21     // Can't create thread
#define SYSSTATUS_INVALIDPARM      22     // Invalid parameters
#define SYSSTATUS_OUTOFRANGE       23     // Out of range
#define SYSSTATUS_NOTINIT          24     // Not initialized yet

// User defined status start
//
#define SYSSTATUS_CODE_USERSTART       0x100

typedef uint32_t   STATUS;               // Status return type


#endif // __SYSSTATUSDEF_H__
