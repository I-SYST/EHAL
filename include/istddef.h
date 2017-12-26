/**-------------------------------------------------------------------------
@file	istddef.h

@brief	Standard generic defines.

Contains software version data structure and application specific data.\n
Mostly for compatibilities.

@author Hoang Nguyen Hoan
@date	Jan. 16, 2012

@license

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

----------------------------------------------------------------------------*/

#ifndef __ISTDDEF_H__
#define __ISTDDEF_H__

#ifndef __cplusplus
#include <stdbool.h>
#endif // __cplusplus

#include <stdint.h>

/** @addtogroup Utilities
  * @{
  */

#ifdef _MSC_VER
// Microsoft does not support C99 inline
#ifndef inline
#define inline __forceinline
#endif
#endif

/// only for backward compatibility, otherwise useless.
#ifndef Bool
typedef bool	Bool;
#endif

/// only for backward compatibility, otherwise useless.
#ifndef FALSE
#define FALSE		false
#endif
/// only for backward compatibility, otherwise useless.
#ifndef TRUE
#define TRUE		true
#endif

#pragma pack(push, 1)

#define ISYST_BLUETOOTH_ID			0x0177	//!< I-SYST Bluetooth company identifier

///
/// Structure defining software version.
///
/// Version number MM.mm.ssss.bbbbbbbb\n
/// Where MM = Major, mm = minor, ssss = Subversion, bbbbbbbb = Build number
typedef struct {
	union {
		uint16_t	Vers;   	//!< Verion number 0xMMmm, MM = Major, mm = minor (MM.mm)
		struct {
			unsigned Minor:8;	//!< Version major
			unsigned Major:8;	//!< Version minor
		};
	};
	uint16_t SubVers;		//!< Subversion
	uint32_t Build;			//!< Build number
} VERS;
#pragma pack(pop)

#define APPINFO_NAMESIZE_MAX		16		//!< Max size in bytes for application name
#define APPINFO_PRIVATESIZE_MAX		16		//!< Max size in bytes for private data

#pragma pack(push, 4)

///
/// Structure defining Application data.
///
/// It contains application identifier (Name), version and application specific private data
/// This data is usually static const located at specific location where bootloader/dfu can
/// access to validate.
typedef struct {
	char Name[APPINFO_NAMESIZE_MAX];			//!< Application signature
	VERS Vers;									//!< Version number
	uint8_t Private[APPINFO_PRIVATESIZE_MAX];	//!< APPINFO_PRIVATESIZE_MAX bytes private data
} APP_INFO;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

#ifndef min

/// An inline min function when there isn't one available.
static inline int min(int x, int y) { return x > y ? y : x; }
#endif

// max function
#ifndef max
/// An inline max function when there isn't one available.
static inline int max(int x, int y) { return x > y ? x : y; }
#endif

#ifdef __cplusplus
}
#endif

/** @} End of group Utilities */

#endif // __ISTDDEF_H__

