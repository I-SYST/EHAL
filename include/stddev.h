/*--------------------------------------------------------------------------
File   : stddev.h

Author : Hoang Nguyen Hoan          Mar. 8, 2015

Desc   : Implementation of stdio device remapping

Copyright (c) 2015, I-SYST, all rights reserved

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
#ifndef __STDDEV_H__
#define __STDDEV_H__

#include <stdint.h>
#include <unistd.h>

#define STDDEV_MAX				6		// Max number of standard device
#define STDDEV_NAME_MAX			8

#define STDFS_FILENO			3		// Default File system
#define STDDEV_USER_FILENO		4		// Start of user device fileno idx

// open
typedef int (*STDDEVOPEN)(void *pDevObj, const char *pDevName, int Flags, int Mode);
// close
typedef int (*STDDEVCLOSE)(void *pDevObj, int Handle);
// Read/Write
typedef int (*STDDEVRW)(void *pDevObj, int Handle, uint8_t *pBuff, size_t Len);
// seek
typedef int (*STDDEVSEEK)(void *pDevObj, int Handle, int Offset);

#pragma pack(push, 4)

typedef struct {
	char 		Name[STDDEV_NAME_MAX];	// Device name
	void 		*pDevObj;	// Device object
	STDDEVOPEN	Open;		// Pointer to Open function
	STDDEVCLOSE Close;		// Pointer to Close function
	STDDEVRW	Read;		// Pointer to Read function
	STDDEVRW	Write;		// Pointer to Write function
	STDDEVSEEK	Seek;		// Pointer to Seek function
} STDDEV;

#pragma pack(pop)


#ifdef __cplusplus
extern "C" {
#endif

/*
 * Install block device into stdio syscall
 *
 * @param	pDev 	: Pointer to standard device descriptor structure
 * 			MapIp	: Mapping id
 * 						STDIN_FILENO 	- to replace stdin
 * 						STDOUT_FILENO 	- to replace stdout
 * 						STDERR_FILENO	- to replace stderr
 * 						STDDEV_FATFS_FILENO - for stdio.h filesystem
 * 						STDEV_USER_FILENO - Start of user defined
 *
 * 	@return	Handle : if succeeded
 * 			- 1 : Failed
 */
int InstallBlkDev(STDDEV *pDev, int MapId);

/*
 * Remove block device from stdio syscall
 *
 * @param	Handle : Handle of device previously installed
 */
void RemoveBlkDev(int Handle);

#ifdef __cplusplus
}
#endif

#endif	// __STDDEV_H__

