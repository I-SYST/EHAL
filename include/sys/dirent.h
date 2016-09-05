/*--------------------------------------------------------------------------
File   : dirent.h

Author : Hoang Nguyen Hoan          Feb. 27, 2015

Desc   : Implementation of dirent for embedded

Copyright (c) 2015, I-SYST inc., all rights reserved

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

#ifndef __DIRENT_H__
#define __DIRENT_H__

#include <sys/types.h>
#include <stdint.h>

#define DT_UNKNOWN       0
#define DT_FIFO          1
#define DT_CHR           2
#define DT_DIR           4
#define DT_BLK           6
#define DT_REG           8
#define DT_LNK          10
#define DT_SOCK         12
#define DT_WHT          14

#define DA_READONLY		1
#define DA_HIDDEN		2
#define DA_SYSTEM		4

#define NAME_MAX	64
#define PATH_MAX	255
#define OPEN_MAX	3

struct dirent {
	ino_t	d_fileno;			// File serial number
	char 	d_name[NAME_MAX + 1];	// File name (full path)
	unsigned char d_namelen;
	unsigned char d_type;		// File type
	uint8_t	d_att;				// File Attribute
	uint32_t d_size;			// File size
	uint32_t d_offset;			// Current offset
	uint32_t EntrySect;
	uint32_t EntryIdx;
	uint32_t FirstClus;			// Start data cluster
};

typedef struct _DIR {
	struct dirent d_dirent;
	char *d_dirname;			// Full path name of directory
	int d_dirnamelen;
	uint32_t DirClus;			// Directory start cluster number
} DIR;

#ifdef __cplusplus
extern "C" {
#endif

int            closedir(DIR *);
DIR           *opendir(const char *);
struct dirent *readdir(DIR *);
int            readdir_r(DIR *, struct dirent *, struct dirent **);
void           rewinddir(DIR *);
void           seekdir(DIR *, long int);
long int       telldir(DIR *);


#ifdef __cplusplus
}
#endif


#endif // __DIRENT_H__
