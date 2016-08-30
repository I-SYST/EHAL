/*--------------------------------------------------------------------------
File   : uart.c

Author : Hoang Nguyen Hoan          Sept. 16, 2011

Desc   : Generic uart implementation

Copyright (c) 2011, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

//#include "istddef.h"
#include "stddev.h"
#include "uart.h"

extern char s_Buffer[];	// defined in sbuffer.c
extern int s_BufferSize;

//#define UARTPRINTF_BUFFMAX		128
//char s_Buffer[UARTPRINTF_BUFFMAX];	// defined in sbuffer.c
//int s_BufferSize = UARTPRINTF_BUFFMAX;
//char g_UARTPrintfBuffer[UARTPRINTF_BUFFMAX] = {0,};

int UARTStdDevOpen(void *pDevObj, const char *pDevName, int Flags, int Mode);
// close
int UARTStdDevClose(void *pDevObj, int Handle);
// Read/Write
int UARTStdDevRead(void *pDevObj, int Handle, uint8_t *pBuff, size_t Len);
int UARTStdDevWrite(void *pDevObj, int Handle, uint8_t *pBuff, size_t Len);
// seek


STDDEV g_UartStdDev = {
	"UARTIO",
	NULL,
	UARTStdDevOpen,
	UARTStdDevClose,
	UARTStdDevRead,
	UARTStdDevWrite,
	NULL
};

void UARTRetargetEnable(UARTDEV *pDev, int FileNo)
{
	if (g_UartStdDev.pDevObj == NULL)
		g_UartStdDev.pDevObj = pDev;

	switch (FileNo)
	{
		case STDIN_FILENO:
			// Enable input
			if (pDev->hStdIn >= 0)
				RemoveBlkDev(pDev->hStdIn);

			pDev->hStdIn = InstallBlkDev(&g_UartStdDev, STDIN_FILENO);
			break;
		case STDOUT_FILENO:
		case STDERR_FILENO:
			// Enable output
			if (pDev->hStdIn >= 0)
				RemoveBlkDev(pDev->hStdOut);

			pDev->hStdOut = InstallBlkDev(&g_UartStdDev, STDOUT_FILENO);
			break;
	}
}

void UARTRetargetDisable(UARTDEV *pDev, int FileNo)
{
	switch (FileNo)
	{
		case STDIN_FILENO:
			// Disable input
			RemoveBlkDev(pDev->hStdIn);
			pDev->hStdIn = -1;
			break;
		case STDOUT_FILENO:
		case STDERR_FILENO:
			// Disable output
			RemoveBlkDev(pDev->hStdOut);
			pDev->hStdOut = -1;
			break;
	}
}

int UARTStdDevOpen(void *pDevObj, const char *pDevName, int Flags, int Mode)
{
	return 0;
}

int UARTStdDevClose(void *pDevObj, int Handle)
{
	return 0;
}

int UARTStdDevRead(void *pDevObj, int Handle, uint8_t *pBuff, size_t Len)
{
	UARTDEV *dev = (UARTDEV *)pDevObj;
	int l = 0;

	if (Handle == dev->hStdIn)
	{
		while (l < Len)
		{
			int cnt = UARTRx(dev, pBuff, Len);
			if (cnt > 0)
			{
				l++;
				if (*pBuff++ == '\r')
					break;
			}
			else if (l > 0)
				break;
		}
	}

	if (l == 0)
		l = EOF;
	return l;
}

int UARTStdDevWrite(void *pDevObj, int Handle, uint8_t *pBuff, size_t Len)
{
	UARTDEV *dev = (UARTDEV *)pDevObj;

	if (Handle == dev->hStdOut)
	{
		return UARTTx(dev, pBuff, Len);
	}

	return 0;
}

