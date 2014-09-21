/*--------------------------------------------------------------------------
File   : ledmx.cpp

Author : Hoang Nguyen Hoan          Feb. 28, 2011

Desc   : LED Matrix control for IDM-LMX3208 series display

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
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "istddef.h"
#include "ledmx.h"
#include "ledmxfont.h"

void LedMxPrintAt(LEDMXDEV *pDev, int col, const char *pStr)
{
	int panelidx = col / 32;
	int paneladdr = pDev->PanelAddr[panelidx];
	int addr = (col % 32) << 1;
	int i = 0;
	uint8_t *data;
	int w;
	int len = strlen(pStr);
                
	if (col > 0)
	{
		i = 0;
		// Clear front
		while (i < panelidx && i < pDev->NbPanel)
		{
			LedMxSetRam(pDev, 0, 0, 32, pDev->PanelAddr[i]);
			i++;
		}
		if (panelidx < pDev->NbPanel)
			LedMxSetRam(pDev, 0, 0, addr >> 1, paneladdr);
	}

	if (col >= pDev->NbPanel * 32)
		return;

	if (panelidx >= pDev->NbPanel)
		return;

	i = 0;
	do {
		uint8_t fidx = (uint8_t)pStr[i];
		LEDMXFONT_BITMAP const *font = &pDev->pFont[fidx];

#ifdef __AVR__
	    LEDMXFONT_BITMAP tfont;

        memcpy_P(&tfont, &pDev->pFont[fidx], sizeof(LEDMXFONT_BITMAP));

        font = &tfont;
#endif

		if (font->Width <= 0)
		{
			i++;
			continue;
		}

		data = font->Data;

		if (col < 0)
		{
			col += font->Width;
			if (col < 0)
			{
				// Next character still out of range
				i++;
				continue;
			}
			w = col;
			data += font->Width - col;
			addr = 0;
			panelidx = 0;
			paneladdr = pDev->PanelAddr[panelidx];
			LedMxWriteRam(pDev, 0, data, col, paneladdr);
			addr += w << 1;
			//col = 0;
			i++;
			continue;
		}
		w = min(font->Width, (64 - addr) >> 1);
		if (w > 0)
		{
			LedMxWriteRam(pDev, addr, data, w, paneladdr);
			addr += w << 1;
		}
		if (font->Width > w)
		{
			if (++panelidx >= pDev->NbPanel)
				break;
			addr = 0;
			data += w;
			w = font->Width - w;
			paneladdr = pDev->PanelAddr[panelidx];
			LedMxWriteRam(pDev, addr, data, w, paneladdr);
			addr += w << 1;
		}
		i++;
	} while (i < len);

	// Clear the rest
	if (addr < 64)
		LedMxSetRam(pDev, addr, 0, (64 - addr) >> 1, paneladdr);
	while (++panelidx < pDev->NbPanel)
	{
		LedMxSetRam(pDev, 0, 0, 32, pDev->PanelAddr[panelidx]);
	}
}

void LedMxPrintLeft(LEDMXDEV *pDev, const char *pStr)
{
	LedMxPrintAt(pDev, 0, pStr);
}

void LedMxPrintRight(LEDMXDEV *pDev, const char *pStr)
{
	int pixlen = LedMxPixStrLen(pDev, pStr);
	int col = (pDev->NbPanel * 32 - pixlen);

	LedMxPrintAt(pDev, col, pStr);
}

void LedMxPrintCenter(LEDMXDEV *pDev, const char *pStr)
{
	int pixlen = LedMxPixStrLen(pDev, pStr);
	int col = (pDev->NbPanel * 32 - pixlen) >> 1;

	LedMxPrintAt(pDev, col, pStr);
}

void LedMxPrintScrollLeft(LEDMXDEV *pDev, const char *pStr)
{
	int col = pDev->NbPanel * 32;
	int i = col;
	int pixlen;

	pixlen = LedMxPixStrLen(pDev, pStr);
	for (i = col; i + pixlen > 0; i-= 1)
	{
		LedMxPrintAt(pDev, i, pStr);
	}
}

void LedMxPrintScrollRight(LEDMXDEV *pDev, const char *pStr)
{

}

static char s_Buffer[512] = {0,};// __attribute__ ((section(".RAMAHB")));

void LedMxvPrintf(LEDMXDEV *pDev, LEDMXPRTMODE Mode, const char *pFormat, va_list vl)
{
    vsnprintf(s_Buffer, sizeof(s_Buffer), pFormat, vl);
    s_Buffer[sizeof(s_Buffer) - 1] = '\0';

    switch (Mode)
    {
    case LEDMXPRTMODE_JCENTER:
    	LedMxPrintCenter(pDev, s_Buffer);
    	break;

    case LEDMXPRTMODE_JRIGHT:
    	LedMxPrintRight(pDev, s_Buffer);
    	break;
    case LEDMXPRTMODE_SCROLLLEFT:
    	LedMxPrintScrollLeft(pDev, s_Buffer);
    	break;
    case LEDMXPRTMODE_SCROLLRIGHT:
    	LedMxPrintScrollRight(pDev, s_Buffer);
    	break;
    case LEDMXPRTMODE_JLEFT:
    default:
    	LedMxPrintLeft(pDev, s_Buffer);
    	break;
    }
}

void LedMxPrintf(LEDMXDEV *pDev, LEDMXPRTMODE Mode, const char *pFormat, ...)
{
    va_list     vArgs;

    va_start(vArgs, pFormat);
    LedMxvPrintf(pDev, Mode, pFormat, vArgs);
    va_end(vArgs);

}

void LedMxIntensity(LEDMXDEV *pDev, int Level)
{
	int i;
	int cmd = 0x940 | ((Level & 0xf) << 1);
	// 100101X-0000-X

	for (i = 0; i < pDev->NbPanel; i++)
		LedMxCmd(pDev, cmd, pDev->PanelAddr[i]);
}

void LedMxBlink(LEDMXDEV *pDev, int OnOff)
{
	int i;

	for (i = 0; i < pDev->NbPanel; i++)
		if (OnOff)
			LedMxCmd(pDev, LEDMX_CMD_BLINKON, pDev->PanelAddr[i]);
		else
			LedMxCmd(pDev, LEDMX_CMD_BLINKOFF, pDev->PanelAddr[i]);
}

int LedMxPixStrLen(LEDMXDEV *pDev, const char *pStr)
{
	unsigned int i;
	int len = 0;

	for (i = 0; i < strlen(pStr); i++)
	{
		uint8_t fidx = pStr[i];
#ifdef __AVR__
		len += pgm_read_word_near(&pDev->pFont[fidx].Width);
#else
		len += pDev->pFont[fidx].Width;
#endif
	}

	return len;
}

void LedMxCmd(LEDMXDEV *pDev, int CmdVal, int PanelAddr)
{
	LedMxStartTx(pDev, PanelAddr);
	LedMxTxData(pDev, CmdVal, 12);
	LedMxStopTx(pDev, PanelAddr);
}

void LedMxSetRam(LEDMXDEV *pDev, unsigned RamAddr, char Data, int Len, int PanelAddr)
{
	uint32_t d;
    int i;
        
	if (RamAddr >= 64) //0x80)
		return;

	while (RamAddr < 0 && Len > 0)
	{
		Len--;
		RamAddr += 2;
	}

	if (Len <= 0)
		return;

	LedMxStartTx(pDev, PanelAddr);

	// 101aaaaaaa
	d = 0x280 | (RamAddr & 0x7f);
	LedMxTxData(pDev, d, 10);

	for (i = 0; i < Len && RamAddr < 64; i++)	//0x80; i++)
	{
		LedMxTxData(pDev, Data, 8);
		RamAddr += 2;
	}

	LedMxStopTx(pDev, PanelAddr);
}

void LedMxWriteRam(LEDMXDEV *pDev, unsigned RamAddr, uint8_t const *pData, int Len, int PanelAddr)
{
	uint32_t d;
    int i;

	if (RamAddr >= 64)	//0x80)
		return;

	while (RamAddr < 0 && Len > 0)
	{
		pData++;
		Len--;
		RamAddr += 2;
	}

	if (Len <= 0)
		return;

	LedMxStartTx(pDev, PanelAddr);


	// 101aaaaaaadddddddd
	d = 0x280 | (RamAddr & 0x7f);// | *pData;
	LedMxTxData(pDev, d, 10);

	for (i = 0; i < Len && RamAddr < 64; i++) //0x80; i++)
	{
		LedMxTxData(pDev, *pData, 8);
		RamAddr += 2;
		pData++;
	}

	LedMxStopTx(pDev, PanelAddr);
}

void LedMxInit(LEDMXDEV *pDev, LEDMXCFG *pCfg)
{
	int i;
	int panelno;

	if (pDev == NULL || pCfg == NULL)
		return;

	// Initialize platform specific I/O
	LedMxIOInit(pDev, pCfg);

	pDev->NbPanel = pCfg->NbPanel;
	memcpy(pDev->PanelAddr, pCfg->PanelAddr, sizeof(pDev->PanelAddr));

	// This loop is a patch for now.  Revise this code is required
	for (i = 0; i < LEDMX_MAX_PANEL; i++) //pDev->NbPanel; i++)
	{
        LedMxCmd(pDev, LEDMX_CMD_SYSDIS, pCfg->PanelAddr[i]);
		// NOTE : 	Put all display in slave mode. By default master will generate clock
		//			it could conflict with unused display is in system
		LedMxCmd(pDev, LEDMX_CMD_SLAVE_MODE, pCfg->PanelAddr[i]);
	}

	for (i = 0; i < LEDMX_MAX_PANEL; i++) // pDev->NbPanel; i++)
	{
		panelno = pCfg->PanelAddr[i];
		LedMxCmd(pDev, LEDMX_CMD_SYSDIS, panelno);
		LedMxCmd(pDev, LEDMX_CMD_NMOS_COM8, panelno);

		if ((panelno & 3) == 0)  // Master display with switch set to 1
		//if (i == 0)
			LedMxCmd(pDev, LEDMX_CMD_RC_MASTER, panelno);
		else
		    LedMxCmd(pDev, LEDMX_CMD_SLAVE_MODE, panelno);
		LedMxCmd(pDev, LEDMX_CMD_SYSEN, panelno);
		LedMxCmd(pDev, LEDMX_CMD_BLINKOFF, panelno);
		LedMxCmd(pDev, LEDMX_CMD_LEDON, panelno);
		LedMxCmd(pDev, LEDMX_CMD_PWM_16, panelno);
	}

	if (pCfg->pFont == NULL || pCfg->FontLen <= 0)
	{
		pDev->FontLen = g_FontBitmapSize;
		pDev->pFont = g_FontBitmap;
	}
	else
	{
		pDev->FontLen = pCfg->FontLen;
		pDev->pFont = pCfg->pFont;
	}
}


