/*--------------------------------------------------------------------------
File   : ledmx.h

Author : Hoang Nguyen Hoan          Feb. 28, 2011

Desc   : LED Matrix control for IDM-LMX3208 series display

This module requires each platform to implement the following :

A data structure to define I/O pin configurations, this structure must be
defined in the ledmxio.h

struct _LedMxIOCfg {
...
};

4 functions implementation for I/O control

void LedMxIOInit(LEDMXDEV *pLedMxDev, LEDMXCFG *pCfg);
void LedMxStartTx(LEDMXDEV *pDev, int PanelAddr);
void LedMxStopTx(LEDMXDEV *pDev);
void LedMxTxData(LEDMXDEV *pDev, uint32_t Data, int NbBits);


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
Modified by         Date         	Description
Hoan				Sep. 14, 2014	Add support for IBB-LMXBLUE with 16 displays
----------------------------------------------------------------------------*/
#ifndef __LEDMX_H__
#define __LEDMX_H__

#include <stdarg.h>
#include <string.h>

#include "ledmxfont.h"

#define LEDMX_CMD_SYSDIS			0x800
#define LEDMX_CMD_SYSEN				0x802
#define LEDMX_CMD_LEDOFF			0x804
#define LEDMX_CMD_LEDON				0x806
#define LEDMX_CMD_BLINKOFF			0x810
#define LEDMX_CMD_BLINKON			0x812
#define LEDMX_CMD_RC_MASTER			0x830
#define LEDMX_CMD_EXCLK_MASTER		0x838
#define LEDMX_CMD_SLAVE_MODE		0x820
#define LEDMX_CMD_NMOS_COM8			0x840
#define LEDMX_CMD_PMOS_COM8			0x850
#define LEDMX_CMD_PWM_16			0x95e


#define LEDMX_MAX_PANEL			16

typedef enum {
	LEDMXPRTMODE_JLEFT,
	LEDMXPRTMODE_JCENTER,
	LEDMXPRTMODE_JRIGHT,
	LEDMXPRTMODE_SCROLLLEFT,
	LEDMXPRTMODE_SCROLLRIGHT
} LEDMXPRTMODE;

// CS mapping type
typedef enum {
	LEDMX_CSTYPE_GPIO, 		// Direct map to GPIO pin
	LEDMX_CSTYPE_BIN,		// Through a binary decoder, i.e. using 74AHCT138 style
    LEDMX_CSTYPE_SER		// Through serial shift register
} LEDMX_CSTYPE;

#pragma pack(push, 4)

// LED matrix data
typedef struct {
	void *pIOCfg;	// Pointer to private I/O config data (platform dependent)
    int NbPanel;
    int PanelAddr[LEDMX_MAX_PANEL];
    int FontLen;
    LEDMXFONT_BITMAP const *pFont;
} LEDMXCFG;

typedef struct {
	int	NbPanel;		// Max number of panels installed
	int PanelAddr[LEDMX_MAX_PANEL];	//
    int FontLen;
    LEDMXFONT_BITMAP const *pFont;
    void *pIODev;		// Pointer to platform specific I/O control
} LEDMXDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void LedMxBlink(LEDMXDEV *pDev, int OnOff);
void LedMxCmd(LEDMXDEV *pDev, int CmdVal, int DevNo);
void LedMxInit(LEDMXDEV *pDev, LEDMXCFG *pCfg);
void LedMxIntensity(LEDMXDEV *pDev, int Level);
int LedMxPixStrLen(LEDMXDEV *pDev, const char *pStr);
void LedMxPrintAt(LEDMXDEV *pDev, int col, const char *pStr);
void LedMxPrintCenter(LEDMXDEV *pDev, const char *pStr);
void LedMxPrintf(LEDMXDEV *pDev, LEDMXPRTMODE Mode, const char *pFormat, ...);
void LedMxvPrintf(LEDMXDEV *pDev, LEDMXPRTMODE Mode, const char *pFormat, va_list vl);
void LedMxPrintLeft(LEDMXDEV *pDev, const char *pStr);
void LedMxPrintRight(LEDMXDEV *pDev, const char *pStr);
void LedMxPrintScrollLeft(LEDMXDEV *pDev, const char *pStr);
void LedMxPrintScrollRight(LEDMXDEV *pDev, const char *pStr);
void LedMxSetRam(LEDMXDEV *pDev, unsigned RamAddr, char Data, int Len, int PanelNo);
void LedMxWriteRam(LEDMXDEV *pDev, unsigned Addr, uint8_t const *pData, int Len, int DevNo);

// Private platform dependent impletmentation
void LedMxIOInit(LEDMXDEV *pLedMxDev, LEDMXCFG *pCfg);
void LedMxStartTx(LEDMXDEV *pDev, int PanelAddr);
void LedMxStopTx(LEDMXDEV *pDev, int PanelAddr);
void LedMxTxData(LEDMXDEV *pDev, uint32_t Data, int NbBits);

#ifdef __cplusplus
}

class LedMx {
public:
	LedMx() {
		memset(&vDevData, 0, sizeof(LEDMXDEV));
		vDevData.FontLen = g_FontBitmapSize;
		vDevData.pFont = (LEDMXFONT_BITMAP*)g_FontBitmap;
	}
	virtual ~LedMx() {}
	LedMx(LedMx&);	// copy ctor not allowed

	void Init(LEDMXCFG &Cfg) { LedMxInit(&vDevData, &Cfg); }
	void Cmd(int CmdVal, int PanelAddr) { LedMxCmd(&vDevData, CmdVal, PanelAddr); }
	void PrintAt(int col, const char *pStr) { LedMxPrintAt(&vDevData, col, pStr); }
	void PrintLeft(const char *pStr) { LedMxPrintLeft(&vDevData, pStr); }
	void PrintRight(const char *pStr) { LedMxPrintRight(&vDevData, pStr); }
	void PrintCenter(const char *pStr) { LedMxPrintCenter(&vDevData, pStr); }
	void Printf(LEDMXPRTMODE Mode, const char *pFormat, ...) {
		va_list     vArgs;
		va_start(vArgs, pFormat);
		vPrintf(Mode, pFormat, vArgs);
		va_end(vArgs);
	}
	void vPrintf(LEDMXPRTMODE Mode, const char *pFormat, va_list vl) {
		LedMxvPrintf(&vDevData, Mode, pFormat, vl); }

	void Blink(int OnOff) { LedMxBlink(&vDevData, OnOff); }
	void Intensity(int Level) { LedMxIntensity(&vDevData, Level); }
	void WriteRam(unsigned RamAddr, uint8_t const *pData, int Len, int PanelAddr) {
		LedMxWriteRam(&vDevData, RamAddr, pData, Len, PanelAddr); }
	void SetRam(unsigned RamAddr, char Data, int Len, int PanelAddr) {
			LedMxSetRam(&vDevData, RamAddr, Data, Len, PanelAddr); }
	int PixStrLen(const char *pStr) { return LedMxPixStrLen(&vDevData, pStr); }
	int GetNbCol(void) { return  vDevData.NbPanel * 32; }
	operator LEDMXDEV * () { return &vDevData; }

protected:
	void StartTx(int PanelAddr) { LedMxStartTx(&vDevData, PanelAddr); }
	void StopTx(int PanelAddr) { LedMxStopTx(&vDevData, PanelAddr); }
	void TxData(uint32_t Data, int NbBits) { LedMxTxData(&vDevData, Data, NbBits); }

private:
	LEDMXDEV	vDevData;
};

#endif

#endif	// __LEDMX_H__


