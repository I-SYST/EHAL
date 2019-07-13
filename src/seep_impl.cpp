/*--------------------------------------------------------------------------
File   : seep_impl.cpp

Author : Hoang Nguyen Hoan          Sept. 16, 2011

Desc   : Serial EEPROM device implementation

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
#include <algorithm>
using namespace std;

#include "istddef.h"
#include "idelay.h"
#include "seep.h"
#include "iopinctrl.h"

Seep::Seep()
{
    memset(&vDevData, 0, sizeof(SEEPDEV));
}

Seep::~Seep()
{
}

bool SeepInit(SEEPDEV * const pDev, const SEEP_CFG *pCfgData, DEVINTRF * const pInterf)
{
    pDev->pInterf = pInterf;
    pDev->DevAddr = pCfgData->DevAddr;
    pDev->PageSize = pCfgData->PageSize;
    pDev->AddrLen = pCfgData->AddrLen;
    pDev->pWaitCB = pCfgData->pWaitCB;
    pDev->WrProtPin = pCfgData->WrProtPin;
    pDev->WrDelay = pCfgData->WrDelay * 1000; // convert to usec
    pDev->Size = pCfgData->Size;

    if (pCfgData->WrProtPin.PortNo >= 0 && pCfgData->WrProtPin.PinNo >= 0)
    {
        // Configure write protect pin
        IOPinCfg(&pCfgData->WrProtPin, 1);
        IOPinClear(pDev->WrProtPin.PortNo, pDev->WrProtPin.PinNo);
    }

    if (pCfgData->pInitCB)
        pCfgData->pInitCB(pCfgData->DevAddr, pInterf);

    return true;
}

int SeepRead(SEEPDEV * const pDev, uint32_t Addr, uint8_t *pData, int Len)
{
    uint8_t ad[4];
    uint8_t *p = (uint8_t*)&Addr;
    uint32_t shift = pDev->AddrLen << 3;
    uint32_t admask = 0xFFFFFFFF << shift;
    int count = 0;

    while (Len > 0 && Addr < pDev->Size)
    {
        uint8_t devaddr = pDev->DevAddr;

        // MSB first
        for (int i = 0; i < pDev->AddrLen; i++)
		{
			ad[i] = p[pDev->AddrLen - i - 1];
		}

        if (Addr & admask)
        {
        	// block select
        	devaddr |= (Addr >> shift) & 7;
        }

	    int l = min(Len, pDev->AddrLen << 8);

	    l = DeviceIntrfRead(pDev->pInterf, devaddr, ad, pDev->AddrLen, pData, Len);
	    if (l <= 0)
	    {
	    	break;
	    }
	    count += l;
	    Addr += l;
	    Len -= l;
	    pData += l;
    }

    return count;
}

// Note: Sequential write is bound by page size boundary
int SeepWrite(SEEPDEV * const pDev, uint32_t Addr, uint8_t *pData, int Len)
{
    int count = 0;
    uint8_t ad[4];
    uint8_t *p = (uint8_t*)&Addr;
    uint32_t shift = pDev->AddrLen << 3;
    uint32_t admask = 0xFFFFFFFF << shift;

    while (Len > 0 && Addr < pDev->Size)
    {
        uint8_t devaddr = pDev->DevAddr;
        int l = min(Len, pDev->PageSize - (Addr % pDev->PageSize));

        // MSB first
        for (int i = 0; i < pDev->AddrLen; i++)
        {
            ad[i] = p[pDev->AddrLen - i - 1];
        }

        if (Addr & admask)
        {
        	// block select
        	devaddr |= (Addr >> shift) & 7;
        }

        l = DeviceIntrfWrite(pDev->pInterf, devaddr, ad, pDev->AddrLen, pData, l);
        if (l <= 0)
        {
        	break;
        }
        if (pDev->pWaitCB)
        {
            pDev->pWaitCB(devaddr, pDev->pInterf);
        }
        else if (pDev->WrDelay > 0)
        {
            usDelay(pDev->WrDelay);
        }

        Addr += l;
        Len -= l;
        pData += l;
        count += l;
    }

    return count;
}

void SeepSetWriteProt(SEEPDEV * const pDev, bool bVal)
{
    if (pDev->WrProtPin.PortNo < 0 || pDev->WrProtPin.PinNo < 0)
        return;

    if (bVal)
        IOPinSet(pDev->WrProtPin.PortNo, pDev->WrProtPin.PinNo);
    else
        IOPinClear(pDev->WrProtPin.PortNo, pDev->WrProtPin.PinNo);
}
