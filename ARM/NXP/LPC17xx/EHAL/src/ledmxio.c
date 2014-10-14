/*--------------------------------------------------------------------------
File   : ledmxio.cpp

Author : Hoang Nguyen Hoan          Aug. 15, 2013

Desc   : This is platform specific I/O control interface for LED Matrix Control

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
#include "ledmxio.h"

#include "LPC17xx.h"
#include "iopincfg.h"
//#include "delay.h"

typedef struct {
	LPC_GPIO_TypeDef 	*pEnPort;					// Decoder enable pin map GPIO port #
	int					EnPin;
	LPC_GPIO_TypeDef 	*pWrPort;					// WR pin map GPIO port #
	int					WrPin;
	LPC_GPIO_TypeDef 	*pRdPort;					// RD pin map GPIO port #
	int					RdPin;
	LPC_GPIO_TypeDef 	*pDataPort;					// DATA pin map GPIO port #
	int					DataPin;
	LPC_GPIO_TypeDef 	*pCsPorts[LEDMX_MAX_ADDRPIN];	// CS pins map GPIO port #s
	int					CsPins[LEDMX_MAX_ADDRPIN];
	int 				NbCsPins;				// Total number of CS pins used
	LEDMX_CSTYPE CsType;		// CS mapping type
} IODEV;

static IODEV g_LmxIODev;

void LedMxIOInit(LEDMXDEV *pLedMxDev, LEDMXCFG *pCfg)
{
	IODEV *pdev = &g_LmxIODev;
	LEDMXIOCFG *pcfg = (LEDMXIOCFG *)pCfg->pIOCfg;

	pLedMxDev->pIODev = (void *)&g_LmxIODev;

	IOPinConfig(pcfg->WrPort, pcfg->WrPin, 0, IOPIN_DIR_OUTPUT, IOPIN_RESMODE_PULLUP, IOPIN_MODE_NORMAL);
	pdev->pWrPort = &LPC_GPIO0[pcfg->WrPort];
	pdev->WrPin = 1 << pcfg->WrPin;
	pdev->pWrPort->FIODIR |= pdev->WrPin;
	pdev->pWrPort->FIOSET = pdev->WrPin;

	IOPinConfig(pcfg->RdPort, pcfg->RdPin, 0, IOPIN_DIR_OUTPUT, IOPIN_RESMODE_PULLUP, IOPIN_MODE_NORMAL);
	pdev->pRdPort = &LPC_GPIO0[pcfg->RdPort];
	pdev->RdPin = 1 << pcfg->RdPin;
	pdev->pRdPort->FIODIR |= pdev->RdPin;
	pdev->pRdPort->FIOSET = pdev->RdPin;

	IOPinConfig(pcfg->DataPort, pcfg->DataPin, 0, IOPIN_DIR_OUTPUT, IOPIN_RESMODE_NONE, IOPIN_MODE_NORMAL);
	pdev->pDataPort = &LPC_GPIO0[pcfg->DataPort];
	pdev->DataPin = 1 << pcfg->DataPin;
	pdev->pDataPort->FIODIR |= pdev->DataPin;

	IOPinConfig(pcfg->EnPort, pcfg->EnPin, 0, IOPIN_DIR_OUTPUT, IOPIN_RESMODE_PULLUP, IOPIN_MODE_NORMAL);
	pdev->pEnPort = &LPC_GPIO0[pcfg->EnPort];
	pdev->EnPin = 1 << pcfg->EnPin;
	pdev->pEnPort->FIODIR |= pdev->EnPin;
	pdev->pEnPort->FIOCLR = pdev->EnPin;

	for (int i = 0; i < LEDMX_MAX_ADDRPIN; i++)
	{
		if (pcfg->CsPorts[i] >= 0)
		{
			IOPinConfig(pcfg->CsPorts[i], pcfg->CsPins[i], 0, IOPIN_DIR_OUTPUT, IOPIN_RESMODE_PULLUP, IOPIN_MODE_NORMAL);
			pdev->pCsPorts[i] = &LPC_GPIO0[pcfg->CsPorts[i]];
			pdev->CsPins[i] = 1 << pcfg->CsPins[i];
			((LPC_GPIO_TypeDef *)(pdev->pCsPorts[i]))->FIODIR |= pdev->CsPins[i];

			if (pdev->CsType == LEDMX_CSTYPE_BIN)
				((LPC_GPIO_TypeDef *)(pdev->pCsPorts[i]))->FIOCLR = pdev->CsPins[i];
			else
				((LPC_GPIO_TypeDef *)(pdev->pCsPorts[i]))->FIOSET = pdev->CsPins[i];
		}
	}

	pdev->NbCsPins = pcfg->NbCsPins;
	pdev->CsType = pcfg->CsType;
}

void LedMxStartTx(LEDMXDEV *pLedMxDev, int PanelAddr)
{
	int i;
	IODEV *pdev = (IODEV *)pLedMxDev->pIODev;

	// Make sure all R/W stopped & CS disabled
	pdev->pRdPort->FIOSET = pdev->RdPin;
	pdev->pWrPort->FIOSET = pdev->WrPin;

	// select panel
	if (pdev->CsType == LEDMX_CSTYPE_BIN)
	{
		pdev->pEnPort->FIOCLR = pdev->EnPin;
		__NOP();

		for (i = 0; i < pdev->NbCsPins; i++)
		{
			if (pdev->CsPins[i] >= 0)
			{
				if (PanelAddr & 1)
					((LPC_GPIO_TypeDef *)(pdev->pCsPorts[i]))->FIOSET = pdev->CsPins[i];
				else
					((LPC_GPIO_TypeDef *)(pdev->pCsPorts[i]))->FIOCLR = pdev->CsPins[i];
			}
			PanelAddr >>= 1;
		}
		pdev->pEnPort->FIOSET = pdev->EnPin;
	}
	else
	{
		pdev->pEnPort->FIOSET = pdev->EnPin;
		__NOP();
		((LPC_GPIO_TypeDef *)(pdev->pCsPorts[PanelAddr]))->FIOCLR = pdev->CsPins[PanelAddr];
		pdev->pEnPort->FIOCLR = pdev->EnPin;
	}

	usDelay(40);
}

void LedMxTxData(LEDMXDEV *pLedMxDev, uint32_t Data, int NbBits)
{
	uint32_t mask = 1 << (NbBits - 1);
	IODEV *pdev = (IODEV *)pLedMxDev->pIODev;

	while (mask)
	{
		if (Data & mask)
			pdev->pDataPort->FIOSET = pdev->DataPin;	// 1
		else
			pdev->pDataPort->FIOCLR = pdev->DataPin;	// 0
		pdev->pWrPort->FIOCLR = pdev->WrPin;
		// Require 4 NOP here to stable i/o line
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		pdev->pWrPort->FIOSET = pdev->WrPin;
		// Require 2 NOP here to stable i/o line
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		mask >>= 1;
	}
}

void LedMxStopTx(LEDMXDEV *pLedMxDev, int PanelAddr)
{
	IODEV *pdev = (IODEV *)pLedMxDev->pIODev;

	pdev->pWrPort->FIOSET = pdev->WrPin;

	if (pdev->CsType == LEDMX_CSTYPE_GPIO)
	{
		for (int i = 0; i < pdev->NbCsPins; i++)
		{
			if (pdev->CsPins[i] >= 0)
				((LPC_GPIO_TypeDef *)(pdev->pCsPorts[i]))->FIOSET = pdev->CsPins[i];
		}
		pdev->pEnPort->FIOSET = pdev->EnPin;
	}
	else
		pdev->pEnPort->FIOCLR = pdev->EnPin;

	__NOP();
}




