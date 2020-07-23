/**-------------------------------------------------------------------------
@file	i2s_nrfx.cpp

@brief	I2S (Inter-IC sound interface) implementation on Nordic nRF series


@author	Nguyen Hoan Hoang
@date	Apr. 4, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "nrf.h"
#include "nrf_peripherals.h"

#include "istddef.h"
#include "idelay.h"
#include "coredev/i2s.h"

#ifndef NRF52_SERIES
#define NRF_I2S			NRF_I2S_S
#define NRF_I2S_BASE	NRF_I2S_S_BASE
#endif

#pragma pack(push, 4)
typedef struct {
	uint32_t Freq;		//!< Freq in Hz
	uint32_t RegVal;	//!< Register value
} MCLK_FREQ;
#pragma pack(pop)

static const MCLK_FREQ s_MClkFreqTable[] = {
	{256000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV125},	// 256 KHz
	{507937, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63},	// 0.5079365 MHz
	{761905, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV42},	// 0.7619048 MHz
	{1000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV32},	// 1.0 MHz
	{1032258, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV31},	// 1.0322581 MHz
	{1066667, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV30},	// 1.0666667 MHz
	{1391304, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV23},	// 1.3913043 MHz
	{1523810, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21},	// 1.5238095 MHz
	{2000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV16},	// 2.0 MHz
	{2133333, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV15},	// 2.1333333 MHz
	{2909091, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11},	// 2.9090909 MHz
	{3200000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV10},	// 3.2 MHz
	{4000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8},	// 4.0 MHz
#if !defined(NRF52_SERIES)
	{5333333, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV6},	// 5.3333333 MHz
	{6400000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV5},	// 6.4 MHz
	{8000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4},	// 8.0 MHz
	{10666667, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV3},	// 10.6666667 MHz
	{16000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV2},	// 16.0 MHz
#endif
};

static const int s_NbFreq = sizeof(s_MClkFreqTable) / sizeof(MCLK_FREQ);

static const uint32_t s_Ratio[] = {
	32, 48, 64, 96, 128, 192, 256, 384, 512
};

static const int s_NbRatio = sizeof(s_Ratio) / sizeof(uint32_t);

static I2SDEV *s_pI2SDev = NULL;

#define I2S_BUFF_MAX		4

alignas(4) static uint32_t s_I2S_RxBuffer[I2S_BUFF_MAX];
alignas(4) static uint32_t s_I2S_TxBuffer[I2S_BUFF_MAX] = { 0xA500A4, 0xA600a7, };

static int s_RxBuffLen = 0;
static int s_TxBuffLen = 0;

bool nRFxI2SWaitStop(I2SDEV * const pDev, int32_t Timeout)
{
	do {
		if (NRF_I2S->EVENTS_STOPPED)
		{
			NRF_I2S->EVENTS_STOPPED = 0;

			return true;
		}
	} while(Timeout-- > 0);

	return false;
}

bool nRFxI2SWaitRx(I2SDEV * const pDev, int32_t Timeout)
{
	do {
		if (NRF_I2S->EVENTS_RXPTRUPD)
		{
			NRF_I2S->EVENTS_RXPTRUPD = 0;

			return true;
		}
	} while(Timeout-- > 0);

	return false;
}

bool nRFxI2SWaitTx(I2SDEV * const pDev, int32_t Timeout)
{
	do {
		if (NRF_I2S->EVENTS_TXPTRUPD)
		{
			NRF_I2S->EVENTS_TXPTRUPD = 0;

			return true;
		}
	} while(Timeout-- > 0);

	return false;
}

static void nRFxI2SDisable(DEVINTRF * const pDev)
{
	NRF_I2S->TASKS_STOP = 1;
	NRF_I2S->ENABLE = 0;
}

static void nRFxI2SEnable(DEVINTRF * const pDev)
{
	I2SDEV *dev = (I2SDEV*)pDev->pDevData;

	if (NRF_I2S->PSEL.SDIN != -1)
	{
		NRF_I2S->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Msk;
	}
	if (NRF_I2S->PSEL.SDOUT != -1)
	{
		NRF_I2S->CONFIG.TXEN = I2S_CONFIG_RXEN_RXEN_Msk;
	}
	NRF_I2S->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Msk;
	NRF_I2S->ENABLE = I2S_ENABLE_ENABLE_Msk;
	NRF_I2S->TASKS_START = 1;
}

static void nRFxI2SPowerOff(DEVINTRF * const pDev)
{
	*(volatile uint32_t *)((uint32_t)NRF_I2S_BASE + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_I2S_BASE + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)NRF_I2S_BASE + 0xFFC) = 0;
}

static uint32_t nRFxI2SGetRate(DEVINTRF * const pDev)
{
	I2SDEV *dev = (I2SDEV*)pDev->pDevData;

	return dev->Freq;
}

static uint32_t nRFxI2SSetRate(DEVINTRF * const pDev, uint32_t Rate)
{
	I2SDEV *dev = (I2SDEV*)pDev->pDevData;
	uint32_t diff = -1;
	int idx = 0;
	int ratio = 0;
	uint32_t w = dev->SampleWidth << 4;

	for (int i = 0; i < s_NbFreq; i++)
	{
		uint32_t r = s_MClkFreqTable[i].Freq / Rate;

		// Ratio must be multiple of 2 * Width
		if (r > w)
		{
			r = ((r + (w >>1)) / w) * w;
		}
		else
		{
			r = w;
		}

		// Recalc real frequency
		uint32_t f = s_MClkFreqTable[i].Freq / r;

		uint d = (f > Rate) ? f - Rate : Rate - f;
		if (d < diff)
		{
			idx = i;
			diff = d;
			ratio = r;
		}
	}

	dev->MClkFreq = s_MClkFreqTable[idx].Freq;
	NRF_I2S->CONFIG.MCKFREQ = s_MClkFreqTable[idx].RegVal;

	for (int i = 0; i < s_NbRatio; i++)
	{
		if (s_Ratio[i] == ratio)
		{
			NRF_I2S->CONFIG.RATIO = i;
			break;
		}
	}

	dev->Freq = dev->MClkFreq / ratio;

	return dev->Freq;
}

bool nRFxI2SStartRx(DEVINTRF * const pDev, uint32_t DevAddr)
{
	return true;
}

int nRFxI2SRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	I2SDEV *dev = (I2SDEV*)pDev->pDevData;


	//NRF_I2S->RXD.PTR = (uint32_t)pBuff;
	//NRF_I2S->RXTXD.MAXCNT = BuffLen;
	memcpy(pBuff, s_I2S_RxBuffer, min(BuffLen, I2S_BUFF_MAX * 4));
//	NRF_I2S->CONFIG.RXEN = 1;
//	NRF_I2S->TASKS_START = 1;
	nRFxI2SWaitRx(dev, 10000);

	return 0;
}

void nRFxI2SStopRx(DEVINTRF * const pDev)
{
}

bool nRFxI2SStartTx(DEVINTRF * const pDev, uint32_t DevAddr)
{
	return true;
}

int nRFxI2STxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{

	return 0;
}

void nRFxI2SStopTx(DEVINTRF * const pDev)
{
}

void nRFxI2SReset(DEVINTRF * const pDev)
{
	nRFxI2SDisable(pDev);
	msDelay(1);
	nRFxI2SDisable(pDev);
}

/**
 * @brief	Initialize I2S hardware interface
 *
 * This is a required implementation.
 *
 * @param	pDev : Pointer to device private data to be initialized
 * @param	pCfg : Pointer to configuration data
 *
 * @return	true - Initialization success
 */
bool I2SInit(I2SDEV * const pDev, const I2SCFG * const pCfgData)
{
	if (pDev == nullptr || pCfgData == nullptr)
	{
		return false;
	}

	s_pI2SDev = pDev;

	pDev->DevIntrf.pDevData = pDev;

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)NRF_I2S_BASE + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_I2S_BASE + 0xFFC) = 1;

	if (pCfgData->pRxFifoMem)
	{

	}

	pDev->Mode = pCfgData->Mode;
	pDev->Format = pCfgData->Format;
	pDev->SampleWidth = pCfgData->SampleWidth / 8;
	pDev->Chan = pCfgData->Chan;
	pDev->pIOPinMap = pCfgData->pIOPinMap;
	pDev->NbIOPins = pCfgData->NbIOPins;

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	NRF_I2S->PSEL.SCK = pDev->pIOPinMap[I2S_SCK_IOPIN_IDX].PinNo;
	NRF_I2S->PSEL.LRCK = pDev->pIOPinMap[I2S_WS_IOPIN_IDX].PinNo;
	NRF_I2S->PSEL.SDIN = pDev->pIOPinMap[I2S_SDI_IOPIN_IDX].PinNo;
	NRF_I2S->PSEL.SDOUT = -1;
	NRF_I2S->PSEL.MCK = -1;

	if (pDev->NbIOPins > 3)
	{
		NRF_I2S->PSEL.SDOUT = pDev->pIOPinMap[I2S_SDO_IOPIN_IDX].PinNo;
	}

	if (pDev->NbIOPins > 4)
	{
		NRF_I2S->PSEL.MCK = pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PinNo;
	}

	if (pCfgData->Mode == I2SMODE_SLAVE)
	{
		NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Slave;
	}
	else
	{
		NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Master;
	}

	NRF_I2S->CONFIG.FORMAT = 0;
	NRF_I2S->CONFIG.SWIDTH = pDev->SampleWidth - 1;

	if (pCfgData->Format != I2SFMT_NORMAL)
	{
		NRF_I2S->CONFIG.FORMAT = 1;
		NRF_I2S->CONFIG.ALIGN = pCfgData->Format == I2SFMT_ALIGN_RIGHT ? 1 : 0;
	}

	nRFxI2SSetRate(&pDev->DevIntrf, pCfgData->Freq);


	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2S;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.Disable = nRFxI2SDisable;
	pDev->DevIntrf.Enable = nRFxI2SEnable;
	pDev->DevIntrf.PowerOff = nRFxI2SPowerOff;
	pDev->DevIntrf.GetRate = nRFxI2SGetRate;
	pDev->DevIntrf.SetRate = nRFxI2SSetRate;
	pDev->DevIntrf.StartRx = nRFxI2SStartRx;
	pDev->DevIntrf.StopRx = nRFxI2SStopRx;
	pDev->DevIntrf.StartTx = nRFxI2SStartTx;
	pDev->DevIntrf.RxData = nRFxI2SRxData;
	pDev->DevIntrf.TxData = nRFxI2STxData;
	pDev->DevIntrf.StopTx = nRFxI2SStopTx;
	pDev->DevIntrf.Reset = nRFxI2SReset;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	NRF_I2S->RXD.PTR = (uint32_t)s_I2S_RxBuffer;
	NRF_I2S->TXD.PTR = (uint32_t)s_I2S_TxBuffer;
	NRF_I2S->RXTXD.MAXCNT = ((pDev->SampleWidth << 1) >> 2) + 1;//I2S_BUFF_MAX;	// count in 32 bits word
	NRF_I2S->INTEN = 0;
	NRF_I2S->EVENTS_RXPTRUPD = 0;
	NRF_I2S->EVENTS_STOPPED = 0;
	NRF_I2S->EVENTS_TXPTRUPD = 0;

	if (pCfgData->bIntEn)
	{
        NRF_I2S->INTEN = I2S_INTEN_RXPTRUPD_Msk | I2S_INTEN_TXPTRUPD_Msk | I2S_INTEN_STOPPED_Msk;

        NVIC_ClearPendingIRQ(I2S_IRQn);
        NVIC_SetPriority(I2S_IRQn, pCfgData->IntPrio);
        NVIC_EnableIRQ(I2S_IRQn);
	}

	nRFxI2SEnable(&pDev->DevIntrf);

	return true;
}

extern "C" void I2S_IRQHandler()
{
	int len = s_pI2SDev->SampleWidth << 1;

	if (NRF_I2S->EVENTS_TXPTRUPD)
	{
		if (s_pI2SDev->hTxFifo)
		{
			uint8_t *p = CFifoGet(s_pI2SDev->hTxFifo);
			if (p)
			{
				memcpy(s_I2S_TxBuffer, p, len);
			}
			else
			{
				memset(s_I2S_TxBuffer, 0, len);
			}
		}
	}

	if (NRF_I2S->EVENTS_RXPTRUPD)
	{
		if (s_pI2SDev->hRxFifo)
		{
			uint8_t *p = CFifoPut(s_pI2SDev->hRxFifo);
			if (p)
			{
				memcpy(p, s_I2S_RxBuffer, len);
			}
		}

		if (s_pI2SDev->DevIntrf.EvtCB)
		{
			s_pI2SDev->DevIntrf.EvtCB(&s_pI2SDev->DevIntrf, DEVINTRF_EVT_RX_DATA, (uint8_t*)s_I2S_RxBuffer, len);
		}
	}

	if (NRF_I2S->EVENTS_STOPPED)
	{

	}
}
