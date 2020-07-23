/**-------------------------------------------------------------------------
@file	spi_nrfx.cpp

@brief	SPI implementation on nRFx series MCU

Note: SPI device are shared with other device such as I2C therefore be careful
not to use the same device number on an other device.

For 3 wire support : Set both MISO & MOSI to the same pin.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

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
#include "nrf.h"

#include "istddef.h"
#include "coredev/spi.h"
#include "iopinctrl.h"
#include "coredev/shared_irq.h"

#include "nrf_peripherals.h"

#ifdef SPIM_PRESENT
#define NRFX_SPI_MAXDEV			SPIM_COUNT
#else
#define NRFX_SPI_MAXDEV			SPI_COUNT
#endif

#define NRFX_SPISLAVE_MAXDEV	SPIS_COUNT
#define NRFX_SPI_DMA_MAXCNT		((1<<SPIS0_EASYDMA_MAXCNT_SIZE)-1)

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	union {
#ifdef SPI_PRESENT
		NRF_SPI_Type  *pReg;	// Master I/O register map
#endif
#ifdef SPIM_PRESENT
		NRF_SPIM_Type *pDmaReg;	// Master DMA register map
#endif
#ifdef SPIS_COUNT
		NRF_SPIS_Type *pDmaSReg;// Slave DMA register map
#endif
	};
} NRFX_SPIDEV;

typedef struct {
	uint32_t Freq;
	uint32_t RegVal;
} NRFX_SPIFREQ;

#pragma pack(pop)

static const NRFX_SPIFREQ s_nRFxSPIFreq[] = {
#ifdef SPIM_PRESENT
		{125000, SPIM_FREQUENCY_FREQUENCY_K125},
		{250000, SPIM_FREQUENCY_FREQUENCY_K250},
		{500000, SPIM_FREQUENCY_FREQUENCY_K500},
		{1000000, SPIM_FREQUENCY_FREQUENCY_M1},
		{2000000, SPIM_FREQUENCY_FREQUENCY_M2},
		{4000000, SPIM_FREQUENCY_FREQUENCY_M4},
		{8000000, SPIM_FREQUENCY_FREQUENCY_M8},
#else
		{125000, SPI_FREQUENCY_FREQUENCY_K125},
		{250000, SPI_FREQUENCY_FREQUENCY_K250},
		{500000, SPI_FREQUENCY_FREQUENCY_K500},
		{1000000, SPI_FREQUENCY_FREQUENCY_M1},
		{2000000, SPI_FREQUENCY_FREQUENCY_M2},
		{4000000, SPI_FREQUENCY_FREQUENCY_M4},
		{8000000, SPI_FREQUENCY_FREQUENCY_M8},
#endif
#ifdef NRF52840_XXAA
		{16000000, SPIM_FREQUENCY_FREQUENCY_M16},
		{32000000, SPIM_FREQUENCY_FREQUENCY_M32},
#endif
};

static const int g_NbSPIFreq = sizeof(s_nRFxSPIFreq) / sizeof(NRFX_SPIFREQ);

static NRFX_SPIDEV s_nRFxSPIDev[NRFX_SPI_MAXDEV] = {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	{
		0, NULL, (NRF_SPIM_Type*)NRF_SPIM0_NS_BASE
	},
#else
	{
		0, NULL, (NRF_SPIM_Type*)NRF_SPIM0_S_BASE
	},
	{
		1, NULL, (NRF_SPIM_Type*)NRF_SPIM1_S_BASE
	},
	{
		2, NULL, (NRF_SPIM_Type*)NRF_SPIM2_S_BASE
	},
	{
		3, NULL, (NRF_SPIM_Type*)NRF_SPIM3_S_BASE
	},
#endif
#else
	{
		0, NULL, (NRF_SPI_Type*)NRF_SPI0_BASE
	},
	{
		1, NULL, (NRF_SPI_Type*)NRF_SPI1_BASE
	},
#if NRFX_SPI_MAXDEV > 2
	{
		2, NULL, {.pDmaReg = (NRF_SPIM_Type*)NRF_SPIM2_BASE}
	},
#if NRFX_SPI_MAXDEV > 3
	{
		3, NULL, {.pDmaReg = (NRF_SPIM_Type*)NRF_SPIM3_BASE}
	},
#endif
#endif
#endif
};

#ifdef SPIM_PRESENT
bool nRFxSPIWaitDMA(NRFX_SPIDEV * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pDmaReg->EVENTS_END)
		{
			pDev->pDmaReg->EVENTS_END = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}
#endif

#ifdef SPI_PRESENT
bool nRFxSPIWaitReady(NRFX_SPIDEV * const pDev, uint32_t Timeout)
{
	do {
        if (pDev->pReg->EVENTS_READY)
        {
            pDev->pReg->EVENTS_READY = 0; // clear event
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}
#endif

#ifdef SPIM_PRESENT
bool nRFxSPIWaitRX(NRFX_SPIDEV * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pDmaReg->EVENTS_ENDRX)
		{
			pDev->pDmaReg->EVENTS_ENDRX = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}
#endif

uint32_t nRFxSPIGetRate(DEVINTRF * const pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((NRFX_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
uint32_t nRFxSPISetRate(DEVINTRF * const pDev, uint32_t Rate)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev->pDevData;
	uint32_t regval = 0;


	for (int i = 0; i < g_NbSPIFreq; i++)
	{
		if (s_nRFxSPIFreq[i].Freq <= Rate)
		{
			regval =  s_nRFxSPIFreq[i].RegVal;
			dev->pSpiDev->Cfg.Rate = s_nRFxSPIFreq[i].Freq;
		}
	}

#ifdef SPI_PRESENT
	dev->pReg->FREQUENCY = regval;
#else
	dev->pDmaReg->FREQUENCY = regval;
#endif

	return dev->pSpiDev->Cfg.Rate;
}

void nRFxSPIDisable(DEVINTRF * const pDev)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		dev->pDmaSReg->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
	}
#ifdef SPIM_PRESENT
	else if (pDev->bDma)
	{
		dev->pDmaReg->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
	}
#endif
#ifdef SPI_PRESENT
	else
	{
        dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
	}
#endif
}

void nRFxSPIEnable(DEVINTRF * const pDev)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		dev->pDmaSReg->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
	}
#ifdef SPIM_PRESENT
	else if (pDev->bDma)
	{
		dev->pDmaReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
	}
#endif
#ifdef SPI_PRESENT
	else
	{
        dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
	}
#endif
}

void nRFxSPIPowerOff(DEVINTRF * const pDev)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev->pDevData;

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
#ifdef SPIM_PRESENT
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pDmaReg + 0xFFC) = 0;
#else
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pReg + 0xFFC) = 0;
#endif

	if (dev->pSpiDev->Cfg.NbIOPins > 3)
	{
		for (int i = 0; i < dev->pSpiDev->Cfg.NbIOPins; i++)
		{
			IOPinDisable(dev->pSpiDev->Cfg.pIOPinMap[i].PortNo, dev->pSpiDev->Cfg.pIOPinMap[i].PinNo);
		}
	}
}

// Initial receive
bool nRFxSPIStartRx(DEVINTRF * const pDev, uint32_t DevCs)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;

	if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
	{
#ifdef SPIM_PRESENT
        dev->pDmaReg->PSEL.MISO = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
		dev->pDmaReg->PSEL.MOSI = -1;
#else
        dev->pReg->PSELMISO = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
		dev->pReg->PSELMOSI = -1;
#endif
	}

	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

	return true;
}

// Receive Data only, no Start/Stop condition
int nRFxSPIRxDataDma(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

#ifdef SPIM_PRESENT
    dev->pDmaReg->TXD.PTR = 0;
    dev->pDmaReg->TXD.MAXCNT = 0;
    dev->pDmaReg->TXD.LIST = 0;
    dev->pDmaReg->RXD.PTR = (uint32_t)pBuff;
    dev->pDmaReg->RXD.LIST = SPIM_RXD_LIST_LIST_ArrayList << SPIM_RXD_LIST_LIST_Pos;

    while (BuffLen > 0)
	{
		int l = min(BuffLen, NRFX_SPI_DMA_MAXCNT);

		dev->pDmaReg->RXD.MAXCNT = l;
		dev->pDmaReg->EVENTS_END = 0;
		dev->pDmaReg->EVENTS_ENDRX = 0;
		dev->pDmaReg->TASKS_START = 1;

		if (nRFxSPIWaitRX(dev, 100000) == false)
			break;

        l = dev->pDmaReg->RXD.AMOUNT;
		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}
#endif

	return cnt;
}

// Receive Data only, no Start/Stop condition
int nRFxSPIRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
    NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev-> pDevData;
    int cnt = 0;

#ifdef SPI_PRESENT
    dev->pReg->EVENTS_READY = 0;

    while (BuffLen > 0)
    {
        dev->pReg->TXD = 0xFF;

        if (nRFxSPIWaitReady(dev, 100000) == false)
            break;

        *pBuff = dev->pReg->RXD;

        BuffLen--;
        pBuff++;
        cnt++;
    }

#endif
    return cnt;
}

// Stop receive
void nRFxSPIStopRx(DEVINTRF * const pDev)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
bool nRFxSPIStartTx(DEVINTRF * const pDev, uint32_t DevCs)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_CS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;

    if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
    {
#ifdef SPIM_PRESENT
        dev->pDmaReg->PSEL.MOSI = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        dev->pDmaReg->PSEL.MISO = -1;
#else
        dev->pReg->PSELMOSI = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        dev->pReg->PSELMISO = -1;
#endif
    }

	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_CS_IOPIN_IDX].PinNo);

	return true;
}

// Transmit Data only, no Start/Stop condition
int nRFxSPITxDataDma(DEVINTRF * const pDev, uint8_t *pData, int DataLen)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

#ifdef SPIM_PRESENT
	dev->pDmaReg->RXD.PTR = 0;
    dev->pDmaReg->RXD.MAXCNT = 0;
    dev->pDmaReg->RXD.LIST = 0;
    dev->pDmaReg->TXD.PTR = (uint32_t)pData;
    dev->pDmaReg->TXD.LIST = SPIM_TXD_LIST_LIST_ArrayList << SPIM_TXD_LIST_LIST_Pos;

    while (DataLen > 0)
	{
		int l = min(DataLen, NRFX_SPI_DMA_MAXCNT);
		dev->pDmaReg->TXD.MAXCNT = l;
		dev->pDmaReg->EVENTS_END = 0;
		dev->pDmaReg->EVENTS_ENDTX = 0;
		dev->pDmaReg->TASKS_START = 1;

		if (nRFxSPIWaitDMA(dev, 100000) == false)
		{
		    break;
		}

		l = dev->pDmaReg->TXD.AMOUNT;
		DataLen -= l;
		pData += l;
		cnt += l;
	}
#endif
	return cnt;
}

// Send Data only, no Start/Stop condition
int nRFxSPITxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
    NRFX_SPIDEV *dev = (NRFX_SPIDEV*)pDev->pDevData;
    int cnt = 0;

#ifdef SPI_PRESENT
    if (pData == NULL)
    {
        return 0;
    }

    while (DataLen > 0)
    {
        dev->pReg->TXD = *pData;

        if (nRFxSPIWaitReady(dev, 10000) == false)
        {
            break;
        }

        int d = dev->pReg->RXD;

        DataLen--;
        pData++;
        cnt++;
    }

#endif

    return cnt;
}

// Stop transmit
void nRFxSPIStopTx(DEVINTRF * const pDev)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_CS_IOPIN_IDX].PinNo);
	}
    if (dev->pSpiDev->Cfg.Phy == SPIPHY_3WIRE)
    {
#ifdef SPIM_PRESENT
        dev->pDmaReg->PSEL.MISO = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        dev->pDmaReg->PSEL.MOSI = -1;
#else
        dev->pReg->PSELMOSI = -1;
        dev->pReg->PSELMISO = (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (dev->pSpiDev->Cfg.pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
#endif
    }
}

void nRFxSPIReset(DEVINTRF * const pDev)
{

}

void SPIIrqHandler(int DevNo, DEVINTRF * const pDev)
{
	NRFX_SPIDEV *dev = (NRFX_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		if (dev->pDmaSReg->EVENTS_ENDRX)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_RX_FIFO_FULL, NULL, 0);
			}
			dev->pDmaSReg->EVENTS_ENDRX = 0;
			dev->pDmaSReg->STATUS = dev->pDmaSReg->STATUS;
		}

		if (dev->pDmaSReg->EVENTS_END)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
#ifdef SPIM_PRESENT
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_COMPLETED, (uint8_t*)dev->pDmaSReg->RXD.PTR, dev->pDmaSReg->RXD.AMOUNT);
#else
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_COMPLETED, (uint8_t*)dev->pDmaSReg->RXDPTR, dev->pDmaSReg->AMOUNTRX);
#endif
			}
			dev->pDmaSReg->EVENTS_END = 0;
		}

		if (dev->pDmaSReg->EVENTS_ACQUIRED)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_STATECHG, NULL, 0);
			}
			dev->pDmaSReg->STATUS = dev->pDmaSReg->STATUS;

#ifdef SPIM_PRESENT
			dev->pDmaSReg->RXD.PTR = (uint32_t)dev->pSpiDev->pRxBuff[0];
			dev->pDmaSReg->RXD.MAXCNT = dev->pSpiDev->RxBuffLen[0];
			dev->pDmaSReg->TXD.PTR = (uint32_t)dev->pSpiDev->pTxData[0];
			dev->pDmaSReg->TXD.MAXCNT = dev->pSpiDev->TxDataLen[0];
#else
			dev->pDmaSReg->RXDPTR = (uint32_t)dev->pSpiDev->pRxBuff[0];
			dev->pDmaSReg->MAXRX = dev->pSpiDev->RxBuffLen[0];
			dev->pDmaSReg->TXDPTR = (uint32_t)dev->pSpiDev->pTxData[0];
			dev->pDmaSReg->MAXTX = dev->pSpiDev->TxDataLen[0];
#endif
			dev->pDmaSReg->EVENTS_ACQUIRED = 0;
			dev->pDmaSReg->TASKS_RELEASE = 1;
		}
	}
}

bool SPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->Mode == SPIMODE_SLAVE && pCfgData->DevNo >= NRFX_SPISLAVE_MAXDEV)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= NRFX_SPI_MAXDEV || pCfgData->NbIOPins < 3)
	{
		return false;
	}

	uint32_t cfgreg = 0;

	// Get the correct register map
#ifdef SPIM_PRESENT
	NRF_SPIM_Type *reg = reg = s_nRFxSPIDev[pCfgData->DevNo].pDmaReg;
#else
	NRF_SPI_Type *reg = reg = s_nRFxSPIDev[pCfgData->DevNo].pReg;
#endif

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;

	// Configure I/O pins
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	for (int i = SPI_CS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
	{
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
	}

	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
#ifdef SPIM_PRESENT
		cfgreg |= SPIM_CONFIG_ORDER_LsbFirst;
#else
		cfgreg |= SPI_CONFIG_ORDER_LsbFirst;
#endif
	}
	else
	{
#ifdef SPIM_PRESENT
		cfgreg |= SPIM_CONFIG_ORDER_MsbFirst;
#else
		cfgreg |= SPI_CONFIG_ORDER_MsbFirst;
#endif
	}

	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPHA_Trailing << SPIM_CONFIG_CPHA_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos);
#endif
	}
	else
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPHA_Leading << SPIM_CONFIG_CPHA_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos);
#endif
	}

	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPOL_ActiveLow << SPIM_CONFIG_CPOL_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
#endif
		IOPinSet(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}
	else
	{
#ifdef SPIM_PRESENT
		cfgreg |= (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos);
#else
		cfgreg |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
#endif
		IOPinClear(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}

	reg->CONFIG = cfgreg;

	pDev->Cfg = *pCfgData;
	s_nRFxSPIDev[pCfgData->DevNo].pSpiDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_nRFxSPIDev[pCfgData->DevNo];

	nRFxSPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = nRFxSPIDisable;
	pDev->DevIntrf.Enable = nRFxSPIEnable;
	pDev->DevIntrf.GetRate = nRFxSPIGetRate;
	pDev->DevIntrf.SetRate = nRFxSPISetRate;
	pDev->DevIntrf.StartRx = nRFxSPIStartRx;
	pDev->DevIntrf.RxData = nRFxSPIRxData;
	pDev->DevIntrf.StopRx = nRFxSPIStopRx;
	pDev->DevIntrf.StartTx = nRFxSPIStartTx;
	pDev->DevIntrf.TxData = nRFxSPITxData;
	pDev->DevIntrf.StopTx = nRFxSPIStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.PowerOff = nRFxSPIPowerOff;
	pDev->DevIntrf.Reset = nRFxSPIReset;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	if (pCfgData->Mode == SPIMODE_SLAVE)
	{
		// Force DMA for slave mode
		pDev->DevIntrf.bDma = true;
	}

#ifndef SPI_PRESENT
	// only DMA mode avail.  Force DMA
	pDev->DevIntrf.bDma = true;
#endif

	if (pDev->DevIntrf.bDma == true)
	{
		pDev->DevIntrf.RxData = nRFxSPIRxDataDma;
		pDev->DevIntrf.TxData = nRFxSPITxDataDma;
	}

	uint32_t inten = 0;

    if (pCfgData->Mode == SPIMODE_SLAVE)
	{
		NRF_SPIS_Type *sreg = s_nRFxSPIDev[pCfgData->DevNo].pDmaSReg;

#ifdef SPIM_PRESENT
        sreg->PSEL.SCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
        sreg->PSEL.MISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        sreg->PSEL.MOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);
		sreg->PSEL.CSN = (pCfgData->pIOPinMap[SPI_CS_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_CS_IOPIN_IDX].PortNo << 5);
#else
        sreg->PSELSCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
        sreg->PSELMISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        sreg->PSELMOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);
		sreg->PSELCSN = (pCfgData->pIOPinMap[SPI_CS_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_CS_IOPIN_IDX].PortNo << 5);
#endif
		sreg->ORC = 0xFF;
		sreg->STATUS = sreg->STATUS;
		sreg->EVENTS_ENDRX = 0;
		sreg->EVENTS_END = 0;
		sreg->EVENTS_ACQUIRED = 0;
		sreg->DEF = 0xFF;
		sreg->SHORTS = (SPIS_SHORTS_END_ACQUIRE_Enabled << SPIS_SHORTS_END_ACQUIRE_Pos);

		inten = (SPIS_INTENSET_ACQUIRED_Enabled << SPIS_INTENSET_ACQUIRED_Pos) |
				(SPIS_INTENSET_ENDRX_Enabled << SPIS_INTENSET_ENDRX_Pos) |
				(SPIS_INTENSET_END_Enabled << SPIS_INTENSET_END_Pos);
		reg->ENABLE =  (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
		sreg->TASKS_ACQUIRE = 1;	// Active event to update rx/tx buffer
	}
	else
	{
#ifdef SPIM_PRESENT
		reg->PSEL.SCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
		reg->PSEL.MISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
		reg->PSEL.MOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);

		if (pDev->DevIntrf.bDma)
		{
			reg->ORC = 0xFF;
			reg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
		}
		else
		{
#ifdef SPI_PRESENT
			s_nRFxSPIDev[pCfgData->DevNo].pReg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
			s_nRFxSPIDev[pCfgData->DevNo].pReg->EVENTS_READY = 0;
#endif
		}
#else
		reg->PSELSCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
		reg->PSELMISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
		reg->PSELMOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);

		reg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
        reg->EVENTS_READY = 0;
#endif
	}

    if (inten != 0)
    {
    	SetSharedIntHandler(pCfgData->DevNo, &pDev->DevIntrf, SPIIrqHandler);

    	switch (pCfgData->DevNo)
    	{
#ifdef NRF52_SERIES
    		case 0:
                NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
                NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
                break;
    	    case 1:
                NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
                NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(SPIM2_SPIS2_SPI2_IRQn);
                NVIC_SetPriority(SPIM2_SPIS2_SPI2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM2_SPIS2_SPI2_IRQn);
                break;
#ifdef NRF52840_XXAA
    	    case 3:
                NVIC_ClearPendingIRQ(SPIM3_IRQn);
                NVIC_SetPriority(SPIM3_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM3_IRQn);
                break;
#endif
#elif defined(NRF91_SERIES)
    		case 0:
                NVIC_ClearPendingIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
                NVIC_SetPriority(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
                break;
    	    case 1:
                NVIC_ClearPendingIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
                NVIC_SetPriority(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
                NVIC_SetPriority(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
                break;
    	    case 3:
                NVIC_ClearPendingIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
                NVIC_SetPriority(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
                break;
#elif defined(NRF53_SERIES)
    		case 0:
                NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
                break;
#ifdef NRF5340_XXAA_APPLICATION
    	    case 1:
                NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
                break;
    	    case 2:
                NVIC_ClearPendingIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                NVIC_SetPriority(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
                break;
    	    case 3:
                NVIC_ClearPendingIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                NVIC_SetPriority(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
                break;
#endif
#else
    		case 0:
                NVIC_ClearPendingIRQ(SPI0_TWI0_IRQn);
                NVIC_SetPriority(SPI0_TWI0_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPI0_TWI0_IRQn);
                break;
    	    case 1:
                NVIC_ClearPendingIRQ(SPI1_TWI1_IRQn);
                NVIC_SetPriority(SPI1_TWI1_IRQn, pCfgData->IntPrio);
                NVIC_EnableIRQ(SPI1_TWI1_IRQn);
                break;
#endif
    	}

    	reg->INTENSET = inten;
    }

	return true;
}




