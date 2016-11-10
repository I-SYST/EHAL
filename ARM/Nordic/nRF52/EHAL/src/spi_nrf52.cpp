/*--------------------------------------------------------------------------
File   : spi_nrf5x.cpp

Author : Hoang Nguyen Hoan          Oct. 6, 2016

Desc   : SPI implementation on nRF52 series MCU

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

----------------------------------------------------------------------------
Modified by         Date            Description

----------------------------------------------------------------------------*/
#include "nrf.h"

#include "spi_nrf52.h"
#include "iopinctrl.h"

typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	uint32_t Clk;
	NRF_SPIM_Type *pReg;	// Register map
} NRF52_SPIDEV;

#define NRF52_SPI_MAXDEV		3
#define NRF52_SPI_DMA_MAXCNT	255

static NRF52_SPIDEV s_nRF52SPIDev[NRF52_SPI_MAXDEV] = {
	{
		0, NULL, 0, (NRF_SPIM_Type*)NRF_SPIM0_BASE
	},
	{
		1, NULL, 0, (NRF_SPIM_Type*)NRF_SPIM1_BASE
	},
	{
		2, NULL, 0, (NRF_SPIM_Type*)NRF_SPIM2_BASE
	},
};

bool nRF52SPIWaitDMA(NRF52_SPIDEV *pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pReg->EVENTS_END)
		{
			pDev->pReg->EVENTS_END = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

int nRF52SPIGetRate(SERINTRFDEV *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((NRF52_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
int nRF52SPISetRate(SERINTRFDEV *pDev, int DataRate)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	if (DataRate < 250000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K125;
		dev->Clk = 125000;
	}
	else if (DataRate < 500000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K250;
		dev->Clk = 250000;
	}
	else if (DataRate < 1000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K500;
		dev->Clk = 500000;
	}
	else if (DataRate < 2000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M1;
		dev->Clk = 1000000;
	}
	else if (DataRate < 4000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M2;
		dev->Clk = 2000000;
	}
	else if (DataRate < 8000000)
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M4;
		dev->Clk = 4000000;
	}
	else
	{
		dev->pReg->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M8;
		dev->Clk = 8000000;
	}

	dev->pSpiDev->Cfg.Rate = dev->Clk;

	return dev->pSpiDev->Cfg.Rate;
}

void nRF52SPIDisable(SERINTRFDEV *pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
}

void nRF52SPIEnable(SERINTRFDEV *pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}

// Initial receive
bool nRF52SPIStartRx(SERINTRFDEV *pDev, int DevCs)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);

	return true;
}

// Receive Data only, no Start/Stop condition
int nRF52SPIRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	while (BuffLen > 0)
	{
		int l = min(BuffLen, NRF52_SPI_DMA_MAXCNT);

		dev->pReg->RXD.PTR = (uint32_t)pBuff;
		dev->pReg->RXD.MAXCNT = l;
		dev->pReg->RXD.LIST = 0; // Scatter/Gather not supported
		dev->pReg->TXD.PTR = 0;
		dev->pReg->TXD.MAXCNT = 0;
		dev->pReg->TXD.LIST = 0; // Scatter/Gather not supported
		dev->pReg->EVENTS_END = 0;
		dev->pReg->TASKS_START = 1;

		nRF52SPIWaitDMA(dev, 100000);

        l = dev->pReg->RXD.AMOUNT;
		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}

	return cnt;
}

// Stop receive
void nRF52SPIStopRx(SERINTRFDEV *pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;

	IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
			 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
}

// Initiate transmit
bool nRF52SPIStartTx(SERINTRFDEV *pDev, int DevCs)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);

	return true;
}

// Transmit Data only, no Start/Stop condition
int nRF52SPITxData(SERINTRFDEV *pDev, uint8_t *pData, int DataLen)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	while (DataLen > 0)
	{
		int l = min(DataLen, NRF52_SPI_DMA_MAXCNT);
		dev->pReg->RXD.PTR = 0;
		dev->pReg->RXD.MAXCNT = 0;
		dev->pReg->RXD.LIST = 0; // Scatter/Gather not supported
		dev->pReg->TXD.PTR = (uint32_t)pData;
		dev->pReg->TXD.MAXCNT = l;
		dev->pReg->TXD.LIST = 0; // Scatter/Gather not supported
		dev->pReg->EVENTS_END = 0;
		dev->pReg->TASKS_START = 1;

		if (nRF52SPIWaitDMA(dev, 100000) == false)
		{
		    break;
		}

		l = dev->pReg->TXD.AMOUNT;
		DataLen -= l;
		pData += l;
		cnt += l;
	}

	return cnt;
}

// Stop transmit
void nRF52SPIStopTx(SERINTRFDEV *pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;

	IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
}

bool SPIInit(SPIDEV *pDev, const SPICFG *pCfgData)
{
	NRF_SPIM_Type *reg;
	uint32_t err_code;
	uint32_t cfgreg = 0;

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= NRF52_SPI_MAXDEV || pCfgData->NbIOPins < 3)
		return false;

	// Get the correct register map
	reg = s_nRF52SPIDev[pCfgData->DevNo].pReg;

	// Configure I/O pins
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	reg->PSEL.SCK = pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo;
	reg->PSEL.MISO = pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo;
	reg->PSEL.MOSI = pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo;

	for (int i = SPI_SS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);

	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cfgreg |= SPIM_CONFIG_ORDER_LsbFirst;
	}
	else
	{
		cfgreg |= SPIM_CONFIG_ORDER_MsbFirst;
	}

	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cfgreg |= (SPIM_CONFIG_CPHA_Trailing   << SPIM_CONFIG_CPHA_Pos);
	}
	else
	{
		cfgreg |= (SPIM_CONFIG_CPHA_Leading    << SPIM_CONFIG_CPHA_Pos);
	}

	//config.irq_priority = APP_IRQ_PRIORITY_LOW;

	if (pCfgData->ClkPol == SPICLKPOL_LOW)
	{
		cfgreg |= (SPI_CONFIG_CPOL_ActiveLow  << SPI_CONFIG_CPOL_Pos);
		IOPinSet(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}
	else
	{
		cfgreg |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
		IOPinClear(0, pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}

	reg->CONFIG = cfgreg;

	reg->ORC = 0xFF;

	pDev->Cfg = *pCfgData;
	s_nRF52SPIDev[pCfgData->DevNo].pSpiDev  = pDev;
	pDev->SerIntrf.pDevData = (void*)&s_nRF52SPIDev[pCfgData->DevNo];

	nRF52SPISetRate(&pDev->SerIntrf, pCfgData->Rate);

	pDev->SerIntrf.Disable = nRF52SPIDisable;
	pDev->SerIntrf.Enable = nRF52SPIEnable;
	pDev->SerIntrf.GetRate = nRF52SPIGetRate;
	pDev->SerIntrf.SetRate = nRF52SPISetRate;
	pDev->SerIntrf.StartRx = nRF52SPIStartRx;
	pDev->SerIntrf.RxData = nRF52SPIRxData;
	pDev->SerIntrf.StopRx = nRF52SPIStopRx;
	pDev->SerIntrf.StartTx = nRF52SPIStartTx;
	pDev->SerIntrf.TxData = nRF52SPITxData;
	pDev->SerIntrf.StopTx = nRF52SPIStopTx;
	pDev->SerIntrf.IntPrio = pCfgData->IntPrio;
	pDev->SerIntrf.EvtCB = pCfgData->EvtCB;
	pDev->SerIntrf.Busy = false;

	reg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}




