/*--------------------------------------------------------------------------
File   : spi_nrf5x.cpp

Author : Hoang Nguyen Hoan          Oct. 6, 2016

Desc   : SPI implementation on nRF5x series MCU

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
#include "spi_nrf52.h"
#include "nrf.h"

#include "iopinctrl.h"

#ifdef NRF52
	typedef NRF_SPIM_Type	NRF5X_SPI_REG;	// Register map
#else
	typedef NRF_SPI_Type	NRF5X_SPI_REG;	// Register map
#endif

#define NRF_SPI_REG_EVENTS_READY		0x108

typedef struct {
	int SpiNo;
	SPIDEV *pSpiDev;
	uint32_t Clk;
	NRF_SPIM_Type *pReg;	// Register map
	int CsPin;				// Chip select pin, Nordic SPI has manual SS pin
} NRF5X_SPIDEV;

#define NRF5X_SPI_MAXDEV		3

static NRF5X_SPIDEV s_nRF5xDev[NRF5X_SPI_MAXDEV] = {
	{
		0, NULL, 0, (NRF_SPIM_Type*)NRF_SPIM0_BASE, -1
	},
	{
		1, NULL, 0, (NRF_SPIM_Type*)NRF_SPIM1_BASE, -1
	},
	{
		2, NULL, 0, (NRF_SPIM_Type*)NRF_SPIM2_BASE, -1
	},
};

bool nRF5xSPIWaitReady(NRF5X_SPIDEV *pDev, int32_t Timeout)
{
	uint32_t val = 0;

	do {
		val = *(uint32_t*)((uint32_t)pDev->pReg + NRF_SPI_REG_EVENTS_READY);
		if (val)
			return true;
	} while (Timeout-- > 0);

	return false;
}

bool nRF5xSPIWaitDMA(NRF5X_SPIDEV *pDev, int32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pReg->EVENTS_END)
			return true;
	} while (Timeout-- > 0);

	return false;
}

int nRF5xSPIGetRate(SERINTRFDEV *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((NRF5X_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
int nRF5xSPISetRate(SERINTRFDEV *pDev, int DataRate)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev-> pDevData;

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

void nRF5xSPIDisable(SERINTRFDEV *pDev)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev->pDevData;

	dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
}

void nRF5xSPIEnable(SERINTRFDEV *pDev)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev->pDevData;

	dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}

// Initial receive
bool nRF5xSPIStartRx(SERINTRFDEV *pDev, int DevAddr)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev-> pDevData;

	IOPinClear(0, dev->CsPin);

	return true;
}

// Receive Data only, no Start/Stop condition
int nRF5xSPIRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev-> pDevData;

	dev->pReg->RXD.PTR = (uint32_t)pBuff;
	dev->pReg->RXD.MAXCNT = BuffLen;
	dev->pReg->RXD.LIST = 0; // Scatter/Gather not supported
	dev->pReg->TXD.PTR = 0;
	dev->pReg->TXD.MAXCNT = 0;
	dev->pReg->TXD.LIST = 0; // Scatter/Gather not supported
	dev->pReg->TASKS_START = 1;

	nRF5xSPIWaitDMA(dev, 10000);

	return dev->pReg->RXD.AMOUNT;
}

// Stop receive
void nRF5xSPIStopRx(SERINTRFDEV *pDev)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev-> pDevData;

	IOPinSet(0, dev->CsPin);
}

// Initiate transmit
bool nRF5xSPIStartTx(SERINTRFDEV *pDev, int DevAddr)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev-> pDevData;

	IOPinClear(0, dev->CsPin);

	return true;
}

// Transmit Data only, no Start/Stop condition
int nRF5xSPITxData(SERINTRFDEV *pDev, uint8_t *pData, int DataLen)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev-> pDevData;

	dev->pReg->RXD.PTR = NULL;
	dev->pReg->RXD.MAXCNT = 0;
	dev->pReg->RXD.LIST = 0; // Scatter/Gather not supported
	dev->pReg->TXD.PTR = (uint32_t)pData;
	dev->pReg->TXD.MAXCNT = DataLen;
	dev->pReg->TXD.LIST = 0; // Scatter/Gather not supported
	dev->pReg->TASKS_START = 1;

	nRF5xSPIWaitDMA(dev, 10000);

	return dev->pReg->TXD.AMOUNT;
}

// Stop transmit
void nRF5xSPIStopTx(SERINTRFDEV *pDev)
{
	NRF5X_SPIDEV *dev = (NRF5X_SPIDEV *)pDev-> pDevData;

	IOPinSet(0, dev->CsPin);
}

bool SPIInit(SPIDEV *pDev, const SPICFG *pCfgData)
{
	NRF5X_SPI_REG *reg;
	uint32_t err_code;
	uint32_t cfgreg = 0;

	if (pCfgData->DevNo < 0 || pCfgData->DevNo > 2)
		return false;

	// Get the correct register map
	reg = s_nRF5xDev[pCfgData->DevNo].pReg;

	// Configure I/O pins
	IOPinCfg(pCfgData->IOPinMap, SPI_MAX_NB_IOPIN);

	reg->PSEL.SCK = pCfgData->IOPinMap[SPI_SCK_IOPIN_IDX].PinNo;
	reg->PSEL.MISO = pCfgData->IOPinMap[SPI_MISO_IOPIN_IDX].PinNo;
	reg->PSEL.MOSI = pCfgData->IOPinMap[SPI_MOSI_IOPIN_IDX].PinNo;

	s_nRF5xDev[pCfgData->DevNo].CsPin = pCfgData->IOPinMap[SPI_SS_IOPIN_IDX].PinNo;
	IOPinSet(0, s_nRF5xDev[pCfgData->DevNo].CsPin);

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
		IOPinSet(0, pCfgData->IOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}
	else
	{
		cfgreg |= (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
		IOPinClear(0, pCfgData->IOPinMap[SPI_SCK_IOPIN_IDX].PinNo);
	}

	reg->CONFIG = cfgreg;

	reg->ORC = 0xFF;

	pDev->Cfg = *pCfgData;
	s_nRF5xDev[pCfgData->DevNo].pSpiDev  = pDev;
	pDev->SerIntrf.pDevData = (void*)&s_nRF5xDev[pCfgData->DevNo];

	nRF5xSPISetRate(&pDev->SerIntrf, pCfgData->Rate);

	pDev->SerIntrf.Disable = nRF5xSPIDisable;
	pDev->SerIntrf.Enable = nRF5xSPIEnable;
	pDev->SerIntrf.GetRate = nRF5xSPIGetRate;
	pDev->SerIntrf.SetRate = nRF5xSPISetRate;
	pDev->SerIntrf.StartRx = nRF5xSPIStartRx;
	pDev->SerIntrf.RxData = nRF5xSPIRxData;
	pDev->SerIntrf.StopRx = nRF5xSPIStopRx;
	pDev->SerIntrf.StartTx = nRF5xSPIStartTx;
	pDev->SerIntrf.TxData = nRF5xSPITxData;
	pDev->SerIntrf.StopTx = nRF5xSPIStopTx;
	pDev->SerIntrf.IntPrio = pCfgData->IntPrio;
	pDev->SerIntrf.EvtCB = pCfgData->EvtCB;

	reg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
}




