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

#include "coredev/spi.h"
#include "iopinctrl.h"
#include "i2c_spi_nrf52_irq.h"

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	uint32_t Clk;
	union {
		NRF_SPIM_Type *pReg;	// Master register map
		NRF_SPIS_Type *pSReg;	// Slave register map
	};
} NRF52_SPIDEV;
#pragma pack(pop)

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

bool nRF52SPIWaitDMA(NRF52_SPIDEV * const pDev, uint32_t Timeout)
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

bool nRF52SPIWaitRX(NRF52_SPIDEV * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pReg->EVENTS_ENDRX)
		{
			pDev->pReg->EVENTS_ENDRX = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

int nRF52SPIGetRate(DEVINTRF * const pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((NRF52_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
int nRF52SPISetRate(DEVINTRF * const pDev, int DataRate)
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

void nRF52SPIDisable(DEVINTRF * const pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		dev->pReg->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
	}
	else
	{
		dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
	}
}

void nRF52SPIEnable(DEVINTRF * const pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		dev->pReg->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
	}
	else
	{
		dev->pReg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
	}
}

// Initial receive
bool nRF52SPIStartRx(DEVINTRF * const pDev, int DevCs)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);

	return true;
}

// Receive Data only, no Start/Stop condition
int nRF52SPIRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
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
		dev->pReg->EVENTS_ENDRX = 0;
		dev->pReg->TASKS_START = 1;

		if (nRF52SPIWaitRX(dev, 100000) == false)
			break;

        l = dev->pReg->RXD.AMOUNT;
		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}

	return cnt;
}

// Stop receive
void nRF52SPIStopRx(DEVINTRF * const pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
bool nRF52SPIStartTx(DEVINTRF * const pDev, int DevCs)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);

	return true;
}

// Transmit Data only, no Start/Stop condition
int nRF52SPITxData(DEVINTRF * const pDev, uint8_t *pData, int DataLen)
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
		dev->pReg->EVENTS_ENDTX = 0;
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
void nRF52SPIStopTx(DEVINTRF * const pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
	}
}

void SPIIrqHandler(int DevNo, DEVINTRF * const pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		if (dev->pSReg->EVENTS_ENDRX)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_RX_FIFO_FULL, NULL, 0);
			}
			dev->pSReg->EVENTS_ENDRX = 0;
			dev->pSReg->STATUS = dev->pSReg->STATUS;
		}

		if (dev->pSReg->EVENTS_END)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_COMPLETED, (uint8_t*)dev->pSReg->RXD.PTR, dev->pSReg->RXD.AMOUNT);
			}
			dev->pSReg->EVENTS_END = 0;
		}

		if (dev->pSReg->EVENTS_ACQUIRED)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_STATECHG, NULL, 0);
			}
			dev->pSReg->STATUS = dev->pSReg->STATUS;

			dev->pSReg->RXD.PTR = (uint32_t)dev->pSpiDev->pRxBuff[0];
			dev->pSReg->RXD.MAXCNT = dev->pSpiDev->RxBuffLen[0];
			dev->pSReg->TXD.PTR = (uint32_t)dev->pSpiDev->pTxData[0];
			dev->pSReg->TXD.MAXCNT = dev->pSpiDev->TxDataLen[0];

			dev->pSReg->EVENTS_ACQUIRED = 0;
			dev->pSReg->TASKS_RELEASE = 1;
		}
	}
}

bool SPIInit(SPIDEV * const pDev, const SPICFG *pCfgData)
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

	for (int i = SPI_SS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
	{
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);
	}

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
	pDev->DevIntrf.pDevData = (void*)&s_nRF52SPIDev[pCfgData->DevNo];

	nRF52SPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Disable = nRF52SPIDisable;
	pDev->DevIntrf.Enable = nRF52SPIEnable;
	pDev->DevIntrf.GetRate = nRF52SPIGetRate;
	pDev->DevIntrf.SetRate = nRF52SPISetRate;
	pDev->DevIntrf.StartRx = nRF52SPIStartRx;
	pDev->DevIntrf.RxData = nRF52SPIRxData;
	pDev->DevIntrf.StopRx = nRF52SPIStopRx;
	pDev->DevIntrf.StartTx = nRF52SPIStartTx;
	pDev->DevIntrf.TxData = nRF52SPITxData;
	pDev->DevIntrf.StopTx = nRF52SPIStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.bBusy = false;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;

	uint32_t inten = 0;

    if (pCfgData->Mode == SPIMODE_SLAVE)
	{
		NRF_SPIS_Type *sreg = s_nRF52SPIDev[pCfgData->DevNo].pSReg;

		sreg->PSEL.SCK = pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo;
		sreg->PSEL.MISO = pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo;
		sreg->PSEL.MOSI = pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo;
		sreg->PSEL.CSN = pCfgData->pIOPinMap[SPI_SS_IOPIN_IDX].PinNo;

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
		reg->PSEL.SCK = pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo;
		reg->PSEL.MISO = pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo;
		reg->PSEL.MOSI = pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo;
    	reg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
	}

    if (pCfgData->bIntEn)
    {
    	SetI2cSpiIntHandler(pCfgData->DevNo, &pDev->DevIntrf, SPIIrqHandler);

    	if (pCfgData->DevNo == 0)
    	{
    		NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
    		NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, pCfgData->IntPrio);
    		NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
    	}
    	else
    	{
    		NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
    		NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, pCfgData->IntPrio);
    		NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
    	}

    	reg->INTENSET = inten;
    }

	return true;
}




