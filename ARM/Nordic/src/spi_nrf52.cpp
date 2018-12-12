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
		NRF_SPIM_Type *pRegDma;	// Master register map
		NRF_SPIS_Type *pSRegDma;// Slave register map
		NRF_SPI_Type  *pReg;
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
		if (pDev->pRegDma->EVENTS_END)
		{
			pDev->pRegDma->EVENTS_END = 0; // clear event
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRF52SPIWaitReady(NRF52_SPIDEV * const pDev, uint32_t Timeout)
{
    uint32_t val = 0;

    do {
        if (pDev->pReg->EVENTS_READY)
        {
            pDev->pReg->EVENTS_READY = 0; // clear event
            return true;
        }
    } while (Timeout-- > 0);

    return false;
}

bool nRF52SPIWaitRX(NRF52_SPIDEV * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pRegDma->EVENTS_ENDRX)
		{
			pDev->pRegDma->EVENTS_ENDRX = 0; // clear event
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
		dev->pRegDma->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K125;
		dev->Clk = 125000;
	}
	else if (DataRate < 500000)
	{
		dev->pRegDma->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K250;
		dev->Clk = 250000;
	}
	else if (DataRate < 1000000)
	{
		dev->pRegDma->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_K500;
		dev->Clk = 500000;
	}
	else if (DataRate < 2000000)
	{
		dev->pRegDma->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M1;
		dev->Clk = 1000000;
	}
	else if (DataRate < 4000000)
	{
		dev->pRegDma->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M2;
		dev->Clk = 2000000;
	}
	else if (DataRate < 8000000)
	{
		dev->pRegDma->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M4;
		dev->Clk = 4000000;
	}
	else
	{
		dev->pRegDma->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M8;
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
		dev->pRegDma->ENABLE = (SPIS_ENABLE_ENABLE_Disabled << SPIS_ENABLE_ENABLE_Pos);
	}
	else if (pDev->bDma)
	{
		dev->pRegDma->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos);
	}
	else
	{
        dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
	}
}

void nRF52SPIEnable(DEVINTRF * const pDev)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.Mode == SPIMODE_SLAVE)
	{
		dev->pRegDma->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
	}
	//else if (pDev->bDma)
	//{
	//	dev->pRegDma->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
	//}
	else
	{
        dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
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
int nRF52SPIRxDataDma(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

    dev->pRegDma->TXD.PTR = 0;
    dev->pRegDma->TXD.MAXCNT = 0;
    dev->pRegDma->TXD.LIST = 0;
    dev->pRegDma->RXD.PTR = (uint32_t)pBuff;
    dev->pRegDma->RXD.LIST = SPIM_RXD_LIST_LIST_ArrayList << SPIM_RXD_LIST_LIST_Pos;

    while (BuffLen > 0)
	{
		int l = min(BuffLen, NRF52_SPI_DMA_MAXCNT);

		dev->pRegDma->RXD.MAXCNT = l;
		dev->pRegDma->EVENTS_END = 0;
		dev->pRegDma->EVENTS_ENDRX = 0;
		dev->pRegDma->TASKS_START = 1;

		if (nRF52SPIWaitRX(dev, 100000) == false)
			break;

        l = dev->pRegDma->RXD.AMOUNT;
		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}

	return cnt;
}

// Receive Data only, no Start/Stop condition
int nRF52SPIRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen)
{
    NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;
    int cnt = 0;

    dev->pReg->EVENTS_READY = 0;

    while (BuffLen > 0)
    {
        dev->pReg->TXD = 0xFF;

        if (nRF52SPIWaitReady(dev, 100000) == false)
            break;

        *pBuff = dev->pReg->RXD;

        BuffLen--;
        pBuff++;
        cnt++;
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
int nRF52SPITxDataDma(DEVINTRF * const pDev, uint8_t *pData, int DataLen)
{
	NRF52_SPIDEV *dev = (NRF52_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

    dev->pRegDma->RXD.PTR = 0;
    dev->pRegDma->RXD.MAXCNT = 0;
    dev->pRegDma->RXD.LIST = 0;
    dev->pRegDma->TXD.PTR = (uint32_t)pData;
    dev->pRegDma->TXD.LIST = SPIM_TXD_LIST_LIST_ArrayList << SPIM_TXD_LIST_LIST_Pos;

    while (DataLen > 0)
	{
		int l = min(DataLen, NRF52_SPI_DMA_MAXCNT);
		dev->pRegDma->TXD.MAXCNT = l;
		dev->pRegDma->EVENTS_END = 0;
		dev->pRegDma->EVENTS_ENDTX = 0;
		dev->pRegDma->TASKS_START = 1;

		if (nRF52SPIWaitDMA(dev, 100000) == false)
		{
		    break;
		}

		l = dev->pRegDma->TXD.AMOUNT;
		DataLen -= l;
		pData += l;
		cnt += l;
	}

	return cnt;
}

// Send Data only, no Start/Stop condition
int nRF52SPITxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
    NRF52_SPIDEV *dev = (NRF52_SPIDEV*)pDev->pDevData;
    int cnt = 0;

    if (pData == NULL)
    {
        return 0;
    }

    while (DataLen > 0)
    {
        dev->pReg->TXD = *pData;

        if (nRF52SPIWaitReady(dev, 10000) == false)
        {
            break;
        }

        int d = dev->pReg->RXD;

        DataLen--;
        pData++;
        cnt++;
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
		if (dev->pSRegDma->EVENTS_ENDRX)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_RX_FIFO_FULL, NULL, 0);
			}
			dev->pSRegDma->EVENTS_ENDRX = 0;
			dev->pSRegDma->STATUS = dev->pSRegDma->STATUS;
		}

		if (dev->pSRegDma->EVENTS_END)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_COMPLETED, (uint8_t*)dev->pSRegDma->RXD.PTR, dev->pSRegDma->RXD.AMOUNT);
			}
			dev->pSRegDma->EVENTS_END = 0;
		}

		if (dev->pSRegDma->EVENTS_ACQUIRED)
		{
			if (dev->pSpiDev->Cfg.EvtCB)
			{
				dev->pSpiDev->Cfg.EvtCB(pDev, DEVINTRF_EVT_STATECHG, NULL, 0);
			}
			dev->pSRegDma->STATUS = dev->pSRegDma->STATUS;

			dev->pSRegDma->RXD.PTR = (uint32_t)dev->pSpiDev->pRxBuff[0];
			dev->pSRegDma->RXD.MAXCNT = dev->pSpiDev->RxBuffLen[0];
			dev->pSRegDma->TXD.PTR = (uint32_t)dev->pSpiDev->pTxData[0];
			dev->pSRegDma->TXD.MAXCNT = dev->pSpiDev->TxDataLen[0];

			dev->pSRegDma->EVENTS_ACQUIRED = 0;
			dev->pSRegDma->TASKS_RELEASE = 1;
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
	reg = s_nRF52SPIDev[pCfgData->DevNo].pRegDma;

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

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
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
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;

	if (pCfgData->Mode == SPIMODE_SLAVE)
	{
		// Force DMA for slave mode
		pDev->DevIntrf.bDma = true;
	}

	if (pDev->DevIntrf.bDma == true)
	{
		pDev->DevIntrf.RxData = nRF52SPIRxDataDma;
		pDev->DevIntrf.TxData = nRF52SPITxDataDma;
	}

	uint32_t inten = 0;

    if (pCfgData->Mode == SPIMODE_SLAVE)
	{
		NRF_SPIS_Type *sreg = s_nRF52SPIDev[pCfgData->DevNo].pSRegDma;

        sreg->PSEL.SCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
        sreg->PSEL.MISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
        sreg->PSEL.MOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);
		sreg->PSEL.CSN = (pCfgData->pIOPinMap[SPI_SS_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SS_IOPIN_IDX].PortNo << 5);

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
		reg->PSEL.SCK = (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PortNo << 5);
		reg->PSEL.MISO = (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PortNo << 5);
		reg->PSEL.MOSI = (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo & 0x1f) | (pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PortNo << 5);

		if (pDev->DevIntrf.bDma == true)
		{
			reg->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos);
		}
		else
		{
			reg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
		}

        s_nRF52SPIDev[pCfgData->DevNo].pReg->EVENTS_READY = 0;
	}

    if (pCfgData->bIntEn)
    {
    	SetI2cSpiIntHandler(pCfgData->DevNo, &pDev->DevIntrf, SPIIrqHandler);

    	switch (pCfgData->DevNo)
    	{
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
    	}

    	reg->INTENSET = inten;
    }

	return true;
}




