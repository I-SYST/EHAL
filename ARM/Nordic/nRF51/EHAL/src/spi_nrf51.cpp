/*--------------------------------------------------------------------------
File   : spi_nrf51.cpp

Author : Hoang Nguyen Hoan          Oct. 12, 2016

Desc   : SPI implementation on nRF51 series MCU

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

#include "idelay.h"
#include "coredev/spi.h"
#include "iopinctrl.h"

#define NRF51_SPI_MAXDEV        2

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	uint32_t Clk;
	NRF_SPI_Type *pReg;	// Register map
} NRF51_SPIDEV;
#pragma pack(pop)

static NRF51_SPIDEV s_nRF51SPIDev[NRF51_SPI_MAXDEV] = {
	{
		0, NULL, 0, NRF_SPI0
	},
	{
		1, NULL, 0, NRF_SPI1
	},
};

bool nRF51SPIWaitReady(NRF51_SPIDEV *pDev, int Timeout)
{
    do {
        if (pDev->pReg->EVENTS_READY)
        {
            return true;
        }
    } while (Timeout-- >  0);

    return false;
}

void nRF51SPIDisable(DEVINTRF *pDev)
{
	NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

	dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
}
void nRF51SPIEnable(DEVINTRF *pDev)
{
	NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

	dev->pReg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
}

int nRF51SPIGetRate(DEVINTRF *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((NRF51_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

int nRF51SPISetRate(DEVINTRF *pDev, int DataRate)
{
	NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

	if (DataRate < 250000)
	{
		dev->pReg->FREQUENCY = SPI_FREQUENCY_FREQUENCY_K125;
		dev->Clk = 125000;
	}
	else if (DataRate < 500000)
	{
		dev->pReg->FREQUENCY = SPI_FREQUENCY_FREQUENCY_K250;
		dev->Clk = 250000;
	}
	else if (DataRate < 1000000)
	{
		dev->pReg->FREQUENCY = SPI_FREQUENCY_FREQUENCY_K500;
		dev->Clk = 500000;
	}
	else if (DataRate < 2000000)
	{
		dev->pReg->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M1;
		dev->Clk = 1000000;
	}
	else if (DataRate < 4000000)
	{
		dev->pReg->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M2;
		dev->Clk = 2000000;
	}
	else if (DataRate < 8000000)
	{
		dev->pReg->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M4;
		dev->Clk = 4000000;
	}
	else
	{
		dev->pReg->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M8;
		dev->Clk = 8000000;
	}

	dev->pSpiDev->Cfg.Rate = dev->Clk;

	return dev->pSpiDev->Cfg.Rate;
}

bool nRF51SPIStartRx(DEVINTRF *pDev, int DevCs)
{
	NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);

	return true;
}

// Receive Data only, no Start/Stop condition
int nRF51SPIRxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen)
{
	NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;
	int cnt = 0;

	if (pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	uint8_t d = 0xff;

	while (BuffLen > 0)
	{
		dev->pReg->TXD = d;

		if (nRF51SPIWaitReady(dev, 100000) == false)
		{
			break;
		}

		*pBuff = dev->pReg->RXD;

		BuffLen--;
		pBuff++;
		cnt++;
	}

	return cnt;
}

void nRF51SPIStopRx(DEVINTRF *pDev)
{
    NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

	IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
			 dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
}

bool nRF51SPIStartTx(DEVINTRF *pDev, int DevCs)
{
	NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;
	IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[DevCs + SPI_SS_IOPIN_IDX].PinNo);

	return true;
}

// Send Data only, no Start/Stop condition
int nRF51SPITxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;
	int cnt = 0;

	if (pData == NULL)
	{
		return 0;
	}

	while (DataLen > 0)
	{
		dev->pReg->TXD = *pData;

		if (nRF51SPIWaitReady(dev, 10000) == false)
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

void nRF51SPIStopTx(DEVINTRF *pDev)
{
    NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

	IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PortNo,
			   dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + SPI_SS_IOPIN_IDX].PinNo);
}

void nRF51SPIReset(DEVINTRF *pDev)
{
    NRF51_SPIDEV *dev = (NRF51_SPIDEV*)pDev->pDevData;

    nRF51SPIDisable(pDev);
/*
    IOPinConfig(0, dev->pReg->PSELSCL, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    IOPinConfig(0, dev->pReg->PSELSDA, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

    IOPinSet(0, dev->pReg->PSELSDA);

    for (int i = 0; i < 10; i++)
    {
        IOPinSet(0, dev->pReg->PSELSCL);
//        usDelay(5);
        IOPinClear(0, dev->pReg->PSELSCL);
//        usDelay(5);
    }

    IOPinConfig(0, dev->pReg->PSELSDA, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    IOPinClear(0, dev->pReg->PSELSDA);
//    usDelay(5);
    IOPinSet(0, dev->pReg->PSELSCL);
//    usDelay(2);
    IOPinSet(0, dev->pReg->PSELSDA);
*/
    nRF51SPIEnable(pDev);
}

bool SPIInit(SPIDEV *pDev, const SPICFG *pCfgData)
{
	uint32_t cfgreg = 0;

	if (pCfgData->DevNo < 0 || pCfgData->DevNo >= NRF51_SPI_MAXDEV || pCfgData->NbIOPins < 3)
		return false;

	// Get the correct register map
	NRF_SPI_Type *reg = s_nRF51SPIDev[pCfgData->DevNo].pReg;

	// Configure I/O pins
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	reg->PSELSCK = pCfgData->pIOPinMap[SPI_SCK_IOPIN_IDX].PinNo;
	reg->PSELMISO = pCfgData->pIOPinMap[SPI_MISO_IOPIN_IDX].PinNo;
	reg->PSELMOSI = pCfgData->pIOPinMap[SPI_MOSI_IOPIN_IDX].PinNo;

	for (int i = SPI_SS_IOPIN_IDX; i < pCfgData->NbIOPins; i++)
		IOPinSet(pCfgData->pIOPinMap[i].PortNo, pCfgData->pIOPinMap[i].PinNo);

	if (pCfgData->BitOrder == SPIDATABIT_LSB)
	{
		cfgreg |= SPI_CONFIG_ORDER_LsbFirst;
	}
	else
	{
		cfgreg |= SPI_CONFIG_ORDER_MsbFirst;
	}

	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		cfgreg |= (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos);
	}
	else
	{
		cfgreg |= (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos);
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

	pDev->Cfg = *pCfgData;
    s_nRF51SPIDev[pCfgData->DevNo].DevNo = pCfgData->DevNo;
	s_nRF51SPIDev[pCfgData->DevNo].pSpiDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_nRF51SPIDev[pCfgData->DevNo];

	nRF51SPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = nRF51SPIDisable;
	pDev->DevIntrf.Enable = nRF51SPIEnable;
	pDev->DevIntrf.GetRate = nRF51SPIGetRate;
	pDev->DevIntrf.SetRate = nRF51SPISetRate;
	pDev->DevIntrf.StartRx = nRF51SPIStartRx;
	pDev->DevIntrf.RxData = nRF51SPIRxData;
	pDev->DevIntrf.StopRx = nRF51SPIStopRx;
	pDev->DevIntrf.StartTx = nRF51SPIStartTx;
	pDev->DevIntrf.TxData = nRF51SPITxData;
	pDev->DevIntrf.StopTx = nRF51SPIStopTx;
	pDev->DevIntrf.Reset = nRF51SPIReset;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.bBusy = false;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;

	// Clear all errors

	reg->EVENTS_READY = 0;
	reg->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

	return true;
}
