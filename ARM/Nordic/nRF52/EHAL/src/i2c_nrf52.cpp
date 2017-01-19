/*--------------------------------------------------------------------------
File   : i2c_nrf52.cpp

Author : Hoang Nguyen Hoan          Oct. 12, 2016

Desc   : I2C implementation on nRF52 series MCU using EasyDMA

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

#include "i2c.h"
#include "iopinctrl.h"

typedef struct {
	int DevNo;
	I2CDEV *pI2cDev;
	uint32_t Clk;
	NRF_TWIM_Type *pReg;	// Register map
} NRF52_I2CDEV;

#define NRF52_I2C_MAXDEV		2
#define NRF52_I2C_DMA_MAXCNT	255

static NRF52_I2CDEV s_nRF52I2CDev[NRF52_I2C_MAXDEV] = {
	{
		0, NULL, 0, (NRF_TWIM_Type *)NRF_TWIM0_BASE
	},
	{
		1, NULL, 0, (NRF_TWIM_Type *)NRF_TWIM1_BASE
	},
};

bool nRF52I2CWaitRxComplete(NRF52_I2CDEV *pDev, int Timeout)
{
	do {
		if (pDev->pReg->EVENTS_ERROR)
		{
			// Abort in case error
            pDev->pReg->ERRORSRC = pDev->pReg->ERRORSRC;
			pDev->pReg->EVENTS_ERROR = 0;
			pDev->pReg->TASKS_RESUME = 1;
			pDev->pReg->TASKS_STOP;
		}
		if (pDev->pReg->EVENTS_LASTRX)
		{
			// Must wait for last DMA then issue a stop
			pDev->pReg->EVENTS_LASTRX = 0;
			//pDev->pReg->TASKS_STOP = 1;
			return true;
		}
/*		if (pDev->pReg->EVENTS_STOPPED)
		{
			// Must wait for stop, other wise DMA count would
			// not be updated with correct value
			pDev->pReg->EVENTS_STOPPED = 0;
			return true;
		}*/
	} while (Timeout-- >  0);

	return false;
}

bool nRF52I2CWaitTxComplete(NRF52_I2CDEV *pDev, int Timeout)
{
	uint32_t d;
	do {
		if (pDev->pReg->EVENTS_ERROR)
		{
			// Abort in case error
			pDev->pReg->ERRORSRC = pDev->pReg->ERRORSRC;
			pDev->pReg->EVENTS_ERROR = 0;
			pDev->pReg->TASKS_RESUME = 1;
			pDev->pReg->TASKS_STOP = 1;
		}
		if (pDev->pReg->EVENTS_LASTTX)
		{
			// Must wait for last DMA then issue a stop
			pDev->pReg->EVENTS_LASTTX = 0;
//			pDev->pReg->TASKS_STOP = 1;
			return true;
		}
/*		if (pDev->pReg->EVENTS_STOPPED)
		{
			// Must wait for stop, other wise DMA count would
			// not be updated with correct value
			pDev->pReg->EVENTS_STOPPED = 0;
			return true;
		}*/
	} while (Timeout-- >  0);

	return false;
}

bool nRF52I2CWaitStop(NRF52_I2CDEV *pDev, int Timeout)
{
    uint32_t d;
    do {
        if (pDev->pReg->EVENTS_ERROR)
        {
            // Abort in case error
            pDev->pReg->ERRORSRC = pDev->pReg->ERRORSRC;
            pDev->pReg->EVENTS_ERROR = 0;
            pDev->pReg->TASKS_RESUME = 1;
            pDev->pReg->TASKS_STOP = 1;
        }
        if (pDev->pReg->EVENTS_STOPPED)
        {
            // Must wait for stop, other wise DMA count would
            // not be updated with correct value
            pDev->pReg->EVENTS_STOPPED = 0;
            return true;
        }
    } while (Timeout-- >  0);

    return false;
}

void nRF52I2CDisable(SERINTRFDEV *pDev)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	dev->pReg->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);
}
void nRF52I2CEnable(SERINTRFDEV *pDev)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	dev->pReg->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}

int nRF52I2CGetRate(SERINTRFDEV *pDev)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	return dev->pI2cDev->Rate;
}

int nRF52I2CSetRate(SERINTRFDEV *pDev, int RateHz)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	if (RateHz < 250000)
	{
		dev->pReg->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100;
		dev->pI2cDev->Rate = 100000;
	}
	else if (RateHz < 400000)
	{
		dev->pReg->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K250;
		dev->pI2cDev->Rate = 250000;
	}
	else
	{
		dev->pReg->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400;
		dev->pI2cDev->Rate = 400000;
	}

	return dev->pI2cDev->Rate;
}

bool nRF52I2CStartRx(SERINTRFDEV *pDev, int DevAddr)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	dev->pReg->ADDRESS = DevAddr;
	dev->pReg->INTENCLR = 0xFFFFFFFF;
	return true;
}

// Receive Data only, no Start/Stop condition
int nRF52I2CRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;
	uint32_t d;
	int cnt = 0;

	while (BuffLen > 0)
	{
		int l = min(BuffLen, NRF52_I2C_DMA_MAXCNT);
		dev->pReg->EVENTS_ERROR = 0;
		dev->pReg->EVENTS_STOPPED = 0;
		dev->pReg->RXD.PTR = (uint32_t)pBuff;
		dev->pReg->RXD.MAXCNT = l;
		dev->pReg->RXD.LIST = 0;
		dev->pReg->TASKS_STARTRX = 1;

		if (nRF52I2CWaitRxComplete(dev, 100000) == false)
		    break;

/*		l = dev->pReg->RXD.AMOUNT;
		if (l <= 0)
		    break;*/

		BuffLen -= l;
		pBuff += l;
		cnt += l;
	}
	return cnt;
}

void nRF52I2CStopRx(SERINTRFDEV *pDev)
{
    NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;
    dev->pReg->TASKS_STOP = 1;
    nRF52I2CWaitStop(dev, 1000);
}

bool nRF52I2CStartTx(SERINTRFDEV *pDev, int DevAddr)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	dev->pReg->ADDRESS = DevAddr;
	dev->pReg->INTENCLR = 0xFFFFFFFF;

	return true;
}

// Send Data only, no Start/Stop condition
int nRF52I2CTxData(SERINTRFDEV *pDev, uint8_t *pData, int DataLen)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;
	uint32_t d;
	int cnt = 0;

	while (DataLen > 0)
	{
		int l = min(DataLen, NRF52_I2C_DMA_MAXCNT);

		dev->pReg->EVENTS_ERROR = 0;
		dev->pReg->EVENTS_STOPPED = 0;
		dev->pReg->TXD.PTR = (uint32_t)pData;
		dev->pReg->TXD.MAXCNT = l;
		dev->pReg->TXD.LIST = 0;
		dev->pReg->TASKS_STARTTX = 1;

		if (nRF52I2CWaitTxComplete(dev, 100000) == false)
		    break;

		DataLen -= l;
		pData += l;
		cnt += l;
	}
	return cnt;
}

void nRF52I2CStopTx(SERINTRFDEV *pDev)
{
    NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

    dev->pReg->TASKS_STOP = 1;
    nRF52I2CWaitStop(dev, 1000);
}

bool I2CInit(I2CDEV *pDev, const I2CCFG *pCfgData)
{
	if (pCfgData->DevNo < 0 || pCfgData->DevNo > 2)
		return false;

	// Get the correct register map
	NRF_TWIM_Type *reg = s_nRF52I2CDev[pCfgData->DevNo].pReg;

	// Configure I/O pins
	IOPinCfg(pCfgData->IOPinMap, I2C_MAX_NB_IOPIN);

    reg->PSEL.SCL = pCfgData->IOPinMap[I2C_SCL_IOPIN_IDX].PinNo;
    reg->PSEL.SDA = pCfgData->IOPinMap[I2C_SDA_IOPIN_IDX].PinNo;

    pDev->MaxRetry = pCfgData->MaxRetry;
    pDev->Mode = pCfgData->Mode;
    pDev->SlaveAddr = pCfgData->SlaveAddr;

	s_nRF52I2CDev[pCfgData->DevNo].pI2cDev  = pDev;
	pDev->SerIntrf.pDevData = (void*)&s_nRF52I2CDev[pCfgData->DevNo];

	nRF52I2CSetRate(&pDev->SerIntrf, pCfgData->Rate);

	pDev->SerIntrf.Disable = nRF52I2CDisable;
	pDev->SerIntrf.Enable = nRF52I2CEnable;
	pDev->SerIntrf.GetRate = nRF52I2CGetRate;
	pDev->SerIntrf.SetRate = nRF52I2CSetRate;
	pDev->SerIntrf.StartRx = nRF52I2CStartRx;
	pDev->SerIntrf.RxData = nRF52I2CRxData;
	pDev->SerIntrf.StopRx = nRF52I2CStopRx;
	pDev->SerIntrf.StartTx = nRF52I2CStartTx;
	pDev->SerIntrf.TxData = nRF52I2CTxData;
	pDev->SerIntrf.StopTx = nRF52I2CStopTx;
	pDev->SerIntrf.IntPrio = pCfgData->IntPrio;
	pDev->SerIntrf.EvtCB = pCfgData->EvtCB;
	pDev->SerIntrf.Busy = false;
	pDev->SerIntrf.MaxRetry = pCfgData->MaxRetry;

	reg->EVENTS_ERROR = 0;
	reg->ERRORSRC = reg->ERRORSRC;
	reg->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);

	return true;
}
