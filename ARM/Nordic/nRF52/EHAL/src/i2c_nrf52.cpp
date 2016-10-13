/*--------------------------------------------------------------------------
File   : i2c_nrf52.cpp

Author : Hoang Nguyen Hoan          Oct. 12, 2016

Desc   : I2C implementation on nRF52 series MCU

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

#include "i2c_nrf52.h"
#include "iopinctrl.h"

typedef struct {
	int DevNo;
	I2CDEV *pI2cDev;
	uint32_t Clk;
	NRF_TWIM_Type *pReg;	// Register map
} NRF52_I2CDEV;

#define NRF52_I2C_MAXDEV		2

static NRF52_I2CDEV s_nRF52I2CDev[NRF52_I2C_MAXDEV] = {
	{
		0, NULL, 0, (NRF_TWIM_Type *)NRF_TWIM0_BASE
	},
	{
		1, NULL, 0, (NRF_TWIM_Type *)NRF_TWIM1_BASE
	},
};


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

I2CSTATUS nRF52I2CGetStatus(I2CDEV *pDev)
{
//	return (I2CSTATUS)(pDev->pI2CReg->I2STAT & 0xF8);
}

bool nRF52I2CStartCond(NRF52_I2CDEV *pDev)
{
	I2CSTATUS status;

	if (status != I2CSTATUS_START_COND && status != I2CSTATUS_RESTART_COND)
		return false;
	return true;
}

void nRF52I2CStopCond(NRF52_I2CDEV *pDev)
{

	// Disable I2C engine
}

bool nRF52I2CStartRx(SERINTRFDEV *pDev, int DevAddr)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	if (nRF52I2CStartCond(dev))
	{
		// Send I2C address
		//dev->pReg->I2DAT = (DevAddr << 1)| 1;
		//pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;
		//LpcI2CWaitInt(pDev, 100000);
		//if (LpcI2CWaitStatus(pDev, 100000) == I2CSTATUS_SLAR_ACK)
			return true;
	}

	return false;
}

// Receive Data only, no Start/Stop condition
int nRF52I2CRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;
	I2CSTATUS status;
	int rcount = 0;

	// Start read data
	while (BuffLen > rcount)
	{
		if (status != I2CSTATUS_RXDATA_ACK && status != I2CSTATUS_RXDATA_NACK)
			break;
		pBuff++;
		rcount++;
	}

	return rcount;
}

// Send Data only, no Start/Stop condition
int nRF52I2CTxData(SERINTRFDEV *pDev, uint8_t *pData, int DataLen)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;
	I2CSTATUS status;
	int tcount = 0;

	// Start sending data
	while (DataLen > 0)
	{
		if (status != I2CSTATUS_M_TXDAT_ACK && status != I2CSTATUS_M_TXDAT_NACK)
			break;
		pData++;
		DataLen--;
		tcount++;
	}

	return tcount;
}

void nRF52I2CStopRx(SERINTRFDEV *pDev)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	nRF52I2CStopCond(dev);
}

bool nRF52I2CStartTx(SERINTRFDEV *pDev, int DevAddr)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	if (nRF52I2CStartCond(dev))
	{
			return true;
	}

	return false;
}

void nRF52I2CStopTx(SERINTRFDEV *pDev)
{
	NRF52_I2CDEV *dev = (NRF52_I2CDEV*)pDev->pDevData;

	nRF52I2CStopCond(dev);
}

bool I2CInit(I2CDEV *pDev, I2CCFG *pCfgData)
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

	reg->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);
}
