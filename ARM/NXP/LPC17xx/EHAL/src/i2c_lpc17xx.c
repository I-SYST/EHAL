/*--------------------------------------------------------------------------
File   : lpci2c.c

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : I2C implementation on LPC

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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "system_LPC17xx.h"

#include "i2c_lpcxx.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#define LPCI2C_MAX_INTRF			3			// Max number of I2C interface

static LPCI2CDEV g_LpcI2CDev[LPCI2C_MAX_INTRF];


bool LpcI2CWaitInt(LPCI2CDEV *pDev, int Timeout)
{
	while (!(pDev->pI2CReg->I2CONSET & LPCI2C_I2CONSET_SI) && --Timeout > 0);
	if (Timeout > 0)
		return true;

	return false;
}

I2CSTATUS LpcI2CWaitStatus(LPCI2CDEV *pDev, int Timeout)
{
	while ((pDev->pI2CReg->I2STAT & 0xF8) == 0xF8 && --Timeout > 0);

	return (I2CSTATUS)(pDev->pI2CReg->I2STAT & 0xF8);
}

I2CSTATUS LpcI2CGetStatus(LPCI2CDEV *pDev)
{
	return (I2CSTATUS)(pDev->pI2CReg->I2STAT & 0xF8);
}


bool LpcI2CStartCond(LPCI2CDEV *pDev)
{
	I2CSTATUS status;

	// Enable I2C engine
	pDev->pI2CReg->I2CONSET = LPCI2C_I2CONSET_I2EN;
    while (!(pDev->pI2CReg->I2CONSET & LPCI2C_I2CONSET_I2EN));

    pDev->pI2CReg->I2CONSET = LPCI2C_I2CONSET_STA;
    pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;

	LpcI2CWaitInt(pDev, 100000);
	pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_STAC;
	status = LpcI2CWaitStatus(pDev, 100000);
	if (status != I2CSTATUS_START_COND && status != I2CSTATUS_RESTART_COND)
		return false;
	return true;
}

void LpcI2CStopCond(LPCI2CDEV *pDev)
{
	if (pDev->pI2CReg->I2CONSET & LPCI2C_I2CONSET_STA)
		pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_STAC;

	pDev->pI2CReg->I2CONSET = LPCI2C_I2CONSET_STO;
	pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;

	LpcI2CWaitInt(pDev, 100000);
	//LpcI2CWaitStatus(pDev, 100000);

	// Disable I2C engine
	pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_I2ENC;
}


void LpcI2CDisable(DEVINTRF *pDev)
{
	if (pDev && pDev->pDevData)
		((LPCI2CDEV*)pDev->pDevData)->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_I2ENC;
}

void LpcI2CEnable(DEVINTRF *pDev)
{
	if (pDev && pDev->pDevData)
		((LPCI2CDEV*)pDev->pDevData)->pI2CReg->I2CONSET |= LPCI2C_I2CONSET_I2EN;
}

int LpcI2CGetRate(DEVINTRF *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((LPCI2CDEV*)pDev->pDevData)->pI2cDev->Rate;

	return rate;
}

int LpcI2CSetRate(DEVINTRF *pDev, int RateHz)
{
	if (pDev == NULL || pDev->pDevData == NULL)
		return 0;

	LPCI2CDEV *dev = (LPCI2CDEV *)pDev->pDevData;

	// our default clock setting PCLK div 4
	uint32_t clk = (SystemCoreClock >> 2) / RateHz;

	dev->pI2CReg->I2SCLH = clk >> 1;
	dev->pI2CReg->I2SCLL = clk - dev->pI2CReg->I2SCLH;
	dev->pI2cDev->Rate = RateHz;

	return RateHz;
}

bool LpcI2CStartRx(DEVINTRF *pDev, int DevAddr)
{
	if (pDev == NULL || pDev->pDevData == NULL)
		return false;

	LPCI2CDEV *dev = (LPCI2CDEV *)pDev->pDevData;

	if (LpcI2CStartCond(dev))
	{
		// Send I2C address
		dev->pI2CReg->I2DAT = (DevAddr << 1)| 1;
		dev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;
		LpcI2CWaitInt(dev, 100000);
		if (LpcI2CWaitStatus(dev, 100000) == I2CSTATUS_SLAR_ACK)
			return true;
	}

	return false;
}

// Receive Data only, no Start/Stop condition
int LpcI2CRxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen)
{
	I2CSTATUS status;
	int rcount = 0;
	LPCI2CDEV *dev = (LPCI2CDEV *)pDev->pDevData;

	// Start read data
	while (BuffLen > rcount)
	{
		if (rcount < (BuffLen - 1))
			dev->pI2CReg->I2CONSET = LPCI2C_I2CONSET_AA;
		else
			dev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_AAC;
		dev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;

		LpcI2CWaitInt(dev, 100000);
		*pBuff = dev->pI2CReg->I2DAT;
		status = LpcI2CWaitStatus(dev, 100000);
		if (status != I2CSTATUS_RXDATA_ACK && status != I2CSTATUS_RXDATA_NACK)
			break;
		pBuff++;
		rcount++;
	}

	return rcount;
}

// Send Data only, no Start/Stop condition
int LpcI2CTxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	I2CSTATUS status;
	int tcount = 0;
	LPCI2CDEV *dev = (LPCI2CDEV *)pDev->pDevData;

	// Start sending data
	while (DataLen > 0)
	{
		dev->pI2CReg->I2DAT = *pData;
		dev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;
		LpcI2CWaitInt(dev, 100000);
		status = LpcI2CWaitStatus(dev, 100000);
		if (status != I2CSTATUS_M_TXDAT_ACK && status != I2CSTATUS_M_TXDAT_NACK)
			break;
		pData++;
		DataLen--;
		tcount++;
	}

	return tcount;
}

void LpcI2CStopRx(DEVINTRF *pDev)
{
	LpcI2CStopCond((LPCI2CDEV *)pDev->pDevData);
}

bool LpcI2CStartTx(DEVINTRF *pDev, int DevAddr)
{
	if (pDev == NULL || pDev->pDevData == NULL)
		return false;

	LPCI2CDEV *dev = (LPCI2CDEV *)pDev->pDevData;

	if (LpcI2CStartCond(dev))
	{
		if (dev->pI2CReg->I2CONSET & LPCI2C_I2CONSET_STA)
			dev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_STAC;

		// Send I2C address

		dev->pI2CReg->I2DAT = DevAddr << 1;
		dev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;
		LpcI2CWaitInt(dev, 1000);
		if (LpcI2CWaitStatus(dev, 10000) == I2CSTATUS_SLAW_ACK)
			return true;
	}

	return false;
}

void LpcI2CStopTx(DEVINTRF *pDev)
{
	LpcI2CStopCond((LPCI2CDEV *)pDev->pDevData);
}

bool I2CInit(I2CDEV *pDev, const I2CCFG *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	uint32_t clk = (SystemCoreClock >> 2) / pCfgData->Rate;

	// Note : contrary to the user guide.  Pullup resistor is required
	// for I2C to function properly
	// Pin selection for SCL
//	IOPinConfig(pCfgData->SclPortNo, pCfgData->SclPinNo, pCfgData->SclPinOp, IOPINDIR_OUTPUT,
//				IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN);

	// Pin selection for SDA
//	IOPinConfig(pCfgData->SdaPortNo, pCfgData->SdaPinNo, pCfgData->SdaPinOp, IOPINDIR_BI,
//				IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN);

	memcpy(pDev->Pins, pCfgData->Pins, sizeof(IOPINCFG) * I2C_MAX_NB_IOPIN);

	// Configure I/O pins
	IOPinCfg(pCfgData->Pins, I2C_MAX_NB_IOPIN);
    IOPinSet(pCfgData->Pins[I2C_SCL_IOPIN_IDX].PortNo, pCfgData->Pins[I2C_SCL_IOPIN_IDX].PinNo);
    IOPinSet(pCfgData->Pins[I2C_SDA_IOPIN_IDX].PortNo, pCfgData->Pins[I2C_SDA_IOPIN_IDX].PinNo);

	switch (pCfgData->DevNo)
	{
		case 0:
			LPC_SC->PCONP |= LPC_PCONP_I2C0;
			LPC_SC->PCLKSEL0 &= ~LPC_PCLKSEL0_I2C0_MASK; // CCLK / 4
			g_LpcI2CDev[pCfgData->DevNo].pI2CReg = (LPCI2CREG *)LPC_I2C0;
			//NVIC_DisableIRQ(I2C0_IRQn);
			break;
		case 1:
			LPC_SC->PCONP |= LPC_PCONP_I2C1;
			LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_I2C1_MASK; // CCLK / 4
			g_LpcI2CDev[pCfgData->DevNo].pI2CReg = (LPCI2CREG *)LPC_I2C1;
			//NVIC_DisableIRQ(I2C1_IRQn);
			break;
		case 2:
			LPC_SC->PCONP |= LPC_PCONP_I2C2;
			LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_I2C2_MASK; // CCLK / 4
			g_LpcI2CDev[pCfgData->DevNo].pI2CReg = (LPCI2CREG *)LPC_I2C2;
			//NVIC_DisableIRQ(I2C2_IRQn);
			break;
	}
	g_LpcI2CDev[pCfgData->DevNo].I2CNo = pCfgData->DevNo;
	g_LpcI2CDev[pCfgData->DevNo].pI2CReg->I2SCLH = clk >> 1;
	g_LpcI2CDev[pCfgData->DevNo].pI2CReg->I2SCLL = clk - g_LpcI2CDev[pCfgData->DevNo].pI2CReg->I2SCLH;

	g_LpcI2CDev[pCfgData->DevNo].pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_AAC | LPCI2C_I2CONCLR_STAC |
						 	 	 	 	 	 	 	 LPCI2C_I2CONCLR_STOC | LPCI2C_I2CONCLR_I2ENC;
	g_LpcI2CDev[pCfgData->DevNo].pI2cDev = pDev;

	pDev->Mode = pCfgData->Mode;
	pDev->Rate = pCfgData->Rate;
	pDev->NbSlaveAddr = pCfgData->NbSlaveAddr;
	memcpy(pDev->SlaveAddr, pCfgData->SlaveAddr, pDev->NbSlaveAddr * sizeof(uint8_t));
//	pDev->MaxRetry = pCfgData->MaxRetry;

	pDev->DevIntrf.pDevData = (void*)&g_LpcI2CDev[pCfgData->DevNo];

	pDev->DevIntrf.Disable = LpcI2CDisable;
	pDev->DevIntrf.Enable = LpcI2CEnable;
	pDev->DevIntrf.GetRate = LpcI2CGetRate;
	pDev->DevIntrf.SetRate = LpcI2CSetRate;
	pDev->DevIntrf.StartRx = LpcI2CStartRx;
	pDev->DevIntrf.RxData = LpcI2CRxData;
	pDev->DevIntrf.StopRx = LpcI2CStopRx;
	pDev->DevIntrf.StartTx = LpcI2CStartTx;
	pDev->DevIntrf.TxData = LpcI2CTxData;
	pDev->DevIntrf.StopTx = LpcI2CStopTx;
	pDev->DevIntrf.Reset = NULL;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.bBusy = false;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;

	return true;
}

