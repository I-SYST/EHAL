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

#include "system_core_clock.h"

#include "i2c_lpcxx.h"
#include "coredev/iopincfg.h"

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */

/*
bool LpcI2CInit(I2CDEV *pDev, I2CCFG *pCfgData)
{
	uint32_t clk = (SystemCoreClock >> 2) / pCfgData->Rate;

	// Note : contrary to the user guide.  Pullup resistor is required
	// for I2C to function properly
	// Pin selection for SCL
	IOPinConfig(pCfgData->SclPortNo, pCfgData->SclPinNo, pCfgData->SclPinOp,
				IOPIN_RES_PULLUP, IOPIN_MODE_OPENDRAIN);

	// Pin selection for SDA
	IOPinConfig(pCfgData->SdaPortNo, pCfgData->SdaPinNo, pCfgData->SdaPinOp,
				IOPIN_RES_PULLUP, IOPIN_MODE_OPENDRAIN);

	switch (pCfgData->I2cNo)
	{
		case 0:
			LPC_SC->PCONP |= LPC_PCONP_I2C0;
			LPC_SC->PCLKSEL0 &= ~LPC_PCLKSEL0_I2C0_MASK; // CCLK / 4
			pDev->pI2CReg = LPC_I2C0;
			//NVIC_DisableIRQ(I2C0_IRQn);
			break;
		case 1:
			LPC_SC->PCONP |= LPC_PCONP_I2C1;
			LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_I2C1_MASK; // CCLK / 4
			pDev->pI2CReg = LPC_I2C1;
			//NVIC_DisableIRQ(I2C1_IRQn);
			break;
		case 2:
			LPC_SC->PCONP |= LPC_PCONP_I2C2;
			LPC_SC->PCLKSEL1 &= ~LPC_PCLKSEL1_I2C2_MASK; // CCLK / 4
			pDev->pI2CReg = LPC_I2C2;
			//NVIC_DisableIRQ(I2C2_IRQn);
			break;
	}

	pDev->pI2CReg->I2SCLH = clk >> 1;
	pDev->pI2CReg->I2SCLL = clk - pDev->pI2CReg->I2SCLH;

	pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_AAC | LPCI2C_I2CONCLR_STAC |
						 	 LPCI2C_I2CONCLR_STOC | LPCI2C_I2CONCLR_I2ENC;

	pDev->Mode = pCfgData->Mode;
	pDev->Rate = pCfgData->Rate;
	pDev->SlaveAddr = pCfgData->SlaveAddr;
	pDev->MaxRetry = pCfgData->MaxRetry;

	return true;
}
*/
void LpcI2CDisable(DEVINTRF *pDev)
{
	LPCI2CDEV *dev = (LPCI2CDEV*)pDev->pDevData;

	if (dev->pI2CReg)
	{
		dev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_I2ENC;
	}

}
void LpcI2CEnable(DEVINTRF *pDev)
{
	LPCI2CDEV *dev = (LPCI2CDEV*)pDev->pDevData;

	if (dev->pI2CReg)
		dev->pI2CReg->I2CONSET |= LPCI2C_I2CONSET_I2EN;
}

int LpcI2CGetRate(DEVINTRF *pDev)
{
	LPCI2CDEV *dev = (LPCI2CDEV*)pDev->pDevData;

	return dev->pI2cDev->Rate;
}

int LpcI2CSetRate(DEVINTRF *pDev, int RateHz)
{
	LPCI2CDEV *dev = (LPCI2CDEV*)pDev->pDevData;

	if (dev->pI2CReg == NULL)
		return 0;

	// our default clock setting PCLK div 4
	uint32_t clk = (SystemCoreClock >> 2) / RateHz;

	dev->pI2cDev->Rate = RateHz;
	dev->pI2CReg->I2SCLH = clk >> 1;
	dev->pI2CReg->I2SCLL = clk - pDev->pI2CReg->I2SCLH;

	return pDev->Rate;
}

I2CSTATUS LpcI2CGetStatus(I2CDEV *pDev)
{
	return (I2CSTATUS)(pDev->pI2CReg->I2STAT & 0xF8);
}

bool LpcI2CStartCond(I2CDEV *pDev)
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

void LpcI2CStopCond(I2CDEV *pDev)
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

bool LpcI2CStartRx(SERINTRFDEV *pDev, int DevAddr)
{
	if (LpcI2CStartCond(pDev))
	{
		// Send I2C address
		pDev->pI2CReg->I2DAT = (DevAddr << 1)| 1;
		pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;
		LpcI2CWaitInt(pDev, 100000);
		if (LpcI2CWaitStatus(pDev, 100000) == I2CSTATUS_SLAR_ACK)
			return true;
	}

	return false;
}

// Receive Data only, no Start/Stop condition
int LpcI2CRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen)
{
	I2CSTATUS status;
	int rcount = 0;

	// Start read data
	while (BuffLen > rcount)
	{
		if (rcount < (BuffLen - 1))
			pDev->pI2CReg->I2CONSET = LPCI2C_I2CONSET_AA;
		else
			pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_AAC;
		pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;

		LpcI2CWaitInt(pDev, 100000);
		*pBuff = pDev->pI2CReg->I2DAT;
		status = LpcI2CWaitStatus(pDev, 100000);
		if (status != I2CSTATUS_RXDATA_ACK && status != I2CSTATUS_RXDATA_NACK)
			break;
		pBuff++;
		rcount++;
	}

	return rcount;
}

// Send Data only, no Start/Stop condition
int LpcI2CTxData(SERINTRFDEV *pDev, uint8_t *pData, int DataLen)
{
	I2CSTATUS status;
	int tcount = 0;

	// Start sending data
	while (DataLen > 0)
	{
		pDev->pI2CReg->I2DAT = *pData;
		pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;
		LpcI2CWaitInt(pDev, 100000);
		status = LpcI2CWaitStatus(pDev, 100000);
		if (status != I2CSTATUS_M_TXDAT_ACK && status != I2CSTATUS_M_TXDAT_NACK)
			break;
		pData++;
		DataLen--;
		tcount++;
	}

	return tcount;
}

void LpcI2CStopRx(SERINTRFDEV *pDev)
{
	LpcI2CStopCond(pDev);
}

bool LpcI2CStartTx(SERINTRFDEV *pDev, int DevAddr)
{
	if (LpcI2CStartCond(pDev))
	{
		if (pDev->pI2CReg->I2CONSET & LPCI2C_I2CONSET_STA)
			pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_STAC;

		// Send I2C address

		pDev->pI2CReg->I2DAT = DevAddr << 1;
		pDev->pI2CReg->I2CONCLR = LPCI2C_I2CONCLR_SIC;
		LpcI2CWaitInt(pDev, 10000);
		if (LpcI2CWaitStatus(pDev, 10000) == I2CSTATUS_SLAW_ACK)
			return true;
	}

	return false;
}

void LpcI2CStopTx(SERINTRFDEV *pDev)
{
	LpcI2CStopCond(pDev);
}

bool LpcI2CWaitInt(I2CDEV *pDev, int Timeout)
{
	while (!(pDev->pI2CReg->I2CONSET & LPCI2C_I2CONSET_SI) && --Timeout > 0);
	if (Timeout > 0)
		return true;

	return false;
}

I2CSTATUS LpcI2CWaitStatus(I2CDEV *pDev, int Timeout)
{
	while ((pDev->pI2CReg->I2STAT & 0xF8) == 0xF8 && --Timeout > 0);

	return (I2CSTATUS)(pDev->pI2CReg->I2STAT & 0xF8);
}
/*
LpcI2C::LpcI2C()
{
	memset((void*)&vDevData, 0, (int)sizeof(vDevData));
}

LpcI2C::~LpcI2C()
{
	LpcI2CDisable(&vDevData);
}
*/
