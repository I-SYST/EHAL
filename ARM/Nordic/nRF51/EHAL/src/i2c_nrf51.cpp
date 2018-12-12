/*--------------------------------------------------------------------------
File   : i2c_nrf51.cpp

Author : Hoang Nguyen Hoan          Oct. 12, 2016

Desc   : I2C implementation on nRF51 series MCU

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
#include "coredev/i2c.h"
#include "iopinctrl.h"

#define NRF51_I2C_MAXDEV        2

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	I2CDEV *pI2cDev;
	NRF_TWI_Type *pReg;	// Register map
} NRF51_I2CDEV;
#pragma pack(pop)

static NRF51_I2CDEV s_nRF51I2CDev[NRF51_I2C_MAXDEV] = {
	{
		0, NULL, NRF_TWI0
	},
	{
		1, NULL, NRF_TWI1
	},
};

bool nRF51I2CWaitStop(NRF51_I2CDEV *pDev, int Timeout)
{
    do {
        if (pDev->pReg->EVENTS_ERROR)
        {
            // Abort in case error
            pDev->pReg->ERRORSRC = pDev->pReg->ERRORSRC;
            pDev->pReg->EVENTS_ERROR = 0;
            pDev->pReg->TASKS_RESUME = 1;
            pDev->pReg->TASKS_STOP = 1;
            while( !pDev->pReg->EVENTS_STOPPED );
            return false;
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

bool nRF51I2CWaitRxComplete(NRF51_I2CDEV *pDev, int Timeout)
{
    do {
        if (pDev->pReg->EVENTS_ERROR)
        {
            while ( !nRF51I2CWaitStop( pDev, Timeout ) );
            return false;
        }
        if (pDev->pReg->EVENTS_RXDREADY)
        {
            pDev->pReg->EVENTS_RXDREADY = 0;

            return true;
        }
    } while (Timeout-- >  0);

    return false;
}

bool nRF51I2CWaitTxComplete(NRF51_I2CDEV *pDev, int Timeout)
{
    do {
        if (pDev->pReg->EVENTS_ERROR)
        {
            while ( !nRF51I2CWaitStop( pDev, Timeout ) );

            return false;
        }
        if (pDev->pReg->EVENTS_TXDSENT)
        {
            // Must wait for last DMA then issue a stop
            pDev->pReg->EVENTS_TXDSENT = 0;

            return true;
        }
    } while (Timeout-- >  0);

    return false;
}

void nRF51I2CDisable(DEVINTRF *pDev)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

	dev->pReg->ENABLE = (TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos);
}
void nRF51I2CEnable(DEVINTRF *pDev)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

	dev->pReg->ENABLE = (TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos);
}

int nRF51I2CGetRate(DEVINTRF *pDev)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

	return dev->pI2cDev->Rate;
}

int nRF51I2CSetRate(DEVINTRF *pDev, int RateHz)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

	if (RateHz < 250000)
	{
		dev->pReg->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K100;
		dev->pI2cDev->Rate = 100000;
	}
	else if (RateHz < 400000)
	{
		dev->pReg->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K250;
		dev->pI2cDev->Rate = 250000;
	}
	else
	{
		dev->pReg->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400;
		dev->pI2cDev->Rate = 400000;
	}

	return dev->pI2cDev->Rate;
}

bool nRF51I2CStartRx(DEVINTRF *pDev, int DevAddr)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

	dev->pReg->ADDRESS = DevAddr;
	dev->pReg->INTENCLR = 0xFFFFFFFF;
	dev->pReg->EVENTS_STOPPED = 0;
	dev->pReg->EVENTS_ERROR = 0;
	dev->pReg->SHORTS = 0;

	return true;
}

// Receive Data only, no Start/Stop condition
int nRF51I2CRxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;
	int cnt = 0;

	if (pBuff == NULL || BuffLen <= 0)
	{
		return 0;
	}

	dev->pReg->EVENTS_STOPPED = 0;
	dev->pReg->TASKS_STARTRX = 1;

	while (BuffLen > 0)
	{
		if (nRF51I2CWaitRxComplete(dev, 100000) == false)
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

void nRF51I2CStopRx(DEVINTRF *pDev)
{
    NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

    dev->pReg->TASKS_STOP = 1;

    // must read dummy last byte to generate NACK & STOP condition
    nRF51I2CWaitRxComplete(dev, 100000);
	uint8_t d __attribute__((unused)) = dev->pReg->RXD;

    nRF51I2CWaitStop(dev, 10000);
}

bool nRF51I2CStartTx(DEVINTRF *pDev, int DevAddr)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

	dev->pReg->ADDRESS = DevAddr;
	dev->pReg->INTENCLR = 0xFFFFFFFF;
	dev->pReg->EVENTS_STOPPED = 0;
	dev->pReg->EVENTS_ERROR = 0;

	return true;
}

// Send Data only, no Start/Stop condition
int nRF51I2CTxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;
	int cnt = 0;

	if (pData == NULL)
	{
		return 0;
	}

	dev->pReg->TASKS_STARTTX = 1;

	while (DataLen > 0)
	{
		dev->pReg->TXD = *pData;

		if (nRF51I2CWaitTxComplete(dev, 10000) == false)
		{
			break;
		}

		DataLen--;
		pData++;
		cnt++;
	}

	return cnt;
}

void nRF51I2CStopTx(DEVINTRF *pDev)
{
    NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

    dev->pReg->TASKS_STOP = 1;
    nRF51I2CWaitStop(dev, 10000);
}

void nRF51I2CReset(DEVINTRF *pDev)
{
    NRF51_I2CDEV *dev = (NRF51_I2CDEV*)pDev->pDevData;

    nRF51I2CDisable(pDev);

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

    nRF51I2CEnable(pDev);
}

bool I2CInit(I2CDEV *pDev, const I2CCFG *pCfgData)
{
	if (pDev == NULL || pCfgData == NULL)
	{
		return false;
	}

	if (pCfgData->DevNo < 0 || pCfgData->DevNo > 2)
	{
		return false;
	}

	// Get the correct register map
	NRF_TWI_Type *reg = s_nRF51I2CDev[pCfgData->DevNo].pReg;

	memcpy(pDev->Pins, pCfgData->Pins, sizeof(IOPINCFG) * I2C_MAX_NB_IOPIN);

	// Configure I/O pins
	IOPinCfg(pCfgData->Pins, I2C_MAX_NB_IOPIN);

    reg->PSELSCL = pCfgData->Pins[I2C_SCL_IOPIN_IDX].PinNo;
    reg->PSELSDA = pCfgData->Pins[I2C_SDA_IOPIN_IDX].PinNo;

    //pDev->MaxRetry = pCfgData->MaxRetry;
    pDev->Mode = pCfgData->Mode;
    pDev->NbSlaveAddr = pCfgData->NbSlaveAddr;
    memcpy(pDev->SlaveAddr, pCfgData->SlaveAddr, pDev->NbSlaveAddr * sizeof(uint8_t));

    s_nRF51I2CDev[pCfgData->DevNo].DevNo = pCfgData->DevNo;
	s_nRF51I2CDev[pCfgData->DevNo].pI2cDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_nRF51I2CDev[pCfgData->DevNo];

	nRF51I2CSetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2C;
	pDev->DevIntrf.Disable = nRF51I2CDisable;
	pDev->DevIntrf.Enable = nRF51I2CEnable;
	pDev->DevIntrf.GetRate = nRF51I2CGetRate;
	pDev->DevIntrf.SetRate = nRF51I2CSetRate;
	pDev->DevIntrf.StartRx = nRF51I2CStartRx;
	pDev->DevIntrf.RxData = nRF51I2CRxData;
	pDev->DevIntrf.StopRx = nRF51I2CStopRx;
	pDev->DevIntrf.StartTx = nRF51I2CStartTx;
	pDev->DevIntrf.TxData = nRF51I2CTxData;
	pDev->DevIntrf.StopTx = nRF51I2CStopTx;
	pDev->DevIntrf.Reset = nRF51I2CReset;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.bBusy = false;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;

	// Clear all errors
    if (reg->EVENTS_ERROR)
    {
        reg->ERRORSRC = reg->ERRORSRC;
        reg->EVENTS_ERROR = 0;
        reg->TASKS_RESUME = 1;
        reg->TASKS_STOP = 1;

    }

    nRF51I2CReset(&pDev->DevIntrf);

	reg->ENABLE = (TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos);

	return true;
}
