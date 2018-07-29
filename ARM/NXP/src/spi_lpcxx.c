/*--------------------------------------------------------------------------
File   : spi_lpcxx.c

Author : Hoang Nguyen Hoan          Dec. 25, 2011

Desc   : Synchronous Serial Port(SSP) implementation on LPC series MCU

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
Modified by         Date            Description
Hoan				Feb. 20, 2015	New EHAL
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "coredev/iopincfg.h"
#include "spi_lpcxx.h"

extern uint32_t SystemCoreClock;

int LpcSSPGetRate(DEVINTRF *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((SSPDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
int LpcSSPSetRate(DEVINTRF *pDev, int DataRate)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;
	int div = dev->pSspReg->CPSR & 0xff;
	int srate = 0;

	if (div == 0)
		div = 1;

	srate = DataRate * div;
	//srate = ((SystemCoreClock >> 1) + srate) / srate - 1;
	srate = (dev->PClkFreq + (srate >> 1)) / srate - 1;
	dev->pSspReg->CR0 &= ~LPCSSP_CR0_SCR_MASK;
	dev->pSspReg->CR0 |=  srate << 8;
	dev->pSpiDev->Cfg.Rate = dev->PClkFreq / ((srate + 1)* div);

	//printf("srate = %d, vRate = %d, div = %x\n\r", srate, vRate, div);

	return dev->pSpiDev->Cfg.Rate;
}

// Initial receive
bool LpcSSPStartRx(DEVINTRF *pDev, int DevAddr)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;

	LpcSSPWaitBusy(dev, 100000);

	// Clear interrupt status
	dev->pSspReg->ICR = 3;

	return true;
}

// Receive Data only, no Start/Stop condition
int LpcSSPRxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;
	int cnt = 0;
	bool bw = true;
	//uint16_t *p16 = (uint16_t *)pBuff;
	uint16_t d = 0xff;

	if (dev->pSspReg->CR0 & 0x8)
	{
		// 9-16 bits data
		bw = false;
		d = 0xffff;
	}

	while (BuffLen > cnt) 
	{
		dev->pSspReg->DR = d;
		//for (int i = 10; i > 0; i--)
		//	__NOP();
		if (LpcSSPWaitRxFifo(dev, 100000))
		{
			if (bw) 
			{
				*pBuff = dev->pSspReg->DR & 0xff;
				pBuff++;
				cnt++;
			} 
			else
			{
				d = dev->pSspReg->DR & LPCSSP_DR_MASK;
				pBuff[0] = d & 0xff;
				pBuff[1] = (d >> 8) & 0xff;
				cnt+=2;
				pBuff+=2;
			}
		}
	}

	return cnt;
}

// Stop receive
void LpcSSPStopRx(DEVINTRF *pDev)
{

}

// Receive stream
int LpcSSPRx(DEVINTRF *pDev, int DevAddr, uint8_t *pBuff, int BuffLen)
{
	int cnt = 0;

	if (LpcSSPStartRx(pDev, DevAddr))
	{
		cnt = LpcSSPRxData(pDev, pBuff, BuffLen);
		LpcSSPStopRx(pDev);
	}

	return cnt;
}


// Initiate transmit
bool LpcSSPStartTx(DEVINTRF *pDev, int DevAddr)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;

	LpcSSPWaitBusy(dev, 100000);
	// Clear interrupt status
	dev->pSspReg->ICR = 3;

	return true;
}

// Transmit Data only, no Start/Stop condition
int LpcSSPTxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	SSPDEV *dev = (SSPDEV *)pDev-> pDevData;
	int cnt = 0;
	bool bw = true;	// byte width

	if (dev->pSspReg->CR0 & 0x8)
	{
		// 9-16 bits data
		bw = false;
	}

	while (DataLen > 0)//cnt)
	{
		if (LpcSSPWaitTxFifo(dev, 100000))
		{
			if (bw) 
			{
				dev->pSspReg->DR = *pData & 0xff;
				pData++;
				cnt++;
				DataLen--;

			}
			else
			{
				dev->pSspReg->DR = *(uint16_t *)pData & LPCSSP_DR_MASK;
				pData += 2;
				cnt += 2;
				DataLen -= 2;
			}
			//if (LpcSSPWaitRxFifo(dev, 100000))
			{
				int d = dev->pSspReg->DR;
			}
		}
	}

	return cnt;
}

// Stop transmit
void LpcSSPStopTx(DEVINTRF *pDev)
{

}

// Transmit stream
int LpcSSPTx(DEVINTRF *pDev, int DevAddr, uint8_t *pData, int DataLen)
{
	int cnt = 0;

	if (LpcSSPStartTx(pDev, DevAddr))
	{
		cnt = LpcSSPTxData(pDev, pData, DataLen);
		LpcSSPStopTx(pDev);
	}

	return cnt;
}

bool LpcSSPWaitBusy(SSPDEV *pDev, int TimeoutCnt)
{
	do {
		if ((pDev->pSspReg->SR & LPCSSP_SR_BSY) == 0)
			return true;
	} while (--TimeoutCnt > 0);

	return false;
}

bool LpcSSPWaitRxFifo(SSPDEV *pDev, int TimeoutCnt)
{
	do
	{
		if (pDev->pSspReg->SR & LPCSSP_SR_RNE)
			return true;
	} while (--TimeoutCnt > 0);

	return false;
}

bool LpcSSPWaitTxFifo(SSPDEV *pDev, int TimeoutCnt)
{
	do
	{
		if (pDev->pSspReg->SR & LPCSSP_SR_TNF)
			return true;
	} while (--TimeoutCnt > 0);

	return false;
}


