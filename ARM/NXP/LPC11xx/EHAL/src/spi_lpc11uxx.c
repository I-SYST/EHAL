/*--------------------------------------------------------------------------
File   : spi_lpc11uxx.h

Author : Hoang Nguyen Hoan          Feb. 20, 2015

Desc   : SSP/SPI implementation on LPC11Uxx
		 Current implementation
		 	 Master mode
		 	 Polling

Copyright (c) 2015, I-SYST inc., all rights reserved

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
#include "LPC11Uxx.h"

#include "spi_lpcxx.h"

#define LPC11U_SYSAHBCLKCTRL_SSP0_EN		(1 << 11)
#define LPC11U_SYSAHBCLKCTRL_SSP1_EN		(1 << 18)
#define LPC11U_SYSAHBCLKCTRL_GPIO_EN		(1 << 6)

#define LPC11U_PRESETCTRL_SSP0_RST			(1 << 0)
#define LPC11U_PRESETCTRL_SSP1_RST			(1 << 2)

#define LPC11U_MAX_SSPDEV		2

SSPDEV g_SspDev[LPC11U_MAX_SSPDEV] = {
		{0, 0, (LPCSSPREG*)LPC_SSP0,},
		{1, 0, (LPCSSPREG*)LPC_SSP1,},
};

bool SPIInit(SPIDEV *pDev, const SPICFG *pCfgData)
{
	SSPDEV *dev = NULL;

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

	if (pCfgData->DevNo == 0)
	{
		LPC_SYSCON->PRESETCTRL |= LPC11U_PRESETCTRL_SSP0_RST;
		LPC_SYSCON->SYSAHBCLKCTRL |= LPC11U_SYSAHBCLKCTRL_SSP0_EN;
		LPC_SYSCON->SSP0CLKDIV = 1;//0x2;			// Divided by 1

		dev = &g_SspDev[0];
		dev->pSspReg = (LPCSSPREG*)LPC_SSP0;
	}
	else if (pCfgData->DevNo == 1)
	{
		LPC_SYSCON->PRESETCTRL |= LPC11U_PRESETCTRL_SSP1_RST;
		LPC_SYSCON->SYSAHBCLKCTRL |= LPC11U_SYSAHBCLKCTRL_SSP1_EN;
		LPC_SYSCON->SSP1CLKDIV = 1;//0x02;			// Divided by 1

		dev = &g_SspDev[1];
		dev->pSspReg = (LPCSSPREG*)LPC_SSP1;
	}

	dev->PClkFreq = SystemCoreClock / LPC_SYSCON->SSP1CLKDIV;

	uint32_t d = (pCfgData->DataSize - 1);
	if (pCfgData->DataPhase == SPIDATAPHASE_SECOND_CLK)
		d |= LPCSSP_CR0_CPHA_SECOND;

	if (pCfgData->ClkPol == SPICLKPOL_HIGH)
		d |= LPCSSP_CR0_CPOL_HI;

	dev->pSspReg->CR0 = d | LPCSSP_CR0_FRF_SPI;

	d = LPCSSP_CR1_SSP_EN;

	if (pCfgData->Mode == SPIMODE_SLAVE)
		d |= LPCSSP_CR1_MS_SLAVE;

	dev->pSspReg->CR1 = d;
	dev->pSspReg->CPSR = 2;
	/* Enable AHB clock to the GPIO domain. */
	LPC_SYSCON->SYSAHBCLKCTRL |= LPC11U_SYSAHBCLKCTRL_GPIO_EN;
	//LPC_SSP1->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM;

	// Flush RX FIFO
	while (LpcSSPWaitRxFifo(dev, 2))
	{
		uint32_t d = dev->pSspReg->DR;
	}

	dev->pSpiDev = pDev;
	pDev->SerIntrf.pDevData = (void*)dev;
	pDev->Cfg = *pCfgData;
	pDev->SerIntrf.Disable = LpcSSPDisable;
	pDev->SerIntrf.Enable = LpcSSPEnable;
	pDev->SerIntrf.GetRate = LpcSSPGetRate;
	pDev->SerIntrf.SetRate = LpcSSPSetRate;
	pDev->SerIntrf.StartRx = LpcSSPStartRx;
	pDev->SerIntrf.RxData = LpcSSPRxData;
	pDev->SerIntrf.StopRx = LpcSSPStopRx;
	pDev->SerIntrf.StartTx = LpcSSPStartTx;
	pDev->SerIntrf.TxData = LpcSSPTxData;
	pDev->SerIntrf.StopTx = LpcSSPStopTx;
	pDev->SerIntrf.Busy = false;
	pDev->SerIntrf.MaxRetry = 0;

	LpcSSPSetRate(&pDev->SerIntrf, pCfgData->Rate);

	return true;
}




