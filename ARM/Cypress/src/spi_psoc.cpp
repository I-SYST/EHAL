/**-------------------------------------------------------------------------
@file   spi_psoc.cpp

@brief  SPI implementation on Cypress PSoC


@author Hoang Nguyen Hoan          
@date   Feb. 10, 2018

@license

Copyright (c) 2018, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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

----------------------------------------------------------------------------*/
#include "spi.h"
//#include "SPIM.h"
//#include "cs.h"

extern "C" {
void  SPIM_Init(void);
void  SPIM_Enable(void);
void  SPIM_Start(void);
void  SPIM_Stop(void)                           ;

void  SPIM_EnableTxInt(void)                    ;
void  SPIM_EnableRxInt(void)                    ;
void  SPIM_DisableTxInt(void)                   ;
void  SPIM_DisableRxInt(void)                   ;

void  SPIM_Sleep(void)                          ;
void  SPIM_Wakeup(void)                         ;
void  SPIM_SaveConfig(void)                     ;
void  SPIM_RestoreConfig(void)                  ;

void  SPIM_SetTxInterruptMode(uint8_t intSrc)     ;
void  SPIM_SetRxInterruptMode(uint8_t intSrc)     ;
uint8_t SPIM_ReadTxStatus(void)                   ;
uint8_t SPIM_ReadRxStatus(void);
void  SPIM_WriteTxData(uint8_t txData);
uint8_t SPIM_ReadRxData(void);
uint8_t SPIM_GetRxBufferSize(void)                ;
uint8_t SPIM_GetTxBufferSize(void)                ;
void  SPIM_ClearRxBuffer(void)                  ;
void  SPIM_ClearTxBuffer(void)                  ;
void  SPIM_ClearFIFO(void)                              ;
void  SPIM_PutArray(const uint8_t buffer[], uint8_t byteCount);
}

typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	uint32_t Clk;
} CY_SPIDEV;

static CY_SPIDEV s_CySpiDev;

int CySPIGetRate(DEVINTRF *pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((CY_SPIDEV*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

// Set data rate in bits/sec (Hz)
// return actual rate
int CySPISetRate(DEVINTRF *pDev, int DataRate)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev->pDevData;

    dev->Clk = DataRate;
	dev->pSpiDev->Cfg.Rate = dev->Clk;

	return dev->pSpiDev->Cfg.Rate;
}

void CySPIDisable(DEVINTRF *pDev)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev->pDevData;

}

void CySPIEnable(DEVINTRF *pDev)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev->pDevData;

}

// Initial receive
bool CySPIStartRx(DEVINTRF *pDev, int DevCs)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev->pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;

    cs_Write(0 << DevCs);
    
	return true;
}

// Receive Data only, no Start/Stop condition
int CySPIRxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	while (BuffLen > 0)
	{
        *pBuff = SPIM_ReadRxData();
        
		BuffLen--;
		pBuff++;
		cnt++;
	}

	return cnt;
}

// Stop receive
void CySPIStopRx(DEVINTRF *pDev)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev-> pDevData;

    cs_Write(1 << dev->pSpiDev->CurDevCs);
}

// Initiate transmit
bool CySPIStartTx(DEVINTRF *pDev, int DevCs)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev-> pDevData;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - SPI_SS_IOPIN_IDX)
		return false;

	dev->pSpiDev->CurDevCs = DevCs;

    cs_Write(0 << DevCs);
    
	return true;
}

// Transmit Data only, no Start/Stop condition
int CySPITxData(DEVINTRF *pDev, uint8_t *pData, int DataLen)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev-> pDevData;
	int cnt = 0;

	while (DataLen > 0)
	{
        SPIM_WriteTxData(*pData);
		DataLen--;
		pData++;
		cnt++;
	}

	return cnt;
}

// Stop transmit
void CySPIStopTx(DEVINTRF *pDev)
{
	CY_SPIDEV *dev = (CY_SPIDEV *)pDev-> pDevData;

    cs_Write(1 << dev->pSpiDev->CurDevCs);
}

void CySPIReset(DEVINTRF *pDev)
{
}

bool SPIInit(SPIDEV *pDev, const SPICFG *pCfgData)
{
    if (pDev == NULL || pCfgData == NULL)
        return false;
    
	if (pCfgData->DevNo < 0 || pCfgData->DevNo > 0 || pCfgData->NbIOPins < 3)
		return false;

    SPIM_Start();
    
	pDev->Cfg = *pCfgData;
    
	for (int i = SPI_SS_IOPIN_IDX, j = 0; i < pCfgData->NbIOPins; i++, j++)
	{
		cs_Write(1 << j);
	}

	s_CySpiDev.pSpiDev  = pDev;
	pDev->DevIntrf.pDevData = (void*)&s_CySpiDev;

	CySPISetRate(&pDev->DevIntrf, pCfgData->Rate);

	pDev->DevIntrf.Disable = CySPIDisable;
	pDev->DevIntrf.Enable = CySPIEnable;
	pDev->DevIntrf.GetRate = CySPIGetRate;
	pDev->DevIntrf.SetRate = CySPISetRate;
	pDev->DevIntrf.StartRx = CySPIStartRx;
	pDev->DevIntrf.RxData = CySPIRxData;
	pDev->DevIntrf.StopRx = CySPIStopRx;
	pDev->DevIntrf.StartTx = CySPIStartTx;
	pDev->DevIntrf.TxData = CySPITxData;
	pDev->DevIntrf.StopTx = CySPIStopTx;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
    pDev->DevIntrf.Reset = CySPIReset;
	pDev->DevIntrf.Busy = false;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;

    return true;
}
