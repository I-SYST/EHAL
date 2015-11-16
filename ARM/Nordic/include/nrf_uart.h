/*--------------------------------------------------------------------------
File   : nrf_uart.h

Author : Hoang Nguyen Hoan          Aug. 30, 2015

Desc   : nRF5x UART implementation

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
Modified by          Date              Description

----------------------------------------------------------------------------*/
#ifndef __NRF_UART_H__
#define __NRF_UART_H__

#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "uart.h"

// Device driver data require by low level functions
typedef struct _nRF_UART_Dev {
	int DevNo;				// UART interface number
	NRF_UART_Type *pReg;	// UART registers
	UARTDEV	*pUartDev;		// Pointer to generic UART dev. data
	bool bTxReady;
	uint32_t RxPin;
	uint32_t TxPin;
	uint32_t CtsPin;
	uint32_t RtsPin;
} NRFUARTDEV;


#ifdef __cplusplus
extern "C" {
#endif
void nRFUARTDisable(SERINTRFDEV *pDev);
void nRFUARTEnable(SERINTRFDEV *pDev);
static inline int nRFUARTGetRate(SERINTRFDEV *pDev) { return ((NRFUARTDEV*)pDev->pDevData)->pUartDev->Rate; }
int nRFUARTSetRate(SERINTRFDEV *pDev, int Rate);
static inline bool nRFUARTStartRx(SERINTRFDEV *pSerDev, int DevAddr) { return true; }
int nRFUARTRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int Bufflen);
static inline void nRFUARTStopRx(SERINTRFDEV *pSerDev) {}
static inline bool nRFUARTStartTx(SERINTRFDEV *pDev, int DevAddr) {
//	((NRFUARTDEV*)pDev->pDevData)->pReg->TASKS_STARTTX = 1;
	return true;
}
int nRFUARTTxData(SERINTRFDEV *pDev, uint8_t *pData, int Datalen);
static inline void nRFUARTStopTx(SERINTRFDEV *pDev) {	}//((NRFUARTDEV*)pDev->pDevData)->pReg->TASKS_STOPTX = 1; }

#ifdef __cplusplus
}
/*
class nRFUart : public UART {
public:
	nRFUart();
	virtual ~nRFUart();

	bool Init(const UARTCFG &CfgData) { return UARTInit(vDev.pUartDev, &CfgData); }
	// Set data baudrate
	virtual int Rate(int DataRate) { vDev.pUartDev->Rate = NRFUARTSetRate(&vDev, DataRate); return vDev.Rate; }
	// Get current data baudrate
	virtual int Rate(void) { return vDev.Rate; }
	virtual int Rx(uint8_t *pBuff, uint32_t Len);
	// Stop receive
	virtual int Tx(uint8_t *pData, uint32_t Len);
	// Initiate transmit
	virtual bool StartTx(int DevAddr);
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen);
	// Stop transmit
	virtual void StopTx(void);

private:
	UARTDEV vDev;
};
*/
#endif	// __cplusplus

#endif // __NRF_UART_H__

