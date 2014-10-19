/*--------------------------------------------------------------------------
File   : serialintrf.h

Author : Hoang Nguyen Hoan          Nov. 25, 2011

Desc   : Generic serial interface class
		 This class is used to implement serial communication interfaces
		 such as I2C, UART, etc...  Not limited to wired interface

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
#ifndef __SERIALINTRF_H__
#define __SERIALINTRF_H__

#include <stdint.h>

typedef struct _serialintrf_dev {
	void *hSerDev;		// Interface device data
	bool (*Init)(void *hSerDev, void *pCfgData);
	int (*GetRate)(void *hSerDev);
	int (*SetRate)(void *hSerDev, int Rate);
	int (*StartRx)(void *hSerDev, int DevAddr);
	int (*RxData)(void *hSerDev, int DevAddr, uint8_t *pData, int DataLen);
	void (*StopRx)(void *hSerDev, int DevAddr);
	int (*StartTx)(void *hSerDev, int DevAddr);
	int (*TxData)(void *hSerDev, int DevAddr, uint8_t *pData, int DataLen);
	void (*StopTx)(void *hSerDev, int DevAddr);
} SERINTRFDEV;

#ifdef __cplusplus
extern "C" {
#endif

// C only function prototypes
inline __attribute__((always_inline)) int SerialIntrfGetRate(SERINTRFDEV *pDev) {
	return pDev->GetRate(pDev->hSerDev);
}

inline __attribute__((always_inline)) int SerialIntrfSetRate(
			SERINTRFDEV *pDev, int Rate) {
	return pDev->SetRate(pDev->hSerDev, Rate);
}

inline __attribute__((always_inline)) int SerialIntrfRx(
			SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen) {
	int retval = 0;

	pDev->StartRx(pDev->hSerDev, DevAddr);
	retval = pDev->RxData(pDev->hSerDev, DevAddr, pBuff, BuffLen);
	pDev->StopRx(pDev->hSerDev, DevAddr);

	return retval;
}

inline __attribute__((always_inline)) int SerialIntrfTx(
		SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen) {
	int retval = 0;

	pDev->StartTx(pDev->hSerDev, DevAddr);
	retval = pDev->TxData(pDev->hSerDev, DevAddr, pBuff, BuffLen);
	pDev->StopTx(pDev->hSerDev, DevAddr);

	return retval;
}

#ifdef __cplusplus
}

class SerialIntrf {
public:
	virtual ~SerialIntrf() {}
	// Set data rate in bits/sec (Hz)
	virtual int Rate(int DataRate) = 0;
	// Get current data rate in bits/sec (Hz)
	virtual int Rate(void) = 0;
	// Transmit full frame
	virtual int Tx(int DevAddr, uint8_t *pData, int DataLen);
	// Receive full frame
	virtual int Rx(int DevAddr, uint8_t *pBuff, int BuffLen);
	// Initiate receive
	virtual bool StartRx(int DevAddr) = 0;
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) = 0;
	// Stop receive
	virtual void StopRx(void) = 0;
	// Initiate transmit
	virtual bool StartTx(int DevAddr) = 0;
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) = 0;
	// Stop transmit
	virtual void StopTx(void) = 0;
};
#endif

#endif	// __SERIALCOM_H__
