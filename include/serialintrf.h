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
#include <stdbool.h>

#pragma pack(push, 4)

typedef struct _serialintrf_dev SERINTRFDEV;
struct _serialintrf_dev {
	void *pDevData;		// Private device interface implementation data
	void (*Disable)(SERINTRFDEV *pSerDev);
	void (*Enable)(SERINTRFDEV *pSerDev);
	int (*GetRate)(SERINTRFDEV *pSerDev);
	int (*SetRate)(SERINTRFDEV *pSerDev, int Rate);
	bool (*StartRx)(SERINTRFDEV *pSerDev, int DevAddr);
	int (*RxData)(SERINTRFDEV *pSerDev, uint8_t *pData, int DataLen);
	void (*StopRx)(SERINTRFDEV *pSerDev);
	bool (*StartTx)(SERINTRFDEV *pSerDev, int DevAddr);
	int (*TxData)(SERINTRFDEV *pSerDev, uint8_t *pData, int DataLen);
	void (*StopTx)(SERINTRFDEV *pSerDev);
};

#pragma pack(pop)

// C only function prototypes
static inline void SerialIntrfDisable(SERINTRFDEV *pDev) {
	pDev->Disable(pDev);
}

static inline void SerialIntrfEnable(SERINTRFDEV *pDev) {
	pDev->Enable(pDev);
}

static inline int SerialIntrfGetRate(SERINTRFDEV *pDev) {
	return pDev->GetRate(pDev);
}

static inline int SerialIntrfSetRate(SERINTRFDEV *pDev, int Rate) {
	return pDev->SetRate(pDev, Rate);
}

static inline int SerialIntrfRx(SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen) {
	int retval = 0;

	if (pDev->StartRx(pDev, DevAddr)) {
		retval = pDev->RxData(pDev, pBuff, BuffLen);
		pDev->StopRx(pDev);
	}

	return retval;
}

static inline int SerialIntrfTx(SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen) {
	int retval = 0;

	if (pDev->StartTx(pDev, DevAddr)) {
		retval = pDev->TxData(pDev, pBuff, BuffLen);
		pDev->StopTx(pDev);
	}
	return retval;
}

static inline bool SerialIntrfStartRx(SERINTRFDEV *pDev, int DevAddr) {
	return pDev->StartRx(pDev, DevAddr);
}

static inline int SerialIntrfRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->RxData(pDev, pBuff, BuffLen);
}

static inline void SerialIntrfStopRx(SERINTRFDEV *pDev) {
	pDev->StopRx(pDev);
}

static inline bool SerialIntrfStartTx(SERINTRFDEV *pDev, int DevAddr) {
	return pDev->StartTx(pDev, DevAddr);
}

static inline int SerialIntrfTxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->TxData(pDev, pBuff, BuffLen);
}

static inline void SerialIntrfStopTx(SERINTRFDEV *pDev) {
	pDev->StopTx(pDev);
}


#ifdef __cplusplus

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
