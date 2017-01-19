/*--------------------------------------------------------------------------
File   : serialintrf.h

Author : Hoang Nguyen Hoan          				Nov. 25, 2011

Desc   : Generic serial data transfer interface class
		 This class is used to implement serial communication interfaces
		 such as I2C, UART, etc...  Not limited to wired or
		 physical interface.  It could be soft interface as well such
		 as SLIP protocol or any mean of transferring data between 2
		 entities.

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
#include "atomic.h"

/*
 * Serial interface event types
 */
typedef enum {
	SERINTRF_EVT_RX_TIMEOUT,    // Rx timeout
	SERINTRF_EVT_RX_DATA,       // Data received
	SERINTRF_EVT_RX_FIFO_FULL,  // Receive fifo full, fifo will be pushed out
                                // if handler does not process fifo (returns 0)
	SERINTRF_EVT_TX_TIMEOUT,    // Tx timeout
	SERINTRF_EVT_TX_READY,      // Ready to transmit
	SERINTRF_EVT_TX_FIFO_FULL,  // Transmit fifo full, fifo will be pushed out
                                // if handler does not process fifo (returns 0)
	SERINTRF_EVT_STATECHG,      // State changed. State data is device dependent.
                                // To be interpreted by implementation
} SERINTRF_EVT;

/*
 * Serial Interface forward data structure type definition.
 * This structure is the base object.  Pointer to an instance of this is passed
 * to all function calls.  See structure definition bellow for more details
 */
typedef struct _serialintrf_dev SERINTRFDEV;


/**
 * @brief
 *
 * Event handler callback. This is normally being called within interrupts, avoid blocking
 *
 * @param 	pDev 	: Device handle
 * 			EvtId 	: Event code
 *  		pBuffer : In/Out Buffer containing data
 * 					  on SERINTRF_EVT_RX_TIMEOUT & SERINTRF_EVT_RXDATA, pBuffer contains data received. If
 * 					  driver implements CFIFO, this parameter is NULL with BufferLen indicating total data
 * 					  in fifo.
 * 					  on SERINTRF_EVT_TX_READY, pBuffer contains data to be transmit with max length
 * 					  BufferLen. If driver implements CFIFO, this parameter is NULL and BufferLen
 * 					  indicates amount of data stored in fifo
 * 					  on SERINTRF_EVT_STATECHG, pBuffer contains state data. This is implementation specific
 * 					  for example UART implementation would contains line state info.
 *
 * 			BufferLen : Max buffer length.  See above description
 *
 * @return number of bytes processed.  Implementation specific
 *         in case of FIFO_FULL events,  fifo will be pushed out if return value is zero
 */
typedef int (*SERINTRFEVCB)(SERINTRFDEV *pDev, SERINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#pragma pack(push, 4)

/*
 * Serial interface data structure.  This structure is the actual interface for both C++ & C code
 * It is used to provide C compatibility instead of using C++ interface which is only for C++
 */
struct _serialintrf_dev {
	void *pDevData;			// Private device interface implementation data
	int	IntPrio;			// Interrupt priority.  Value is implementation specific
	SERINTRFEVCB EvtCB;		// Interrupt based event callback function pointer. Must be set to NULL if not used
	volatile bool Busy;		// Busy flag to be set check and set at start and reset at end of transmission
	int MaxRetry;			// Max retry when data could not be transfered (Rx/Tx returns zero count)

	// Bellow are all mandatory functions to implement
	// On init, all implementation must fill these function, no NULL allowed
	// If a function is not used. It must be implemented as do nothing function

	/**
	 * @brief - Disable
	 * 		Turn off the interface.  If this is a physical interface, provide a
	 * way to turn off for energy saving. Make sure the turn off procedure can
	 * be turned back on without going through the full init sequence
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 *
	 * @return None
	 */
	void (*Disable)(SERINTRFDEV *pSerDev);

	/**
	 * @brief - Enable
	 * 		Turn on the interface.
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 *
	 * @return None
	 */
	void (*Enable)(SERINTRFDEV *pSerDev);

	/**
	 * @brief - GetRate
	 * 		Get data rate of the interface in Hertz.  This is not a clock frequency
	 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
	 * implementation as bits/sec or bytes/sec or whatever the case
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 *
	 * @return Transfer rate per second
	 */
	int (*GetRate)(SERINTRFDEV *pSerDev);

	/**
	 * @brief - SetRate
	 * 		Set data rate of the interface in Hertz.  This is not a clock frequency
	 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
	 * implementation as bits/sec or bytes/sec or whatever the case
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 * 		Rate 	: Data rate to be set in Hertz (transfer per second)
	 *
	 * @return 	Actual transfer rate per second set.  It is the real capable rate
	 * 			closes to rate being requested.
	 */
	int (*SetRate)(SERINTRFDEV *pSerDev, int Rate);

	/**
	 * @brief - StartRx
	 * 		Prepare start condition to receive data with subsequence RxData.
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 * This function must check & set the busy state for re-entrancy
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 * 		DevAddr : The device selection id scheme
	 *
	 * @return 	true - Success
	 * 			false - failed.
	 */
	bool (*StartRx)(SERINTRFDEV *pSerDev, int DevAddr);

	/**
	 * @brief - RxData
	 * 		Receive data into pBuff passed in parameter.  Assuming StartRx was
	 * called prior calling this function to get the actual data
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 * 		pBuff 	: Pointer to memory area to receive data.
	 * 		BuffLen : Length of buffer memory in bytes
	 *
	 * @return	Number of bytes read
	 */
	int (*RxData)(SERINTRFDEV *pSerDev, uint8_t *pBuff, int BuffLen);

	/**
	 * @brief - StopRx
	 * 		Completion of read data phase. Do require post processing
	 * after data has been received via RxData
	 * This function must clear the busy state for re-entrancy
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 *
	 * @return	None
	 */
	void (*StopRx)(SERINTRFDEV *pSerDev);

	/**
	 * @brief - StartTx
	 * 		Prepare start condition to transfer data with subsequence TxData.
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 * This function must check & set the busy state for re-entrancy
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 * 		DevAddr : The device selection id scheme
	 *
	 * @return 	true - Success
	 * 			false - failed
	 */
	bool (*StartTx)(SERINTRFDEV *pSerDev, int DevAddr);

	/**
	 * @brief - TxData
	 * 		Transfer data from pData passed in parameter.  Assuming StartTx was
	 * called prior calling this function to send the actual data
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 * 		pData 	: Pointer to memory area of data to send.
	 * 		DataLen : Length of data memory in bytes
	 *
	 * @return	Number of bytes sent
	 */
	int (*TxData)(SERINTRFDEV *pSerDev, uint8_t *pData, int DataLen);

	/**
	 * @brief - StopTx
	 * 		Completion of sending data via TxData.  Do require post processing
	 * after all data was transmitted via TxData.
	 * This function must clear the busy state for re-entrancy
	 *
	 * @param
	 * 		pSerDev : Pointer to an instance of the Serial Interface
	 *
	 * @return	None
	 */
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

int SerialIntrfRx(SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen);
int SerialIntrfTx(SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen);
// Read transfer. Send setup data then read return data.
int SerialIntrfRead(SERINTRFDEV *pDev, int DevAddr, uint8_t *pTxData, int TxLen,
                    uint8_t *pRxBuff, int RxLen);

static inline bool SerialIntrfStartRx(SERINTRFDEV *pDev, int DevAddr) {
	if (pDev->Busy)
		return false;
	AtomicAssign((sig_atomic_t *)&pDev->Busy, true);
	return pDev->StartRx(pDev, DevAddr);
}

static inline int SerialIntrfRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->RxData(pDev, pBuff, BuffLen);
}

static inline void SerialIntrfStopRx(SERINTRFDEV *pDev) {
	AtomicAssign((sig_atomic_t *)&pDev->Busy, false);
	pDev->StopRx(pDev);
}

static inline bool SerialIntrfStartTx(SERINTRFDEV *pDev, int DevAddr) {
	if (pDev->Busy)
		return false;
	AtomicAssign((sig_atomic_t *)&pDev->Busy, true);
	return pDev->StartTx(pDev, DevAddr);
}

static inline int SerialIntrfTxData(SERINTRFDEV *pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->TxData(pDev, pBuff, BuffLen);
}

static inline void SerialIntrfStopTx(SERINTRFDEV *pDev) {
	AtomicAssign((sig_atomic_t *)&pDev->Busy, false);
	pDev->StopTx(pDev);
}


#ifdef __cplusplus
/*
 * C++ interface class
 */
class SerialIntrf {
public:
	virtual ~SerialIntrf() {}
	virtual operator SERINTRFDEV* () = 0;	// Get serial interface data
	// Set data rate in bits/sec (Hz)
	virtual int Rate(int DataRate) = 0;
	// Get current data rate in bits/sec (Hz)
	virtual int Rate(void) = 0;
	// Disable device for power reduction, re-enable with Enable() without
	// full init
	virtual void Disable(void) = 0;
	// Enable device
	virtual void Enable(void) = 0;
	// Receive full frame
	virtual int Rx(int DevAddr, uint8_t *pBuff, int BuffLen) {
		return SerialIntrfRx(*this,DevAddr, pBuff, BuffLen);
	}
	// Transmit full frame
	virtual int Tx(int DevAddr, uint8_t *pData, int DataLen) {
		return SerialIntrfTx(*this, DevAddr, pData, DataLen);
	}
	// Read transfer. Send setup data then read return data.
    virtual int Read(int DevAddr, uint8_t *pTxData, int TxLen, uint8_t *pRxBuff, int RxLen) {
        return SerialIntrfRead(*this, DevAddr, pTxData, TxLen, pRxBuff, RxLen);
    }
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

#endif	// __SERIALINTRF_H__
