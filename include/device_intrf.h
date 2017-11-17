/*--------------------------------------------------------------------------
File   : device_intrf.h

Author : Hoang Nguyen Hoan          				Nov. 25, 2011

Desc   : Generic data transfer interface class
		 This class is used to implement device communication interfaces
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
Modified by         Date              	Description
Hoan				Mar. 2, 2017		Renaming from SerialIntrf to DeviceInterf
----------------------------------------------------------------------------*/
#ifndef __DEVICEINTRF_H__
#define __DEVICEINTRF_H__

#include <stdint.h>
#include <stdbool.h>
#include "atomic.h"

/*
 * Device interface event types
 */
typedef enum {
	DEVINTRF_EVT_RX_TIMEOUT,    // Rx timeout
	DEVINTRF_EVT_RX_DATA,       // Data received
	DEVINTRF_EVT_RX_FIFO_FULL,  // Receive fifo full, fifo will be pushed out
                                // if handler does not process fifo (returns 0)
	DEVINTRF_EVT_TX_TIMEOUT,    // Tx timeout
	DEVINTRF_EVT_TX_READY,      // Ready to transmit
	DEVINTRF_EVT_TX_FIFO_FULL,  // Transmit fifo full, fifo will be pushed out
                                // if handler does not process fifo (returns 0)
	DEVINTRF_EVT_STATECHG,      // State changed. State data is device dependent.
                                // To be interpreted by implementation
} DEVINTRF_EVT;

/*
 * Device Interface forward data structure type definition.
 * This structure is the base object.  Pointer to an instance of this is passed
 * to all function calls.  See structure definition bellow for more details
 */
typedef struct __device_intrf DEVINTRF;


/**
 * @brief
 *
 * Event handler callback. This is normally being called within interrupts, avoid blocking
 *
 * @param 	pDev 	: Device handle
 * 			EvtId 	: Event code
 *  		pBuffer : In/Out Buffer containing data
 * 					  on DEVINTRF_EVT_RX_TIMEOUT & DEVINTRF_EVT_RXDATA, pBuffer contains data received. If
 * 					  driver implements CFIFO, this parameter is NULL with BufferLen indicating total data
 * 					  in fifo.
 * 					  on DEVINTRF_EVT_TX_READY, pBuffer contains data to be transmit with max length
 * 					  BufferLen. If driver implements CFIFO, this parameter is NULL and BufferLen
 * 					  indicates amount of data stored in fifo
 * 					  on DEVINTRF_EVT_STATECHG, pBuffer contains state data. This is implementation specific
 * 					  for example UART implementation would contains line state info.
 *
 * 			BufferLen : Max buffer length.  See above description
 *
 * @return number of bytes processed.  Implementation specific
 *         in case of FIFO_FULL events,  fifo will be pushed out if return value is zero
 */
typedef int (*DEVINTRF_EVTCB)(DEVINTRF *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#pragma pack(push, 4)

/*
 * Device interface data structure.  This structure is the actual interface for both C++ & C code
 * It is used to provide C compatibility instead of using C++ interface which is only for C++
 */
struct __device_intrf {
	void *pDevData;			// Private device interface implementation data
	int	IntPrio;			// Interrupt priority.  Value is implementation specific
	DEVINTRF_EVTCB EvtCB;	// Interrupt based event callback function pointer. Must be set to NULL if not used
	bool Busy;		        // Busy flag to be set check and set at start and reset at end of transmission
	int MaxRetry;			// Max retry when data could not be transfered (Rx/Tx returns zero count)
	int EnCnt;				// Count the number of time device is enabled, this used as ref count where multiple
							// devices are using the same interface. It is to avoid it being disabled while another
							// device is still using it

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
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return None
	 */
	void (*Disable)(DEVINTRF *pDevIntrf);

	/**
	 * @brief - Enable
	 * 		Turn on the interface.
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return None
	 */
	void (*Enable)(DEVINTRF *pDevIntrf);

	/**
	 * @brief - GetRate
	 * 		Get data rate of the interface in Hertz.  This is not a clock frequency
	 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
	 * implementation as bits/sec or bytes/sec or whatever the case
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return Transfer rate per second
	 */
	int (*GetRate)(DEVINTRF *pDevIntrf);

	/**
	 * @brief - SetRate
	 * 		Set data rate of the interface in Hertz.  This is not a clock frequency
	 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
	 * implementation as bits/sec or bytes/sec or whatever the case
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 * 		Rate 	  : Data rate to be set in Hertz (transfer per second)
	 *
	 * @return 	Actual transfer rate per second set.  It is the real capable rate
	 * 			closes to rate being requested.
	 */
	int (*SetRate)(DEVINTRF *pDevIntrf, int Rate);

	/**
	 * @brief - StartRx
	 * 		Prepare start condition to receive data with subsequence RxData.
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 * This function must check & set the busy state for re-entrancy
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 * 		DevAddr   : The device selection id scheme
	 *
	 * @return 	true - Success
	 * 			false - failed.
	 */
	bool (*StartRx)(DEVINTRF *pDevIntrf, int DevAddr);

	/**
	 * @brief - RxData
	 * 		Receive data into pBuff passed in parameter.  Assuming StartRx was
	 * called prior calling this function to get the actual data
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 * 		pBuff 	  : Pointer to memory area to receive data.
	 * 		BuffLen   : Length of buffer memory in bytes
	 *
	 * @return	Number of bytes read
	 */
	int (*RxData)(DEVINTRF *pDevIntrf, uint8_t *pBuff, int BuffLen);

	/**
	 * @brief - StopRx
	 * 		Completion of read data phase. Do require post processing
	 * after data has been received via RxData
	 * This function must clear the busy state for re-entrancy
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return	None
	 */
	void (*StopRx)(DEVINTRF *pSerDev);

	/**
	 * @brief - StartTx
	 * 		Prepare start condition to transfer data with subsequence TxData.
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 * This function must check & set the busy state for re-entrancy
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 * 		DevAddr   : The device selection id scheme
	 *
	 * @return 	true - Success
	 * 			false - failed
	 */
	bool (*StartTx)(DEVINTRF *pDevIntrf, int DevAddr);

	/**
	 * @brief - TxData
	 * 		Transfer data from pData passed in parameter.  Assuming StartTx was
	 * called prior calling this function to send the actual data
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 * 		pData 	: Pointer to memory area of data to send.
	 * 		DataLen : Length of data memory in bytes
	 *
	 * @return	Number of bytes sent
	 */
	int (*TxData)(DEVINTRF *pDevIntrf, uint8_t *pData, int DataLen);

	/**
	 * @brief - StopTx
	 * 		Completion of sending data via TxData.  Do require post processing
	 * after all data was transmitted via TxData.
	 * This function must clear the busy state for re-entrancy
	 *
	 * @param
	 * 		pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return	None
	 */
	void (*StopTx)(DEVINTRF *pDevIntrf);

	/**
	 * @brief - Reset
	 *      This function perform a reset of interface.  Must provide empty
	 * function of not used.
	 *
     * @param
     *      pDevIntrf : Pointer to an instance of the Device Interface
     *
     * @return  None
	 */
	void (*Reset)(DEVINTRF *pDevIntrf);
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

// C only function prototypes
static inline void DeviceIntrfDisable(DEVINTRF *pDev) {
	if (AtomicDec(&pDev->EnCnt) < 1)
    {
    	pDev->Disable(pDev);
    }
}

static inline void DeviceIntrfEnable(DEVINTRF *pDev) {
    if (AtomicInc(&pDev->EnCnt) == 1)
    {
    	pDev->Enable(pDev);
    }
}

static inline int DeviceIntrfGetRate(DEVINTRF *pDev) {
	return pDev->GetRate(pDev);
}

static inline int DeviceIntrfSetRate(DEVINTRF *pDev, int Rate) {
	return pDev->SetRate(pDev, Rate);
}

int DeviceIntrfRx(DEVINTRF *pDev, int DevAddr, uint8_t *pBuff, int BuffLen);
int DeviceIntrfTx(DEVINTRF *pDev, int DevAddr, uint8_t *pBuff, int BuffLen);
// Read transfer. Send setup data (pAdCmd) then read return data.
int DeviceIntrfRead(DEVINTRF *pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                    uint8_t *pRxBuff, int RxLen);
// Write transfer. Send setup data (pAdCmd) and write data in single transfer.
int DeviceIntrfWrite(DEVINTRF *pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                     uint8_t *pTxData, int TxLen);

// Initiate receive
// WARNING this function must be used in pair with StopRx
// Re-entrance protection flag is used
// On success, StopRx must be after transmission is completed to release flag
static inline bool DeviceIntrfStartRx(DEVINTRF *pDev, int DevAddr) {
    if (AtomicTestAndSet(&pDev->Busy))
        return false;

    bool retval = pDev->StartRx(pDev, DevAddr);

    // In case of returned false, app would not call Stop to release busy flag
    // so we need to do that here before returning
    if (retval == false) {
        AtomicClear(&pDev->Busy);
    }

    return retval;
}

static inline int DeviceIntrfRxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->RxData(pDev, pBuff, BuffLen);
}

static inline void DeviceIntrfStopRx(DEVINTRF *pDev) {
    pDev->StopRx(pDev);
    AtomicClear(&pDev->Busy);
}

// Initiate receive
// WARNING this function must be used in pair with StopTx
// Re-entrance protection flag is used
// On success, StopTx must be after transmission is completed to release flag
static inline bool DeviceIntrfStartTx(DEVINTRF *pDev, int DevAddr) {
    if (AtomicTestAndSet(&pDev->Busy))
        return false;

    bool retval =  pDev->StartTx(pDev, DevAddr);

    // In case of returned false, app would not call Stop to release busy flag
    // so we need to do that here before returning
    if (retval == false) {
        AtomicClear(&pDev->Busy);
    }

    return retval;
}

static inline int DeviceIntrfTxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->TxData(pDev, pBuff, BuffLen);
}

static inline void DeviceIntrfStopTx(DEVINTRF *pDev) {
    pDev->StopTx(pDev);
    AtomicClear(&pDev->Busy);
}

static inline void DeviceIntrfReset(DEVINTRF *pDev) {
    if (pDev->Reset)
        pDev->Reset(pDev);
}

#ifdef __cplusplus
}

/*
 * C++ interface class
 */
class DeviceIntrf {
public:
	virtual ~DeviceIntrf() {}
	virtual operator DEVINTRF* () = 0;	// Get device interface data (handle)
	// Set data rate in bits/sec (Hz)
	virtual int Rate(int DataRate) = 0;
	// Get current data rate in bits/sec (Hz)
	virtual int Rate(void) = 0;
	// Disable device for power reduction, re-enable with Enable() without
	// full init
	virtual void Disable(void) { DeviceIntrfDisable(*this); }
	// Enable device
	virtual void Enable(void) { DeviceIntrfEnable(*this); }
	// Receive full frame
	virtual int Rx(int DevAddr, uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRx(*this,DevAddr, pBuff, BuffLen);
	}
	// Transmit full frame
	virtual int Tx(int DevAddr, uint8_t *pData, int DataLen) {
		return DeviceIntrfTx(*this, DevAddr, pData, DataLen);
	}
	// Read transfer. Send setup data then read return data.
    virtual int Read(int DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pRxBuff, int RxLen) {
        return DeviceIntrfRead(*this, DevAddr, pAdCmd, AdCmdLen, pRxBuff, RxLen);
    }
    // Write transfer. Send setup data and write data in single transfer.
    // @return  number of bytes of write data send (not count setup data)
    virtual int Write(int DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pTxData, int TxLen) {
        return DeviceIntrfWrite(*this, DevAddr, pAdCmd, AdCmdLen, pTxData, TxLen);
    }
	// Initiate receive
    // WARNING this function must be used in pair with StopRx
    // Re-entrance protection flag is used
    // On success, StopRx must be after transmission is completed to release flag
	virtual bool StartRx(int DevAddr) = 0;
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) = 0;
	// Stop receive
	// WARNING !!!!!
	// This functions MUST ONLY be called if StartRx returns true.
	virtual void StopRx(void) = 0;
	// Initiate transmit
    // WARNING this function must be used in pair with StopTx
    // Re-entrance protection flag is used
    // On success, StopTx must be after transmission is completed to release flag
	virtual bool StartTx(int DevAddr) = 0;
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) = 0;
	// Stop transmit
	// WARNING !!!!!
	// This functions MUST ONLY be called if StartTx returns true.
	virtual void StopTx(void) = 0;
	virtual bool RequestToSend(int NbBytes) { return true; }
	//
	virtual void Reset(void) { DeviceIntrfReset(*this); }
};

#endif

#endif	// __DEVICEINTRF_H__
