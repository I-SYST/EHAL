/**-------------------------------------------------------------------------
@file	device_intrf.h

@brief	Generic data transfer interface class

This class is used to implement device communication interfaces such as I2C, UART, etc...
Not limited to wired or physical interface.  It could be soft interface as well such
as SLIP protocol or any mean of transferring data between 2 entities.

@author	Hoang Nguyen Hoan
@date	Nov. 25, 2011

@license

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

----------------------------------------------------------------------------*/
#ifndef __DEVICEINTRF_H__
#define __DEVICEINTRF_H__

#include <stdint.h>
#include <stdbool.h>
#include "atomic.h"

/** @addtogroup device_intrf	Device Interface
  * @{
  */

/// Device interface event types.
typedef enum {
	DEVINTRF_EVT_RX_TIMEOUT,    //!< Rx timeout.
	DEVINTRF_EVT_RX_DATA,       //!< Data received
	DEVINTRF_EVT_RX_FIFO_FULL,  //!< Receive FIFO full, FIFO will be pushed out
                                //!< if handler does not process FIFO (returns 0)
	DEVINTRF_EVT_TX_TIMEOUT,    //!< Tx timeout
	DEVINTRF_EVT_TX_READY,      //!< Ready to transmit
	DEVINTRF_EVT_TX_FIFO_FULL,  //!< Transmit FIFO full, FIFO will be pushed out
                                //!< if handler does not process FIFO (returns 0)
	DEVINTRF_EVT_STATECHG,      //!< State changed. State data is device dependent.
                                //!< To be interpreted by implementation
    DEVINTRF_EVT_READ_RQST,     //!< Receive a read request from host
	DEVINTRF_EVT_WRITE_RQST,	//!< Received a write request from host
	DEVINTRF_EVT_COMPLETED,		//!< Transfer completed
} DEVINTRF_EVT;

/// @brief	Device Interface forward data structure type definition.
/// This structure is the base object.  Pointer to an instance of this is passed
/// to all function calls.  See structure definition bellow for more details
typedef struct __device_intrf DEVINTRF;


/**
 * @brief	Event handler callback.
 *
 * This is normally being called within interrupts, avoid blocking
 *
 * @param 	pDev 	: Device handle
 * @param	EvtId 	: Event code
 * @param	pBuffer : In/Out Buffer containing data\n
 * 					  on DEVINTRF_EVT_RX_TIMEOUT & DEVINTRF_EVT_RXDATA, pBuffer contains data received. If
 * 					  driver implements CFIFO, this parameter is NULL with BufferLen indicating total data
 * 					  in FIFO.\n
 * 					  on DEVINTRF_EVT_TX_READY, pBuffer contains data to be transmit with max length
 * 					  BufferLen. If driver implements CFIFO, this parameter is NULL and BufferLen
 * 					  indicates amount of data stored in FIFO\n
 * 					  on DEVINTRF_EVT_STATECHG, pBuffer contains state data. This is implementation specific
 * 					  for example UART implementation would contains line state info.
 *
 * @param	Len 	: Max buffer length.  See above description
 *
 * @return	Number of bytes processed.  Implementation specific.\n
 * 			in case of FIFO_FULL events,  FIFO will be pushed out if return value is zero
 */
typedef int (*DEVINTRF_EVTCB)(DEVINTRF * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

#pragma pack(push, 4)

/// @brief	Device interface data structure.
///
/// This structure is the actual interface for both C++ & C code
/// It is used to provide C compatibility instead of using C++ interface which is only for C++
///
/// This data structure is visible for implementer of interface.
/// It is seen as handle for application to pass to the interface function calls.
/// Application firmware should not access any member of this structure directly.
///
struct __device_intrf {
	void *pDevData;			//!< Private device interface implementation data
	int	IntPrio;			//!< Interrupt priority.  Value is implementation specific
	DEVINTRF_EVTCB EvtCB;	//!< Interrupt based event callback function pointer. Must be set to NULL if not used
	bool Busy;		        //!< Busy flag to be set check and set at start and reset at end of transmission
	int MaxRetry;			//!< Max retry when data could not be transfered (Rx/Tx returns zero count)
	int EnCnt;				//!< Count the number of time device is enabled, this used as ref count where multiple
							//!< devices are using the same interface. It is to avoid it being disabled while another
							//!< device is still using it

	// Bellow are all mandatory functions to implement
	// On init, all implementation must fill these function, no NULL allowed
	// If a function is not used. It must be implemented as do nothing function

	/**
	 * @brief	Put the interface to sleep for maximum energy saving.
	 *
	 * If this is a physical interface, provide a way to put the interface to sleep
	 * for maximum energy saving possible.  This function must be implemented in
	 * such a way that the interface can be re-enable without going through full
	 * initialization sequence.
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 */
	void (*Disable)(DEVINTRF * const pDevIntrf);

	/**
	 * @brief	Wake up the interface.
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 */
	void (*Enable)(DEVINTRF * const pDevIntrf);

	/**
	 * @brief	Get data rate of the interface in Hertz.  This is not a clock frequency
	 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
	 * implementation as bits/sec or bytes/sec or whatever the case
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return	Transfer rate per second
	 */
	int (*GetRate)(DEVINTRF * const pDevIntrf);

	/**
	 * @brief	Set data rate of the interface in Hertz.  This is not a clock frequency
	 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
	 * implementation as bits/sec or bytes/sec or whatever the case
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 * @param	Rate 	  : Data rate to be set in Hertz (transfer per second)
	 *
	 * @return 	Actual transfer rate per second set.  It is the real capable rate
	 * 			closest to rate being requested.
	 */
	int (*SetRate)(DEVINTRF * const pDevIntrf, int Rate);

	/**
	 * @brief	Prepare start condition to receive data with subsequence RxData.
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 * This function must check & set the busy state for re-entrancy
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 * @param	DevAddr   : The device selection id scheme
	 *
	 * @return 	true - Success\n
	 * 			false - failed.
	 */
	bool (*StartRx)(DEVINTRF * const pDevIntrf, int DevAddr);

	/**
	 * @brief	Receive data into pBuff passed in parameter.  Assuming StartRx was
	 * called prior calling this function to get the actual data
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 * @param	pBuff 	  : Pointer to memory area to receive data.
	 * @param	BuffLen   : Length of buffer memory in bytes
	 *
	 * @return	Number of bytes read
	 */
	int (*RxData)(DEVINTRF * const pDevIntrf, uint8_t *pBuff, int BuffLen);

	/**
	 * @brief	Completion of read data phase. Do require post processing
	 * after data has been received via RxData
	 * This function must clear the busy state for reentrancy
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return	None
	 */
	void (*StopRx)(DEVINTRF * const pSerDev);

	/**
	 * @brief	Prepare start condition to transfer data with subsequence TxData.
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 * This function must check & set the busy state for re-entrancy
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 * @param	DevAddr   : The device selection id scheme
	 *
	 * @return 	true - Success\n
	 * 			false - failed
	 */
	bool (*StartTx)(DEVINTRF * const pDevIntrf, int DevAddr);

	/**
	 * @brief	Transfer data from pData passed in parameter.  Assuming StartTx was
	 * called prior calling this function to send the actual data
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 * @param	pData 	: Pointer to memory area of data to send.
	 * @param	DataLen : Length of data memory in bytes
	 *
	 * @return	Number of bytes sent
	 */
	int (*TxData)(DEVINTRF * const pDevIntrf, uint8_t *pData, int DataLen);

	/**
	 * @brief	Completion of sending data via TxData.  Do require post processing
	 * after all data was transmitted via TxData.
	 * This function must clear the busy state for re-entrancy
	 *
	 * @param	pDevIntrf : Pointer to an instance of the Device Interface
	 *
	 * @return	None
	 */
	void (*StopTx)(DEVINTRF * const pDevIntrf);

	/**
	 * @brief	This function perform a reset of interface.  Must provide empty
	 * function of not used.
	 *
     * @param	pDevIntrf : Pointer to an instance of the Device Interface
     *
     * @return  None
	 */
	void (*Reset)(DEVINTRF * const pDevIntrf);

	/**
	 * @brief	Power off device for power saving.
	 *
	 * This function will power off device completely. Not all device provide this
	 * type of functionality.  Once power off is call, full initialization cycle is
	 * required.  Therefore their is no PowerOn counter part of this function contrary
	 * to the Enable/Disable functions.
	 *
     * @param	pDevIntrf : Pointer to an instance of the Device Interface
     *
     * @return  None
	 */
	void (*PowerOff)(DEVINTRF * const pDevIntrf);

};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Disable interface.  Put the interface in lowest power mode.
 *
 * If this is a physical interface, provide a
 * way to turn off for energy saving. Make sure the turn off procedure can
 * be turned back on without going through the full initialization sequence
 *
 * @param	pDev	: Pointer to an instance of the Device Interface
 *
 * @return	None
 */
static inline void DeviceIntrfDisable(DEVINTRF * const pDev) {
	if (AtomicDec(&pDev->EnCnt) < 1) {
    	pDev->Disable(pDev);
    	AtomicAssign(&pDev->EnCnt, 0);
    }
}

/**
 * @brief	Wake up the interface.
 *
 * @param	pDev	: Pointer to an instance of the Device Interface
 *
 * @return	None
 */
static inline void DeviceIntrfEnable(DEVINTRF * const pDev) {
    if (AtomicInc(&pDev->EnCnt) == 1) {
    	pDev->Enable(pDev);
    }
}

/**
 * @brief	Get data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param	pDev	: Pointer to an instance of the Device Interface
 *
 * @return	Transfer rate per second
 */
static inline int DeviceIntrfGetRate(DEVINTRF * const pDev) {
	return pDev->GetRate(pDev);
}

/**
 * @brief	Set data rate of the interface in Hertz.
 *
 * This is not a clock frequency but rather the transfer frequency (number of
 * transfers per second). It has meaning base on the implementation as bits/sec
 * or bytes/sec or whatever the case
 *
 * @param	pDev	: Pointer to an instance of the Device Interface
 * @param	Rate	: Data rate to be set in Hertz (transfer per second)
 *
 * @return 	Actual transfer rate per second set.  It is the real capable rate
 * 			closest to rate being requested.
 */
static inline int DeviceIntrfSetRate(DEVINTRF * const pDev, int Rate) {
	return pDev->SetRate(pDev, Rate);
}

/**
 * @brief	Full receive data sequence.
 *
 * This function does full receive data sequence by calling StartRx, RxData, StopRx.
 *
 * @param	pDev	: Pointer to an instance of the Device Interface
 * @param	DevAddr	: The device selection id scheme
 * @param	pBuff	: Pointer to memory area to receive data.
 * @param	BuffLen	: Length of buffer memory in bytes
 *
 * @return	Number of bytes read
 */
int DeviceIntrfRx(DEVINTRF * const pDev, int DevAddr, uint8_t *pBuff, int BuffLen);

/**
 * @brief	Full transmit data sequence.
 *
 * This function does full transmit data sequence by calling StartTx, TxData, StopTx.
 *
 * @param	pDev	: Pointer to an instance of the Device Interface
 * @param	DevAddr	: The device selection id scheme
 * @param	pData	: Pointer to data to send.
 * @param	DataLen	: Length of data in bytes
 *
 * @return	Number of bytes read
 */
int DeviceIntrfTx(DEVINTRF * const pDev, int DevAddr, uint8_t *pData, int DataLen);

/**
 * @brief	Device read transfer.
 *
 * A device read transfer usually starts with a write of a command or register address.
 * Then follows with a read data results. This function encapsulate that functionality.
 *
 * @param	pDev		: Pointer to an instance of the Device Interface
 * @param	DevAddr   	: The device selection id scheme
 * @param	pAdCmd		: Pointer to buffer containing address or command code to send
 * @param	AdCmdLen	: Size of addr/Cmd in bytes
 * @param	pBuff 	  	: Pointer to memory area to receive data.
 * @param	BuffLen   	: Length of buffer memory in bytes
 *
 * @return	Number of bytes read
 */
int DeviceIntrfRead(DEVINTRF * const pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                    uint8_t *pRxBuff, int RxLen);

/**
 * @brief	Device write transfer.
 *
 * A device write transfer usually starts with a write of a command or register address.
 * Then follows with a write data. This function encapsulate that functionality.
 *
 * @param	pDev		: Pointer to an instance of the Device Interface
 * @param	DevAddr   	: The device selection id scheme
 * @param	pAdCmd		: Pointer to buffer containing address or command code to send
 * @param	AdCmdLen	: Size of addr/Cmd in bytes
 * @param	pData 	  	: Pointer to data to send.
 * @param	DataLen   	: Length of data in bytes
 *
 * @return	Number of bytes of data sent (not counting the Addr/Cmd).
 */
int DeviceIntrfWrite(DEVINTRF * const pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                     uint8_t *pData, int DataLen);

/**
 * @brief	Prepare start condition to receive data with subsequence RxData.
 *
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * NOTE: On success StopRx must be called to release busy flag
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 * @param	DevAddr   : The device selection id scheme
 *
 * @return 	true - Success\n
 * 			false - failed.
 */
static inline bool DeviceIntrfStartRx(DEVINTRF * const pDev, int DevAddr) {
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

/**
 * @brief	Receive data into pBuff passed in parameter.  Assuming StartRx was
 * called prior calling this function to get the actual data
 *
 * @param	pDev 	: Pointer to an instance of the Device Interface
 * @param	pBuff 	: Pointer to memory area to receive data.
 * @param	BuffLen : Length of buffer memory in bytes
 *
 * @return	Number of bytes read
 */
static inline int DeviceIntrfRxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->RxData(pDev, pBuff, BuffLen);
}

/**
 * @brief	Completion of read data phase.
 *
 * Do require post processing after data has been received via RxData
 * This function must clear the busy state for re-entrancy
 *
 * @param	pDev : Pointer to an instance of the Device Interface
 */
static inline void DeviceIntrfStopRx(DEVINTRF * const pDev) {
    pDev->StopRx(pDev);
    AtomicClear(&pDev->Busy);
}

// Initiate receive
// WARNING this function must be used in pair with StopTx
// Re-entrance protection flag is used
// On success, StopTx must be after transmission is completed to release flag
/**
 * @brief	Prepare start condition to transfer data with subsequence TxData.
 *
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * On success StopRx must be called to release busy flag
 *
 * @param	pDev : Pointer to an instance of the Device Interface
 * @param	DevAddr   : The device selection id scheme
 *
 * @return 	true - Success\n
 * 			false - failed
 */
static inline bool DeviceIntrfStartTx(DEVINTRF * const pDev, int DevAddr) {
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

/**
 * @brief	Transfer data from pData passed in parameter.  Assuming StartTx was
 * called prior calling this function to send the actual data
 *
 * @param	pDev	: Pointer to an instance of the Device Interface
 * @param	pData 	: Pointer to memory area of data to send.
 * @param	DataLen : Length of data memory in bytes
 *
 * @return	Number of bytes sent
 */
static inline int DeviceIntrfTxData(DEVINTRF * const pDev, uint8_t *pBuff, int BuffLen) {
	return pDev->TxData(pDev, pBuff, BuffLen);
}

/**
 * @brief	Completion of sending data via TxData.
 *
 * Perform the require post processing
 * after all data was transmitted via TxData.
 * This function must clear the busy state for re-entrancy
 *
 * @param	pDev : Pointer to an instance of the Device Interface
 */
static inline void DeviceIntrfStopTx(DEVINTRF * const pDev) {
    pDev->StopTx(pDev);
    AtomicClear(&pDev->Busy);
}

/**
 * @brief	This function perform a reset of interface.
 *
 * @param	pDev : Pointer to an instance of the Device Interface
 */
static inline void DeviceIntrfReset(DEVINTRF * const pDev) {
    if (pDev->Reset)
        pDev->Reset(pDev);
}

/**
 * @brief	Power off interface completely for power saving.
 *
 * This function will power off the interface completely. Not all interface
 * provides this type of functionality.  Once power off is call, full
 * initialization cycle is required.  Therefore there is no PowerOn counter
 * part of this function contrary to the Enable/Disable functions.
 *
 * @param	pDev : Pointer to an instance of the Device Interface
 *
 * @return  None
 */
static inline void DeviceIntrfPowerOff(DEVINTRF * const pDev) {
	if (pDev->PowerOff) pDev->PowerOff(pDev);
}


#ifdef __cplusplus
}


/// @brief	Generic data transfer interface class
///
/// This class is used to implement device communication interfaces such as I2C, UART, etc...
/// Not limited to wired or physical interface.  It could be soft interface as well such
/// as SLIP protocol or any mean of transferring data between 2 entities.
class DeviceIntrf {
public:

	/**
	 * @brief	Operator to convert this class to device interface handle to be
	 * 			used with C functions.
	 *
	 * @return	Pointer to internal DEVINTRF to be used with C interface functions
	 */
	virtual operator DEVINTRF * const () = 0;	// Get device interface data (handle)

	/**
	 * @brief	Set data rate of the interface in Hertz.
	 *
	 * This is not a clock frequency but rather the transfer frequency (number
	 * of transfers per second). It has meaning base on the implementation as
	 * bits/sec or bytes/sec or whatever the case
	 *
	 * @param	Rate : Data rate to be set in Hertz (transfer per second)
	 *
	 * @return 	Actual transfer rate per second set.  It is the real capable rate
	 * 			closest to rate being requested.
	 */
	virtual int Rate(int DataRate) = 0;

	/**
	 * @brief	Get data rate of the interface in Hertz.
	 *
	 * This is not a clock frequency but rather the transfer frequency (number
	 * of transfers per second). It has meaning base on the implementation as
	 * bits/sec or bytes/sec or whatever the case
	 *
	 * @return	Transfer rate per second
	 */
	virtual int Rate(void) = 0;

	/**
	 * @brief	Turn off/Deep sleep the interface.
	 *
	 * If this is a physical interface, provide a
	 * way to turn off for energy saving. Make sure the turn off procedure can
	 * be turned back on without going through the full initialization sequence
	 */
	virtual void Disable(void) { DeviceIntrfDisable(*this); }

	/**
	 * @brief	Turn on/wake up the interface.
	 */
	virtual void Enable(void) { DeviceIntrfEnable(*this); }

	/**
	 * @brief	Power off device completely for power saving.
	 *
	 * This function will power off device completely. Not all device provide this
	 * type of functionality.  Once power off is called, full initialization cycle is
	 * required.  Therefore there is no PowerOn counter part of this function contrary
	 * to the Enable/Disable functions.
	 *
     * @return  None
	 */
	void PowerOff() { DeviceIntrfPowerOff(*this); }

	/**
	 * @brief	Full receive data sequence.
	 *
	 * This function does full receive data sequence by calling StartRx, RxData, StopRx.
	 *
	 * @param	DevAddr   : The device selection id scheme
	 * @param	pBuff 	  : Pointer to memory area to receive data.
	 * @param	BuffLen   : Length of buffer memory in bytes
	 *
	 * @return	Number of bytes read
	 */
	virtual int Rx(int DevAddr, uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRx(*this,DevAddr, pBuff, BuffLen);
	}

	/**
	 * @brief	Full transmit data sequence.
	 *
	 * This function does full transmit data sequence by calling StartTx, TxData, StopTx.
	 *
	 * @param	DevAddr   : The device selection id scheme
	 * @param	pData 	  : Pointer to data to send.
	 * @param	DataLen   : Length of data in bytes
	 *
	 * @return	Number of bytes read
	 */
	virtual int Tx(int DevAddr, uint8_t *pData, int DataLen) {
		return DeviceIntrfTx(*this, DevAddr, pData, DataLen);
	}

	/**
	 * @brief	Device read transfer.
	 *
	 * A device read transfer usually starts with a write of a command or register address.
	 * Then follows with a read data results. This function encapsulate that functionality.
	 *
	 * @param	DevAddr   	: The device selection id scheme
	 * @param	pAdCmd		: Pointer to buffer containing address or command code to send
	 * @param	AdCmdLen	: Size of addr/Cmd in bytes
	 * @param	pBuff 	  	: Pointer to memory area to receive data.
	 * @param	BuffLen   	: Length of buffer memory in bytes
	 *
	 * @return	Number of bytes read
	 */
    virtual int Read(int DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen) {
        return DeviceIntrfRead(*this, DevAddr, pAdCmd, AdCmdLen, pBuff, BuffLen);
    }

    /**
     * @brief	Device write transfer.
     *
     * A device write transfer usually starts with a write of a command or register address.
     * Then follows with a write data. This function encapsulate that functionality.
     *
     * @param	DevAddr   	: The device selection id scheme
     * @param	pAdCmd		: Pointer to buffer containing address or command code to send
     * @param	AdCmdLen	: Size of addr/Cmd in bytes
     * @param	pData 	  	: Pointer to data to send.
     * @param	DataLen   	: Length of data in bytes
     *
     * @return	Number of bytes of data sent (not counting the Addr/Cmd).
     */
    virtual int Write(int DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pData, int DataLen) {
        return DeviceIntrfWrite(*this, DevAddr, pAdCmd, AdCmdLen, pData, DataLen);
    }

	// Initiate receive
    // WARNING this function must be used in pair with StopRx
    // Re-entrance protection flag is used
    // On success, StopRx must be after transmission is completed to release flag
    /**
     * @brief	Prepare start condition to receive data with subsequence RxData.
     *
     * This can be in case such as start condition for I2C or Chip Select for
     * SPI or precondition for DMA transfer or whatever requires it or not
     * This function must check & set the busy state for reentrancy
     *
     * On success StopRx must be called to release busy flag
     *
     * @param	DevAddr   : The device selection id scheme
     *
     * @return 	true - Success\n
     * 			false - failed.
     */
	virtual bool StartRx(int DevAddr) = 0;

	/**
	 * @brief	Receive data into pBuff passed in parameter.  Assuming StartRx was
	 * called prior calling this function to get the actual data
	 *
	 * @param	pBuff 	  : Pointer to memory area to receive data.
	 * @param	BuffLen   : Length of buffer memory in bytes
	 *
	 * @return	Number of bytes read
	 */
	virtual int RxData(uint8_t *pBuff, int BuffLen) = 0;

	// Stop receive
	// WARNING !!!!!
	// This functions MUST ONLY be called if StartRx returns true.
	/**
	 * @brief	Completion of read data phase.
	 *
	 * Do require post processing after data has been received via RxData
	 * This function must clear the busy state for reentrancy.\n
	 * Call this function only if StartRx was successful.
	 */
	virtual void StopRx(void) = 0;

	// Initiate transmit
    // WARNING this function must be used in pair with StopTx
    // Re-entrance protection flag is used
    // On success, StopTx must be after transmission is completed to release flag
	/**
	 * @brief	Prepare start condition to transfer data with subsequence TxData.
	 *
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 * This function must check & set the busy state for reentrancy
	 *
	 * On success StopRx must be called to release busy flag
	 *
	 * @param	DevAddr   : The device selection id scheme
	 *
	 * @return 	true - Success\n
	 * 			false - failed
	 */
	virtual bool StartTx(int DevAddr) = 0;

	/**
	 * @brief	Transfer data from pData passed in parameter.  Assuming StartTx was
	 * called prior calling this function to send the actual data
	 *
	 * @param	pData 	: Pointer to memory area of data to send.
	 * @param	DataLen : Length of data memory in bytes
	 *
	 * @return	Number of bytes sent
	 */
	virtual int TxData(uint8_t *pData, int DataLen) = 0;

	// Stop transmit
	// WARNING !!!!!
	// This functions MUST ONLY be called if StartTx returns true.
	/**
	 * @brief	Completion of sending data via TxData.
	 *
	 * Do require post processing
	 * after all data was transmitted via TxData.
	 * This function must clear the busy state for reentrancy
	 * Call this function only if StartTx was successful.
	 */
	virtual void StopTx(void) = 0;

	virtual bool RequestToSend(int NbBytes) { return true; }

	/**
	 * @brief	This function perform a reset of the interface.
	 */
	virtual void Reset(void) { DeviceIntrfReset(*this); }
};

#endif

/** @} end group device_intrf */

#endif	// __DEVICEINTRF_H__
