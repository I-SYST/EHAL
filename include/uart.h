/*--------------------------------------------------------------------------
File   : uart.h

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : Generic uart definitions
		 Current implementation

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
#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "serialintrf.h"
#include "cfifo.h"

// Possible baudrate values
// 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
// 230400, 250000, 460800, 500000, 576000, 921600 1000000, 2000000,
// 3000000


typedef enum {
	UART_PARITY_NONE 	= -1,
	UART_PARITY_ODD 	= 0,
	UART_PARITY_EVEN 	= 1,
	UART_PARITY_MARK	= 2,
	UART_PARITY_SPACE	= 3
} UART_PARITY;

typedef enum {
	UART_FLWCTRL_NONE,
	UART_FLWCTRL_XONXOFF,
	UART_FLWCTRL_HW,
} UART_FLWCTRL;

#define UART_LINESTATE_DCD		(1<<0)		// Rx Carrier detect
#define UART_LINESTATE_DSR		(1<<1)		// Tx Carrier detect (CTS/RTS)
#define UART_LINESTATE_BRK		(1<<2)		// Break
#define UART_LINESTATE_RI		(1<<3)		// Ring detect
#define UART_LINESTATE_FRMERR	(1<<4)		// Frame error
#define UART_LINESTATE_PARERR	(1<<5)		// Parity error
#define UART_LINESTATE_OVR		(1<<6)		// Overrun
//#define UART_LINESTATE_CTS		(1<<7)
//#define UART_LINESTATE_RTS		(1<<8)

#define UART_NB_PINS			8

// I/O pin configuration list ordering
#define UARTPIN_RX_IDX			0
#define UARTPIN_TX_IDX			1
#define UARTPIN_CTS_IDX			2
#define UARTPIN_RTS_IDX			3
#define UARTPIN_DCD_IDX			4
#define UARTPIN_DTE_IDX			5
#define UARTPIN_DTR_IDX			6
#define UARTPIN_RI_IDX			7

typedef struct __Uart_Dev UARTDEV;

typedef enum {
	UART_EVT_RXTIMEOUT,
	UART_EVT_RXDATA,
	UART_EVT_TXREADY,
	UART_EVT_LINESTATE,
} UART_EVT;

/**
 * @brief
 *
 * Event handler callback. This is normally being called within interrupts, avoid blocking
 *
 * @param pDev : Device handle
 * @param EvtId : Event code
 * @param pBuffer : In/Out Buffer containing data
 * 					on UART_EVT_RXTIMEOUT & UART_EVT_RXDATA, buffer contains data received. If
 * 					driver implements CFIFO, this parameter is NULL with BufferLen indicating total data
 * 					in fifo.
 * 					on UART_EVT_TXREADY, buffer allocated for data to be transmit with max length
 * 					BufferLen (fifo size). If driver implements CFIFO, this parameter is NULL and BufferLen
 * 					indicates max available space in fifo
 * 					on UART_EVT_LINESTATE, buffer contains 1 byte Line Status
 *
 * @param BufferLen : Max buffer length.  See above description
 *
 * @return number of bytes written to pBuffer for transmit
 */
typedef int (*UARTEVTCB)(UARTDEV*pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#pragma pack(push, 4)

// Configuration data used to initialize device
typedef struct {
	int DevNo;					// UART device number
	const void *pIoMap;			// Pointer to IO mapping.  This can be either IOPINCFG array or device path string
	int IoMapLen;				// Nb of elements in IOPINCFG array or string length of device path
	//IOPINCFG PinCfg[UART_NB_PINS];	// I/O pin to configure for UART
	int Rate;					// Baudrate, set to 0 for auto baudrate
	int DataBits;				// Number of data bits
	UART_PARITY Parity;			// Data parity
	int StopBits;				// Number of stop bits
	UART_FLWCTRL FlowControl;	// Flow control
	bool bIntMode;				// Interrupt mode support
	int IntPrio;				// Interrupt priority
	UARTEVTCB EvtCallback;		// UART event callback
	int RxMemSize;
	uint8_t *pRxMem;			// Pointer to memory allocated for RX FIFO
	int TxMemSize;
	uint8_t *pTxMem;			// Pointer to memory allocated for TX FIFO
	bool bDMAMode;				// DMA transfer support
	bool bIrDAMode;				// Enable IrDA
	bool bIrDAInvert;			// IrDA input inverted
	bool bIrDAFixPulse;			// Enable IrDA fix pulse
	int	IrDAPulseDiv;			// Fix pulse divider
} UARTCFG;

// Device driver data require by low level functions
struct __Uart_Dev {
	int Rate;					// Baudrate, set to 0 for auto baudrate
	int DataBits;				// Number of data bits
	UART_PARITY Parity;			// Data parity
	int StopBits;				// Number of stop bits
	UART_FLWCTRL FlowControl;	//
	bool bIntMode;				// Interrupt mdoe
	bool bIrDAMode;				// Enable IrDA
	bool bIrDAInvert;			// IrDA input inverted
	bool bIrDAFixPulse;			// Enable IrDA fix pulse
	int	IrDAPulseDiv;			// Fix pulse divider
	SERINTRFDEV	SerIntrf;		// Serial device interface implementation
	UARTEVTCB EvtCallback;		// UART event callback
	void *pObj;					// Pointer to UART object instance
	HCFIFO hRxFifo;				// Rx FIFO handle
	HCFIFO hTxFifo;				// Tx Fifo handle
	uint32_t LineState;			// Line state
	int hStdIn;					// Handle to retarget stdin
	int hStdOut;				// Handle to retarget stdout
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif	// __cplusplus

// Require implementations
bool UARTInit(UARTDEV *pDev, const UARTCFG *pCfgData);
void UARTSetCtrlLineState(UARTDEV *pDev, uint32_t LineState);

inline int UARTGetRate(UARTDEV *pDev) { return pDev->SerIntrf.GetRate(&pDev->SerIntrf); }
inline int UARTSetRate(UARTDEV *pDev, int Rate) { return pDev->SerIntrf.SetRate(&pDev->SerIntrf, Rate); }
int UARTRx(UARTDEV *pDev, uint8_t *pBuff, int Bufflen);
int UARTTx(UARTDEV *pDev, uint8_t *pData, int Datalen);
void UARTprintf(UARTDEV *pDev, const char *pFormat, ...);
void UARTvprintf(UARTDEV *pDev, const char *pFormat, va_list vl);
void UARTRetargetEnable(UARTDEV *pDev, int FileNo);
void UartRetargetDisable(UARTDEV *pDev, int FileNo);

#ifdef __cplusplus
}

// C++ class wrapper

// C++ class wrapper
class UART: public SerialIntrf {
public:
	UART() {
		memset(&vDevData, 0, sizeof(vDevData));
		vDevData.pObj = this;
	}
	virtual ~UART() {}
	UART(UART&);

	virtual bool Init(const UARTCFG &CfgData) {
		return UARTInit(&vDevData, &CfgData);
	}
	// ++ ** Require implementation
	// Set data baudrate
	virtual int Rate(int DataRate) { return UARTSetRate(&vDevData, DataRate); }
	// Get current data baudrate
	virtual int Rate(void) { return UARTGetRate(&vDevData); }
	virtual void SetCtrlLineState(int LineState) { UARTSetCtrlLineState(&vDevData, LineState); }
	virtual int Rx(uint8_t *pBuff, uint32_t Len) { return ((SerialIntrf *)this)->Rx(0, pBuff, Len); }
	// Initiate receive
	virtual bool StartRx(int DevAddr) { return SerialIntrfStartRx(&vDevData.SerIntrf, DevAddr); }
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return SerialIntrfRxData(&vDevData.SerIntrf, pBuff, BuffLen);
	}
	// Stop receive
	virtual void StopRx(void) { SerialIntrfStopRx(&vDevData.SerIntrf); }
	virtual int Tx(uint8_t *pData, uint32_t Len) { return ((SerialIntrf*)this)->Tx(0, pData, Len); }
	// Initiate transmit
	virtual bool StartTx(int DevAddr) { return SerialIntrfStartTx(&vDevData.SerIntrf, DevAddr); }
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return SerialIntrfTxData(&vDevData.SerIntrf, pData, DataLen);
	}
	// Stop transmit
	virtual void StopTx(void) { SerialIntrfStopTx(&vDevData.SerIntrf); }
	// -- **
	void printf(const char *pFormat, ...) {
		va_list vl;
	    va_start(vl, pFormat);
	    UARTvprintf(&vDevData, pFormat, vl);
	    va_end(vl);
	}

	operator UARTDEV * () { return &vDevData; }

private:
	UARTDEV	vDevData;
};


#endif	// __cplusplus

#endif	// __UART_H__
