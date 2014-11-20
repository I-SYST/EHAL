/*--------------------------------------------------------------------------
File   : lpcuart.h

Author : Hoang Nguyen Hoan          Jan. 16, 2012

Desc   : LPC UART implementation

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
#ifndef __LPCUART_H__
#define __LPCUART_H__

#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"

// IER interrupt enable
#define LPCUART_IER_RBR			1 		// RBR Interrupt enable
#define LPCUART_IER_THRE		2		// THRE Interrupt enable
#define LPCUART_IER_RX			4		// RX Line Status Interrupt enable
#define LPCUART_IER_ABEO 		0x10	// ABEOIntEn Enables the end of auto-baud interrupt.
#define LPCUART_IER_ABTO		0x20	// ABTOIntEn Enables the auto-baud time-out interrupt.

// IIR
#define LPCUART_IIR_STATUS 		1		// IntStatus Interrupt status. Note that UnIIR[0] is active low.
										// 0 At least one interrupt is pending.
										// 1 No interrupt is pending.
#define LPCUART_IIR_ID_MASK		0xe		// IntId Interrupt identification.
										// 011 1 - Receive Line Status (RLS).
										// 010 2a - Receive Data Available (RDA).
										// 110 2b - Character Time-out Indicator (CTI).
										// 001 3 - THRE Interrupt
#define LPCUART_IIR_FIFO_MASK	0xc0	// FIFO Enable Copies of UnFCR[0].
#define LPCUART_IIR_ABEO 		0x100	// End of auto-baud interrupt. True if auto-baud has finished successfully and
										// interrupt is enabled.

#define LPCUART_IIR_ABTO		0x200	// Auto-baud time-out interrupt. True if auto-baud has timed out and interrupt is
										// enabled.

// FCR - FIFO Control Register
#define LPCUART_FCR_FIFOEN 			1	// 0 - UARTn FIFOs are disabled. Must not be used in the application.
										// 1 Active high enable for both UARTn Rx and TX FIFOs and UnFCR[7:1] access.
										// This bit must be set for proper UART operation. Any transition on this bit will
										// automatically clear the related UART FIFOs.
#define LPCUART_FCR_RST_RXFIFO		2	// 0 - No impact on either of UARTn FIFOs.
										// 1 - Writing a logic 1 to UnFCR[1] will clear all bytes in UARTn Rx FIFO, reset the
										// pointer logic. This bit is self-clearing.
#define LPCUART_FCR_RST_TXFIFO		4	// 0 - No impact on either of UARTn FIFOs.
										// 1 - Writing a logic 1 to UnFCR[2] will clear all bytes in UARTn TX FIFO, reset the
										// pointer logic. This bit is self-clearing.
#define LPCUART_FCR_DMA_MODE		8	// DMA Mode
#define LPCUART_FCR_RX_TRIG_MASK	0xc0	// RX Trigger Level. These two bits determine how many receiver UARTn FIFO characters must be
											// written before an interrupt or DMA request is activated.
#define LPCUART_FCR_RX_TRIG1		0		// 00 Trigger level 0 (1 character or 0x01)
#define LPCUART_FCR_RX_TRIG4		1		// 01 Trigger level 1 (4 characters or 0x04)
#define LPCUART_FCR_RX_TRIG8		2		// 10 Trigger level 2 (8 characters or 0x08)
#define LPCUART_FCR_RX_TRIG14		3		// 11 Trigger level 3 (14 characters or 0x0E)

// LCR - Line Control Register
#define LPCUART_LCR_WLEN_MASK		3		// Word Length Select
											// 00 5-bit character length
											// 01 6-bit character length
											// 10 7-bit character length
											// 11 8-bit character length
#define LPCUART_LCR_STOPBIT_MASK	4 		// Stop Bit Select
											// 0 : 1 stop bit. 0
											// 1 : 2 stop bits (1.5 if UnLCR[1:0]=00).
#define LPCUART_LCR_PAR_EN			8		// Parity Enable 0 Disable parity generation and checking.
											// 1 Enable parity generation and checking.
#define LPCUART_LCR_PAR_MASK 		0x30 	// Parity Select
											// 00 Odd parity. Number of 1s in the transmitted character and the attached
											// parity bit will be odd.
											// 01 Even Parity. Number of 1s in the transmitted character and the attached
											// parity bit will be even.
											// 10 Forced "1" stick parity.
											// 11 Forced "0" stick parity.
#define LPCUART_LCR_BRK 			0x40 	// Disable break transmission.
											// 1 Enable break transmission. Output pin UARTn TXD is forced to logic 0
											// when UnLCR[6] is active high.
#define LPCUART_LCR_DLAB			0x80	// Divisor Latch Access Bit (DLAB)
											// 0 - Disable access to Divisor Latches.
											// 1 - Enable access to Divisor Latches.

// LSR - Line Status Register
#define LPCUART_LSR_RDR				1		// Receiver Data Ready (RDR)
											// UnLSR0 is set when the UnRBR holds an unread character and is cleared when
											// the UARTn RBR FIFO is empty.
											// 0 The UARTn receiver FIFO is empty.
											// 1 The UARTn receiver FIFO is not empty.
#define LPCUART_LSR_OE				2		// Overrun Error (OE)
											// The overrun error condition is set as soon as it occurs. An UnLSR read clears
											// UnLSR1. UnLSR1 is set when UARTn RSR has a new character assembled and
											// the UARTn RBR FIFO is full. In this case, the UARTn RBR FIFO will not be
											// overwritten and the character in the UARTn RSR will be lost.
											// 0 Overrun error status is inactive.
											// 1 Overrun error status is active.
#define LPCUART_LSR_PE				4		// Parity Error (PE)
											// Note: A parity error is associated with the character at the top of the UARTn RBR FIFO.
											// 0 Parity error status is inactive.
											// 1 Parity error status is active.
#define LPCUART_LSR_FE				8		// Framing Error (FE)
											// 0 Framing error status is inactive.
											// 1 Framing error status is active.
#define LPCUART_LSR_BI				0x10	// Break Interrupt (BI)
											// 0 Break interrupt status is inactive.
											// 1 Break interrupt status is active.
#define LPCUART_LSR_THRE			0x20	// Transmitter Holding Register Empty (THRE))
											// 0 UnTHR contains valid data.
											// 1 UnTHR is empty.
#define LPCUART_LSR_TEMT			0x40	// Transmitter Empty (TEMT)
											// 0 UnTHR and/or the UnTSR contains valid data.
											// 1 UnTHR and the UnTSR are empty.
#define LPCUART_LSR_RXFE			0x80	// Error in RX FIFO (RXFE)
											// 0 UnRBR contains no UARTn RX errors or UnFCR[0]=0.
											// 1 UARTn RBR contains at least one UARTn RX error.

// ACR - Auto-baud Control Register
#define LPCUART_ACR_START			1		// Start This bit is automatically cleared after auto-baud completion.
											// 0 Auto-baud stop (auto-baud is not running).
											// 1 Auto-baud start (auto-baud is running). Auto-baud run bit. This bit is
											// automatically cleared after auto-baud completion.
#define LPCUART_ACR_MODE			2		// Mode Auto-baud mode select bit.
											// 0 Mode 0.
											// 1 Mode 1.
#define LPCUART_ACR_AUTORESTART		4		// AutoRestart 0 No restart.
											// 1 Restart in case of time-out (counter restarts at next UARTn Rx falling edge) 0
#define LPCUART_ACR_ABEO_INTCLR		0x100	// ABEOIntClr End of auto-baud interrupt clear bit (write-only accessible). Writing a 1 will
											// clear the corresponding interrupt in the UnIIR. Writing a 0 has no impact.
#define LPCUART_ACR_ABTO_INTCLR		0x200	// ABTOIntClr Auto-baud time-out interrupt clear bit (write-only accessible). Writing a 1 will
											// clear the corresponding interrupt in the UnIIR. Writing a 0 has no impact.

#define LPCUART_ICR_IRDAEN			1 		// Enable IrDA mode
#define LPCUART_ICR_IRDAINV			2		// When 1, the serial input is inverted. This has no effect on the serial output.
											// When 0, the serial input is not inverted.
#define LPCUART_ICR_IRDA_FIXPULSE	4		// Enable fix pulse
#define LPCUART_ICR_IRDA_PULSEDIV_MASK	0x38	//

#define LPCUART_TER_TXEN			0x80	// Start tx

typedef enum {
	UART_STATUS_RDR = LPCUART_LSR_RDR,		// Receive data ready
	UART_STATUS_OE = LPCUART_LSR_OE,		// Overun error
	UART_STATUS_PE = LPCUART_LSR_PE,		// Parity error
	UART_STATUS_FE = LPCUART_LSR_FE,		// Framing error
	UART_STATUS_BI = LPCUART_LSR_BI,		// Break condition
	UART_STATUS_THRE = LPCUART_LSR_THRE,
	UART_STATUS_RX_FIFO = LPCUART_LSR_RXFE	// Receive FIFO error
} UART_STATUS;

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

#pragma pack(push, 1)

// Partial common UART register mapping
typedef struct {
	union {
		uint32_t DLL;                      /*!< (@ 0x40008000) Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider. (DLAB=1) */
		uint32_t THR;                      /*!< (@ 0x40008000) Transmit Holding Register. The next character to be transmitted is written here. (DLAB=0) */
		uint32_t RBR;                      /*!< (@ 0x40008000) Receiver Buffer Register. Contains the next received character to be read. (DLAB=0) */
	};

	union {
		uint32_t IER;                      /*!< (@ 0x40008004) Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential USART interrupts. (DLAB=0) */
		uint32_t DLM;                      /*!< (@ 0x40008004) Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider. (DLAB=1) */
	};

	union {
		uint32_t FCR;                      /*!< (@ 0x40008008) FIFO Control Register. Controls USART FIFO usage and modes. */
		uint32_t IIR;                      /*!< (@ 0x40008008) Interrupt ID Register. Identifies which interrupt(s) are pending. */
	};
	uint32_t LCR;                        /*!< (@ 0x4000800C) Line Control Register. Contains controls for frame formatting and break generation. */
	uint32_t MCR;                        /*!< (@ 0x40008010) Modem Control Register. */
	uint32_t LSR;                        /*!< (@ 0x40008014) Line Status Register. Contains flags for transmit and receive status, including line errors. */
	uint32_t MSR;                        /*!< (@ 0x40008018) Modem Status Register. */
	uint32_t SCR;                        /*!< (@ 0x4000801C) Scratch Pad Register. Eight-bit temporary storage for software. */
	uint32_t ACR;                        /*!< (@ 0x40008020) Auto-baud Control Register. Contains controls for the auto-baud feature. */
	uint32_t ICR;                        /*!< (@ 0x40008024) IrDA Control Register. Enables and configures the IrDA (remote control) mode. */
	uint32_t FDR;                        /*!< (@ 0x40008028) Fractional Divider Register. Generates a clock input for the baud rate divider. */
	uint32_t OSR;                        /*!< (@ 0x4000802C) Oversampling Register. Controls the degree of oversampling during each bit time. */
	uint32_t TER;                        /*!< (@ 0x40008030) Transmit Enable Register. Turns off USART transmitter for use with software flow control. */

} LPCUARTREG;

#pragma pack(pop)

// Possible baudrate values
// 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
// 230400, 460800, 921600

#define LPCUART_NB_PINS			8

#pragma pack(push, 4)

typedef struct {
	int DevNo;					// UART device number
	IOPINCFG PinCfg[LPCUART_NB_PINS];	// I/O pin to configure for UART
	int Rate;					// Baudrate, set to 0 for auto baudrate
	int DataBits;				// Number of data bits
	UART_PARITY Parity;			// Data parity
	int StopBits;				// Number of stop bits
	UART_FLWCTRL FlowControl;	//
	bool DMAMode;				// DMA transfer support
	bool IrDAMode;				// Enable IrDA
	bool IrDAInvert;			// IrDA input inverted
	bool IrDAFixPulse;			// Enable IrDA fix pulse
	int	IrDAPulseDiv;			// Fix pulse divider
} UARTCFG;

typedef struct {
	LPCUARTREG *pUartReg;				// Pointer to UART registers
	UARTCFG Cfg;
} UARTDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

// Require implementation for each processor type
bool LpcUARTInit(UARTDEV *pDev, const UARTCFG *pCfg);

// Common to all LPC series
int LpcUARTGetRate(UARTDEV *pDev);
int LpcUARTSetRate(UARTDEV *pDev, int Rate);
int LpcUARTRxData(UARTDEV *pDev, uint8_t *pBuff, int Bufflen);
bool LpcUARTStartTx(UARTDEV *pDev);
int LpcUARTTxData(UARTDEV *pDev, uint8_t *pData, int Datalen);
void LpcUARTStopTx(UARTDEV *pDev);
bool LpcUARTWaitForRxFifo(UARTDEV *pDev, int Timeout);
bool LpcUARTWaitForTxFifo(UARTDEV *pDev, int Timeout);
UART_STATUS LpcUARTGetStatus(UARTDEV *pDev);
void LpcUARTprintf(UARTDEV *pDev, char *pFormat, ...);
void LpcUARTvprintf(UARTDEV *pDev, char *pFormat, va_list vl);

#ifdef __cplusplus
}

#include "serialintrf.h"

// C++ class wrapper
class LpcUART: public SerialIntrf {
public:
	LpcUART() {
		memset(&vDevData, 0, sizeof(vDevData));
	}
	virtual ~LpcUART() {}
	LpcUART(LpcUART&);

	bool Init(const UARTCFG &CfgData) {
		return LpcUARTInit(&vDevData, &CfgData);
	}
	// ++ ** Require implementation for SerialCom
	// Set data baudrate
	virtual int Rate(int DataRate) { return LpcUARTSetRate(&vDevData, DataRate); }
	// Get current data baudrate
	virtual int Rate(void) { return LpcUARTGetRate(&vDevData); }
	// Initiate receive
	virtual bool StartRx(int DevAddr) { return true; }
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return LpcUARTRxData(&vDevData, pBuff, BuffLen);
	}
	// Stop receive
	virtual void StopRx(void) {}
	// Initiate transmit
	virtual bool StartTx(int DevAddr) { return LpcUARTStartTx(&vDevData); }
	// Transmit Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return LpcUARTTxData(&vDevData, pData, DataLen);
	}
	// Stop transmit
	virtual void StopTx(void) { LpcUARTStopTx(&vDevData); }
	// -- **
	void printf(char *pFormat, ...) {
		va_list vl;
	    va_start(vl, pFormat);
	    LpcUARTvprintf(&vDevData, pFormat, vl);
	    va_end(vl);
	}

	operator UARTDEV * () { return &vDevData; }

private:
	UARTDEV	vDevData;
};

#endif

#endif // __LPCUART_H__

