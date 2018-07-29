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

#include "coredev/iopincfg.h"
#include "coredev/uart.h"

// IER interrupt enable
#define LPCUART_IER_RBR			1 		// RBR Interrupt enable
#define LPCUART_IER_THRE		2		// THRE Interrupt enable
#define LPCUART_IER_RLS			4		// RX Line Status Interrupt enable
#define LPCUART_IER_MS			8		// Modem Status interrupt enable
#define LPCUART_IER_ABEO 		0x10	// ABEOIntEn Enables the end of auto-baud interrupt.
#define LPCUART_IER_ABTO		0x20	// ABTOIntEn Enables the auto-baud time-out interrupt.

// IIR
#define LPCUART_IIR_STATUS 		1		// IntStatus Interrupt status. Note that UnIIR[0] is active low.
										// 0 At least one interrupt is pending.
										// 1 No interrupt is pending.
#define LPCUART_IIR_ID_MASK		(7<<1)		// IntId Interrupt identification.
										// 011 1 - Receive Line Status (RLS).
										// 010 2a - Receive Data Available (RDA).
										// 110 2b - Character Time-out Indicator (CTI).
										// 001 3 - THRE Interrupt
										// 000 4 - Modem status
#define LPCUART_IIR_ID_MS		(0<<1)	// Modem status
#define LPCUART_IIR_ID_THRE		(1<<1)	// THRE Interrupt.
#define LPCUART_IIR_ID_RDA		(2<<1)	// Receive Data Available (RDA)
#define LPCUART_IIR_ID_RLS		(3<<1)	// Receive Line Status (RLS).
#define LPCUART_IIR_ID_CTIMOUT	(6<<1)	// Character Time-out Indicator (CTI).

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
#define LPCUART_FCR_RX_TRIG_MASK	(3<<6)	// RX Trigger Level. These two bits determine how many receiver UARTn FIFO characters must be
											// written before an interrupt or DMA request is activated.
#define LPCUART_FCR_RX_TRIG1		(0<<6)		// 00 Trigger level 0 (1 character or 0x01)
#define LPCUART_FCR_RX_TRIG4		(1<<6)		// 01 Trigger level 1 (4 characters or 0x04)
#define LPCUART_FCR_RX_TRIG8		(2<<6)		// 10 Trigger level 2 (8 characters or 0x08)
#define LPCUART_FCR_RX_TRIG14		(3<<6)		// 11 Trigger level 3 (14 characters or 0x0E)

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

// MCR - Modem control register
#define LPCUART_MCR_DTRCTRL			(1<<0)	// Source for modem output pin DTR. This bit reads as 0 when modem loopback
											// mode is active
#define LPCUART_MCR_RTSCTRL			(1<<1)	// Source for modem output pin RTS. This bit reads as 0 when modem loopback
											// mode is active.
#define LPCUART_MCR_LMS				(1<<4)	// Loopback Mode Select.
#define LPCUART_MCR_RTSEN			(1<<6)	// RTS enable
#define LPCUART_MCR_CTSEN			(1<<7)	// CTS enable

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

#define LPCUART_MSR_DCTS			1		// Delta CTS. 0 Set upon state change of input CTS. Cleared on an MSR read.
											// 	0 - No change detected on modem input, CTS.
											//	1 - State change detected on modem input, CTS.
#define LPCUART_MSR_DDSR			2		// Delta DSR. 0 Set upon state change of input DSR. Cleared on an MSR read.
											// 	0 - No change detected on modem input, DSR.
											//	1 - State change detected on modem input, DSR.
#define LPCUART_MSR_TERI			4		// Trailing Edge RI. 0 Set upon low to high transition of input RI. Cleared on an
											// MSR read.
											// 	0 - No change detected on modem input, RI.
											//	1 - Low-to-high transition detected on RI.
#define LPCUART_MSR_DDCD			8		// Delta DCD. Set upon state change of input DCD. Cleared on 0 an MSR read.
											//	0 - No change detected on modem input, DCD.
											//	1 - State change detected on modem input, DCD.
#define LPCUART_MSR_CTS				0x10	// Clear To Send State. Complement of input signal CTS. This 0 bit is connected to MCR[1]
											// in modem loopback mode.
#define LPCUART_MSR_DSR				0x20	// Data Set Ready State. Complement of input signal DSR. 0 This bit is connected to MCR[0]
											// in modem loopback mode.
#define LPCUART_MSR_RI				0x40	// Ring Indicator State. Complement of input RI. This bit is 0 connected to MCR[2]
											// in modem loopback mode.
#define LPCUART_MSR_DCD				0x80	// Data Carrier Detect State. Complement of input DCD. This 0 bit is connected to MCR[3]
											// in modem loopback mode.

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


// Partial common UART register mapping
typedef struct {
	union {
		volatile uint32_t DLL;                      /*!< (@ 0x40008000) Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider. (DLAB=1) */
		volatile uint32_t THR;                      /*!< (@ 0x40008000) Transmit Holding Register. The next character to be transmitted is written here. (DLAB=0) */
		volatile uint32_t RBR;                      /*!< (@ 0x40008000) Receiver Buffer Register. Contains the next received character to be read. (DLAB=0) */
	};

	union {
		volatile uint32_t IER;                      /*!< (@ 0x40008004) Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential USART interrupts. (DLAB=0) */
		volatile uint32_t DLM;                      /*!< (@ 0x40008004) Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider. (DLAB=1) */
	};

	union {
		volatile uint32_t FCR;                      /*!< (@ 0x40008008) FIFO Control Register. Controls USART FIFO usage and modes. */
		volatile uint32_t IIR;                      /*!< (@ 0x40008008) Interrupt ID Register. Identifies which interrupt(s) are pending. */
	};
	volatile uint32_t LCR;                        /*!< (@ 0x4000800C) Line Control Register. Contains controls for frame formatting and break generation. */
	volatile uint32_t MCR;                        /*!< (@ 0x40008010) Modem Control Register. */
	volatile uint32_t LSR;                        /*!< (@ 0x40008014) Line Status Register. Contains flags for transmit and receive status, including line errors. */
	volatile uint32_t MSR;                        /*!< (@ 0x40008018) Modem Status Register. */
	volatile uint32_t SCR;                        /*!< (@ 0x4000801C) Scratch Pad Register. Eight-bit temporary storage for software. */
	volatile uint32_t ACR;                        /*!< (@ 0x40008020) Auto-baud Control Register. Contains controls for the auto-baud feature. */
	volatile uint32_t ICR;                        /*!< (@ 0x40008024) IrDA Control Register. Enables and configures the IrDA (remote control) mode. */
	volatile uint32_t FDR;                        /*!< (@ 0x40008028) Fractional Divider Register. Generates a clock input for the baud rate divider. */
	volatile uint32_t OSR;                        /*!< (@ 0x4000802C) Oversampling Register. Controls the degree of oversampling during each bit time. */
	volatile uint32_t TER;                        /*!< (@ 0x40008030) Transmit Enable Register. Turns off USART transmitter for use with software flow control. */

} LPCUARTREG;

// Device driver data require by low level functions
typedef struct _PLC_UART_Dev {
	int DevNo;					// UART interface number
	LPCUARTREG *pUartReg;		// Pointer to UART register map
	bool DMAMode;				// DMA transfer support
	UARTDEV	*pUartDev;			// Pointer to generic UART dev. data
	volatile bool bTxReady;
} LPCUARTDEV;


#ifdef __cplusplus
extern "C" {
#endif

// Common to all LPC series
uint32_t LpcGetUartClk();
void LpcUARTDisable(DEVINTRF *pDev);
void LpcUARTEnable(DEVINTRF *pDev);
int LpcUARTGetRate(DEVINTRF *pDev);
int LpcUARTSetRate(DEVINTRF *pDev, int Rate);
static inline bool LpcUARTStartRx(DEVINTRF *pSerDev, int DevAddr) { return true; }
int LpcUARTRxData(DEVINTRF *pDev, uint8_t *pBuff, int Bufflen);
static inline void LpcUARTStopRx(DEVINTRF *pSerDev) {}
static inline bool LpcUARTStartTx(DEVINTRF *pDev, int DevAddr) { return true; }
int LpcUARTTxData(DEVINTRF *pDev, uint8_t *pData, int Datalen);
static inline void LpcUARTStopTx(DEVINTRF *pDev) {}
bool LpcUARTWaitForRxFifo(LPCUARTDEV *pDev, uint32_t Timeout);
bool LpcUARTWaitForTxFifo(LPCUARTDEV *pDev, uint32_t Timeout);
UART_STATUS LpcUARTGetStatus(LPCUARTDEV *pDev);

#ifdef __cplusplus
}
#endif

#endif // __LPCUART_H__

