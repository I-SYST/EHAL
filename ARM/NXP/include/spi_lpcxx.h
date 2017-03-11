/*--------------------------------------------------------------------------
File   : spi_lpc11.h

Author : Hoang Nguyen Hoan          Nov. 24, 2011

Desc   : Synchronous Serial Port(SSP) implementation on LPC

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
Modified by         Date            Description
Hoan				Feb. 20, 2015	New EHAL
----------------------------------------------------------------------------*/
#ifndef __LPCSSP_H__
#define __LPCSSP_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "spi.h"

#define LPC_PCLKSEL1_SSP0_MASK	(3 << 10)	// On PCLKSEL1
#define LPC_PCLKSEL1_SSP0_DIV1	(1 << 10)
#define LPC_PCLKSEL1_SSP0_DIV2	(2 << 10)
#define LPC_PCLKSEL1_SSP0_DIV4	(0)
#define LPC_PCLKSEL1_SSP0_DIV8	(3 << 10)

#define LPC_PCLKSEL0_SSP1_MASK	(3 << 20)	// On PCLKSEL0
#define LPC_PCLKSEL0_SSP1_DIV1	(1 << 20)
#define LPC_PCLKSEL0_SSP1_DIV2	(2 << 20)
#define LPC_PCLKSEL0_SSP1_DIV4	(0)
#define LPC_PCLKSEL0_SSP1_DIV8	(3 << 20)

// SSP_CR0 register
#define LPCSSP_CR0_FRF_SPI		0
#define LPCSSP_CR0_FRF_TI		0x10
#define LPCSSP_CR0_FRF_MWIRE	0x20
#define LPCSSP_CR0_CPOL_LO		0
#define LPCSSP_CR0_CPOL_HI		0x40
#define LPCSSP_CR0_CPHA_FIRST	0		// Capture data on first clock
#define LPCSSP_CR0_CPHA_SECOND	0x80	// Capture data on second clock
#define LPCSSP_CR0_SCR_MASK		0xff00	// Clock rate mask

// SSP_CR1 register
#define LPCSSP_CR1_LBM_EN		1		// Enable loop back mode
#define LPCSSP_CR1_SSP_EN		2		// SSP enable
#define LPCSSP_CR1_MS			4		// Master/Slave mode
#define LPCSSP_CR1_MS_MASTER	0
#define LPCSSP_CR1_MS_SLAVE		4
#define LPCSSP_CR1_SOD			8		// Slave output disable

// SSP DATA register
#define LPCSSP_DR_MASK			0xffff

// SSP STATUS register
#define LPCSSP_SR_TFE			1		// Transmit FIFO Empty. This bit is 1 the Transmit FIFO is empty
#define LPCSSP_SR_TNF 			2		// Transmit FIFO Not Full. This bit is 0 if the Tx FIFO is full
#define LPCSSP_SR_RNE 			4		// Receive FIFO Not Empty. This bit is 0 if the Receive FIFO is empty
#define LPCSSP_SR_RFF 			8		// Receive FIFO Full. This bit is 1 if the Receive FIFO is full
#define LPCSSP_SR_BSY 			0x10	// Busy. This bit is 1 if it is currently sending/receiving a frame
										// and/or the Tx FIFO is not empty

typedef struct {                  		/*!< (@ 0x40040000) SSP0 Structure         */
	volatile uint32_t CR0;             	/*!< (@ 0x40040000) Control Register 0. Selects the serial clock rate, bus type, and data size. */
	volatile uint32_t CR1;              /*!< (@ 0x40040004) Control Register 1. Selects master/slave and other modes. */
	volatile uint32_t DR;				/*!< (@ 0x40040008) Data Register. Writes fill the transmit FIFO, and reads empty the receive FIFO. */
	volatile uint32_t SR;             	/*!< (@ 0x4004000C) Status Register        */
	volatile uint32_t CPSR;           	/*!< (@ 0x40040010) Clock Prescale Register */
	volatile uint32_t IMSC;         	/*!< (@ 0x40040014) Interrupt Mask Set and Clear Register */
	volatile uint32_t RIS;              /*!< (@ 0x40040018) Raw Interrupt Status Register */
	volatile uint32_t MIS;              /*!< (@ 0x4004001C) Masked Interrupt Status Register */
	volatile uint32_t ICR;              /*!< (@ 0x40040020) SSPICR Interrupt Clear Register */
} LPCSSPREG;


typedef struct {
	int 		DevNo;			// SSP device number
	uint32_t	PClkFreq;		// Peripheral clock freq in Hz
	LPCSSPREG 	*pSspReg;
	SPIDEV		*pSpiDev;		// Pointer to generic SPI dev. data
} SSPDEV;

#ifdef __cplusplus
extern "C" {
#endif

static inline void LpcSSPDisable(DEVINTRF *pDev) {}
static inline void LpcSSPEnable(DEVINTRF *pDev) {}

// Get current data rate in bits/sec (Hz)
int LpcSSPGetRate(DEVINTRF *pDev);
// Set data rate in bits/sec (Hz)
int LpcSSPSetRate(DEVINTRF *pDev, int DataRate);
// Initiate receive
bool LpcSSPStartRx(DEVINTRF *pDev, int DevAddr);
// Receive Data only, no Start/Stop condition
int LpcSSPRxData(DEVINTRF *pDev, uint8_t *pBuff, int BuffLen);
// Stop receive
void LpcSSPStopRx(DEVINTRF *pDev);
// Receive stream
//int LpcSSPRx(SSPDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen);

// Initiate transmit
bool LpcSSPStartTx(DEVINTRF *pDev, int DevAddr);
// Transmit Data only, no Start/Stop condition
int LpcSSPTxData(DEVINTRF *pDev, uint8_t *pData, int DataLen);
// Stop transmit
void LpcSSPStopTx(DEVINTRF *pDev);
// Transmit stream
//int LpcSSPTx(SSPDEV *pDev, int DevAddr, uint8_t *pData, int DataLen);

bool LpcSSPWaitBusy(SSPDEV *pDev, int TimeoutCnt);
bool LpcSSPWaitRxFifo(SSPDEV *pDev, int TimeoutCnt);
bool LpcSSPWaitTxFifo(SSPDEV *pDev, int TimeoutCnt);

#ifdef __cplusplus
}


#endif // __cplusplus

#endif // __LPCSSP_H__

