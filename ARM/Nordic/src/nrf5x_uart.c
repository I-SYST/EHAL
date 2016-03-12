/*--------------------------------------------------------------------------
File   : nrf5x_uart.c

Author : Hoang Nguyen Hoan          Aug. 30, 2015

Desc   : nRF5x UART implementation

Copyright (c) 2015, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#include "nrf51_bitfields.h"
#include "nrf51.h"
#include "nrf_gpio.h"
//#include "nrf_drv_gpiote.h"

#include "istddef.h"
#include "nrf5x_uart.h"
#include "idelay.h"

extern char s_Buffer[];	// defined in sbuffer.c
extern int s_BufferSize;

#define NRF51UART_FIFO_MAX		6
#define NRF51UART_RXTIMEOUT		15

typedef struct {
	int Baud;
	int nRFBaud;
} NRFRATECVT;

const NRFRATECVT s_BaudnRF[] = {
	{1200, UART_BAUDRATE_BAUDRATE_Baud1200},
	{2400, UART_BAUDRATE_BAUDRATE_Baud2400},
	{4800, UART_BAUDRATE_BAUDRATE_Baud4800},
	{9600, UART_BAUDRATE_BAUDRATE_Baud9600},
	{14400, UART_BAUDRATE_BAUDRATE_Baud14400},
	{19200, UART_BAUDRATE_BAUDRATE_Baud19200},
	{28800, UART_BAUDRATE_BAUDRATE_Baud28800},
	{38400, UART_BAUDRATE_BAUDRATE_Baud38400},
	{57600, UART_BAUDRATE_BAUDRATE_Baud57600},
	{76800, UART_BAUDRATE_BAUDRATE_Baud76800},
	{115200, UART_BAUDRATE_BAUDRATE_Baud115200},
	{230400, UART_BAUDRATE_BAUDRATE_Baud230400},
	{250000, UART_BAUDRATE_BAUDRATE_Baud250000},
	{460800, UART_BAUDRATE_BAUDRATE_Baud460800},
	{921600, UART_BAUDRATE_BAUDRATE_Baud921600},
	{1000000, UART_BAUDRATE_BAUDRATE_Baud1M}
};

const int s_NbBaudnRF = sizeof(s_BaudnRF) / sizeof(NRFRATECVT);

NRFUARTDEV s_nRFUartDev = {
	0,
	NRF_UART0,
	NULL,
	true,
};

//static uint8_t s_nRFUARTHoldBuffer[NRF51UART_FIFO_MAX];
//static int s_nRFUARTHoldBuffLen = 0;
static int s_nRF51RxTimeOutCnt = 0;
//uint32_t g_nRF51RxDropCnt = 0;

#define NRFUART_CFIFO_SIZE		(16 + sizeof(CFIFOHDL))

static uint8_t s_nRFUARTRxFifoMem[NRFUART_CFIFO_SIZE];
static uint8_t s_nRFUARTTxFifoMem[NRFUART_CFIFO_SIZE];

bool nRFUARTWaitForRxReady(NRFUARTDEV *pDev, uint32_t Timeout)
{
	do {
		if (pDev->pReg->EVENTS_RXDRDY || pDev->bRxReady)
		{
//			pDev->pReg->EVENTS_RXDRDY = 0;
//			pDev->pReg->EVENTS_RXTO = 0;
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

bool nRFUARTWaitForTxReady(NRFUARTDEV *pDev, uint32_t Timeout)
{
	do {
		if (pDev->pReg->EVENTS_TXDRDY || pDev->bTxReady == true)
		{
			//pDev->pReg->EVENTS_TXDRDY = 0;
			//pDev->bTxReady = true;
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

void UART0_IRQHandler()
{
	uint8_t buff[NRFUART_CFIFO_SIZE];
	int len = 0;
	int cnt;

	if (s_nRFUartDev.pReg->EVENTS_RXDRDY || s_nRFUartDev.pReg->EVENTS_RXTO)
	{
		s_nRF51RxTimeOutCnt = 0;
		//int l = 0;
		uint8_t *d;

		//while (nRFUARTWaitForRxReady(&s_nRFUartDev, 10)) {
		while (s_nRFUartDev.pReg->EVENTS_RXDRDY || s_nRFUartDev.bRxReady) {
		//do {
			s_nRFUartDev.pReg->EVENTS_RXDRDY = 0;
			d = CFifoPut(s_nRFUartDev.pUartDev->hRxFifo);
			if (d == NULL)
			{
				s_nRFUartDev.bRxReady = true;
				break;
			}
			s_nRFUartDev.bRxReady = false;
			*d = s_nRFUartDev.pReg->RXD;
			cnt++;
		}// while (nRFUARTWaitForRxReady(&s_nRFUartDev, 10));
		// while (s_nRFUartDev.pReg->EVENTS_RXDRDY);// && cnt < NRF51UART_FIFO_MAX) ;

		if (s_nRFUartDev.pUartDev->EvtCallback)
		{
			len = CFifoUsed(s_nRFUartDev.pUartDev->hRxFifo);
			if (s_nRFUartDev.pReg->EVENTS_RXTO)
			{
				s_nRFUartDev.pReg->EVENTS_RXTO = 0;
				s_nRFUartDev.bRxReady = false;
				cnt = s_nRFUartDev.pUartDev->EvtCallback(s_nRFUartDev.pUartDev, UART_EVT_RXTIMEOUT, buff, len);
			}
			else
				cnt = s_nRFUartDev.pUartDev->EvtCallback(s_nRFUartDev.pUartDev, UART_EVT_RXDATA, buff, len);
		}
	}

	if (s_nRFUartDev.pReg->EVENTS_TXDRDY)
	{

		cnt = 0;

		//while (nRFUARTWaitForTxReady(&s_nRFUartDev, 30) && cnt < NRF51UART_FIFO_MAX) {
		do {
			s_nRFUartDev.pReg->EVENTS_TXDRDY = 0;

			uint8_t *p = CFifoGet(s_nRFUartDev.pUartDev->hTxFifo);
			if (p == NULL)
			{
				s_nRFUartDev.bTxReady = true;
				break;
			}
			s_nRFUartDev.bTxReady = false;
			s_nRFUartDev.pReg->TXD = *p;
			cnt++;
		} //while (s_nRFUartDev.pReg->EVENTS_TXDRDY && cnt < NRF51UART_FIFO_MAX);
		while (nRFUARTWaitForTxReady(&s_nRFUartDev, 100) && cnt < NRF51UART_FIFO_MAX);
		if (s_nRFUartDev.pUartDev->EvtCallback)
		{
			//uint8_t buff[NRFUART_CFIFO_SIZE];

			//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFUartDev.pUartDev->hTxFifo));
			len = CFifoAvail(s_nRFUartDev.pUartDev->hTxFifo);
			len = s_nRFUartDev.pUartDev->EvtCallback(s_nRFUartDev.pUartDev, UART_EVT_TXREADY, buff, len);
			if (len > 0)
			{
				//s_nRFUartDev.bTxReady = false;
				nRFUARTTxData(&s_nRFUartDev.pUartDev->SerIntrf, buff, len);
			}
		}
	}

	if (s_nRFUartDev.pReg->EVENTS_ERROR)
	{
		s_nRFUartDev.pReg->EVENTS_ERROR = 0;
		if (s_nRFUartDev.pReg->ERRORSRC & 1)	// Overrrun
		{
			len = 0;
			cnt = 0;
			//int l = 0;
			uint8_t *d;
			do {
				s_nRFUartDev.pReg->EVENTS_RXDRDY = 0;
				d = CFifoPut(s_nRFUartDev.pUartDev->hRxFifo);
				if (d == NULL)
				{
					s_nRFUartDev.bRxReady = true;
					break;
				}
				s_nRFUartDev.bRxReady = false;
				*d = s_nRFUartDev.pReg->RXD;
				cnt++;
			} //while (nRFUARTWaitForRxReady(&s_nRFUartDev, 10));
			while (s_nRFUartDev.pReg->EVENTS_RXDRDY);

			if (s_nRFUartDev.pUartDev->EvtCallback)
			{
				len = CFifoUsed(s_nRFUartDev.pUartDev->hRxFifo);
				s_nRFUartDev.pUartDev->EvtCallback(s_nRFUartDev.pUartDev, UART_EVT_RXDATA, buff, len);
			}
		}
		s_nRFUartDev.pReg->ERRORSRC = s_nRFUartDev.pReg->ERRORSRC;
		len = 0;
		if (s_nRFUartDev.pUartDev->EvtCallback)
		{
			s_nRFUartDev.pUartDev->EvtCallback(s_nRFUartDev.pUartDev, UART_EVT_LINESTATE, buff, len);
		}
		NRF_UART0->TASKS_STARTRX = 1;
	}

	if (s_nRFUartDev.pReg->EVENTS_CTS)
	{
		s_nRFUartDev.pReg->EVENTS_CTS = 0;
		NRF_UART0->TASKS_STARTTX = 1;
	}

	if (s_nRFUartDev.pReg->EVENTS_NCTS)
	{
		s_nRFUartDev.pReg->EVENTS_NCTS = 0;
		NRF_UART0->TASKS_STOPTX = 1;
	}

}

int nRFUARTSetRate(SERINTRFDEV *pDev, int Rate)
{
	int rate = s_BaudnRF[s_NbBaudnRF].Baud;

	for (int i = 0; i < s_NbBaudnRF; i++)
	{
		if (s_BaudnRF[i].Baud >= Rate)
		{
		    NRF_UART0->BAUDRATE = (s_BaudnRF[i].nRFBaud << UART_BAUDRATE_BAUDRATE_Pos);

		    rate = s_BaudnRF[i].Baud;
		    break;
		}
	}

	return rate;
}

int nRFUARTRxData(SERINTRFDEV *pDev, uint8_t *pBuff, int Bufflen)
{
	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Bufflen)
	{
		int l  = Bufflen;
		uint8_t *p = CFifoGetMultiple(dev->pUartDev->hRxFifo, &l);
		if (p == NULL)
			break;
		memcpy(pBuff, p, l);
		cnt += l;
		pBuff += l;
		Bufflen -= l;
	}
	EnableInterrupt(state);
/*
	if (dev->bRxReady)
	{
		uint8_t *p = CFifoPut(dev->pUartDev->hRxFifo);
		if (p)
		{
			dev->pReg->EVENTS_RXDRDY = 0;
			dev->bRxReady = false;
			*p = dev->pReg->RXD;
		}
	}
*/
	return cnt;
}

int nRFUARTTxData(SERINTRFDEV *pDev, uint8_t *pData, int Datalen)
{
	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->pDevData;
	int cnt = 0;

	uint32_t state = DisableInterrupt();
	while (Datalen > 0)
	{
		int l = Datalen;
		uint8_t *p = CFifoPutMultiple(dev->pUartDev->hTxFifo, &l);
		if (p == NULL)
			break;
		memcpy(p, pData, l);
		Datalen -= l;
		pData += l;
		cnt += l;
	}

	if (dev->bTxReady)
	{
		if (nRFUARTWaitForTxReady(dev, 1000))
		{
			uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
			if (p)
			{
				dev->pReg->EVENTS_TXDRDY = 0;
				dev->bTxReady = false;
				dev->pReg->TXD = *p;
			}
		}
	}
	EnableInterrupt(state);

	return cnt;
}

bool UARTInit(UARTDEV *pDev, const UARTCFG *pCfg)
{
//	NRFUARTDEV *dev = (NRFUARTDEV*)pDev->SerIntrf.pDevData;
	// Config I/O pins

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_nRFUARTRxFifoMem, NRFUART_CFIFO_SIZE, 1);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_nRFUARTTxFifoMem, NRFUART_CFIFO_SIZE, 1);
	}

	NRF_GPIO->OUTSET = (1 << pCfg->PinCfg[UARTPIN_TX_IDX].PinNo);
	IOPinCfg(pCfg->PinCfg, UART_NB_PINS);
//    nrf_gpio_pin_set(pCfg->PinCfg[UARTPIN_TX_IDX].PinNo);
//    nrf_gpio_cfg_output(pCfg->PinCfg[UARTPIN_TX_IDX].PinNo);
//    nrf_gpio_cfg_input(pCfg->PinCfg[UARTPIN_RX_IDX].PinNo, NRF_GPIO_PIN_PULLUP);

	pDev->SerIntrf.pDevData = &s_nRFUartDev;
	s_nRFUartDev.pUartDev = pDev;

	//NRF_UART0->POWER = UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos;

    NRF_UART0->PSELRXD = pCfg->PinCfg[UARTPIN_RX_IDX].PinNo;
	NRF_UART0->PSELTXD = pCfg->PinCfg[UARTPIN_TX_IDX].PinNo;

    // Set baud
    pDev->Rate = nRFUARTSetRate(&pDev->SerIntrf, pCfg->Rate);

    NRF_UART0->CONFIG &= ~(UART_CONFIG_PARITY_Msk << UART_CONFIG_PARITY_Pos);
	if (pCfg->Parity == UART_PARITY_NONE)
	{
		NRF_UART0->CONFIG |= UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos;
	}
	else
	{
		NRF_UART0->CONFIG |= UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos;
	}

    NRF_UART0->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->EVENTS_ERROR = 0;
    NRF_UART0->ERRORSRC = NRF_UART0->ERRORSRC;
    NRF_UART0->EVENTS_CTS = 0;

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
		NRF_UART0->CONFIG |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
		NRF_UART0->PSELCTS = pCfg->PinCfg[UARTPIN_CTS_IDX].PinNo;
		NRF_UART0->PSELRTS = pCfg->PinCfg[UARTPIN_RTS_IDX].PinNo;
		NRF_GPIO->OUTCLR = (1 << pCfg->PinCfg[UARTPIN_CTS_IDX].PinNo);
		NRF_GPIO->OUTCLR = (1 << pCfg->PinCfg[UARTPIN_RTS_IDX].PinNo);
        // Setup the gpiote to handle pin events on cts-pin.
        // For the UART we want to detect both low->high and high->low transitions in order to
        // know when to activate/de-activate the TX/RX in the UART.
        // Configure pin.
//        nrf_drv_gpiote_in_config_t cts_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
//        nrf_drv_gpiote_in_init(p_comm_params->cts_pin_no, &cts_config, gpiote_uart_event_handler);
	}
	else
	{
		NRF_UART0->CONFIG &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
		NRF_UART0->PSELRTS = -1;
		NRF_UART0->PSELCTS = -1;
	}


    s_nRFUartDev.RxPin = pCfg->PinCfg[UARTPIN_RX_IDX].PinNo;
    s_nRFUartDev.TxPin = pCfg->PinCfg[UARTPIN_TX_IDX].PinNo;
	s_nRFUartDev.CtsPin = pCfg->PinCfg[UARTPIN_CTS_IDX].PinNo;
	s_nRFUartDev.RtsPin = pCfg->PinCfg[UARTPIN_RTS_IDX].PinNo;
	s_nRFUartDev.bRxReady = false;
	s_nRFUartDev.bTxReady = true;

	pDev->DataBits = pCfg->DataBits;
	pDev->FlowControl = pCfg->FlowControl;
	pDev->StopBits = pCfg->StopBits;
	pDev->bIrDAFixPulse = pCfg->bIrDAFixPulse;
	pDev->bIrDAInvert = pCfg->bIrDAInvert;
	pDev->bIrDAMode = pCfg->bIrDAMode;
	pDev->IrDAPulseDiv = pCfg->IrDAPulseDiv;
	pDev->Parity = pCfg->Parity;
	pDev->bIntMode = pCfg->bIntMode;
	pDev->EvtCallback = pCfg->EvtCallback;
	pDev->SerIntrf.Disable = nRFUARTDisable;
	pDev->SerIntrf.Enable = nRFUARTEnable;
	pDev->SerIntrf.GetRate = nRFUARTGetRate;
	pDev->SerIntrf.SetRate = nRFUARTSetRate;
	pDev->SerIntrf.StartRx = nRFUARTStartRx;
	pDev->SerIntrf.RxData = nRFUARTRxData;
	pDev->SerIntrf.StopRx = nRFUARTStopRx;
	pDev->SerIntrf.StartTx = nRFUARTStartTx;
	pDev->SerIntrf.TxData = nRFUARTTxData;
	pDev->SerIntrf.StopTx = nRFUARTStopTx;

    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;


/*    usDelay(10000);
    NRF_UART0->EVENTS_RXDRDY = 0;
    NRF_UART0->EVENTS_TXDRDY = 0;
    NRF_UART0->EVENTS_CTS = 0;
    NRF_UART0->EVENTS_NCTS = 0;
    NRF_UART0->EVENTS_RXTO = 0;
    NRF_UART0->EVENTS_ERROR = 0;*/

    NRF_UART0->INTENCLR = 0xffffffffUL;

	if (pCfg->bIntMode)
	{
		NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
						  (UART_INTENSET_RXTO_Set << UART_INTENSET_RXTO_Pos) |
						  (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
						  (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos) |
						  (UART_INTENSET_CTS_Set << UART_INTENSET_CTS_Pos) |
						  (UART_INTENSET_NCTS_Set << UART_INTENSET_NCTS_Pos);

		NVIC_ClearPendingIRQ(UART0_IRQn);
		NVIC_SetPriority(UART0_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(UART0_IRQn);
    }

	//NRF_UART0->TASKS_STARTTX = 1;
    uint8_t d = NRF_UART0->RXD;	// Dummy read
	return true;
}

void nRFUARTDisable(SERINTRFDEV *pDev)
{
	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->pDevData;

	dev->pReg->TASKS_STOPRX = 1;
	dev->pReg->TASKS_STOPTX = 1;

	dev->pReg->PSELRXD = -1;
	dev->pReg->PSELTXD = -1;
	dev->pReg->PSELRTS = -1;
	dev->pReg->PSELCTS = -1;

	dev->pReg->ENABLE  &= ~(UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
}

void nRFUARTEnable(SERINTRFDEV *pDev)
{
	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->pDevData;

	dev->pReg->PSELRXD = dev->RxPin;
	dev->pReg->PSELTXD = dev->TxPin;
	dev->pReg->PSELCTS = dev->CtsPin;
	dev->pReg->PSELRTS = dev->RtsPin;

	dev->pReg->ENABLE  |= (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	dev->pReg->TASKS_STARTRX = 1;
	dev->pReg->TASKS_STARTTX = 1;
}

void UARTSetCtrlLineState(UARTDEV *pDev, uint32_t LineState)
{
	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->SerIntrf.pDevData;

}

