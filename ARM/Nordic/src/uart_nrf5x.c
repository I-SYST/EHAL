/*--------------------------------------------------------------------------
File   : uart_nrf5x.c

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

#include "nrf.h"
/*
#ifdef NRF51
#include "nrf51.h"
#include "nrf51_bitfields.h"
#else
#include "nrf52.h"
#include "nrf52_bitfields.h"
#define NRF_GPIO			NRF_P0
#define UART0_IRQn			UARTE0_UART0_IRQn
#define UART0_IRQHandler	UARTE0_UART0_IRQHandler
#endif
*/
#include "istddef.h"
#include "uart_nrf5x.h"
#include "idelay.h"
#include "atomic.h"

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

static const int s_NbBaudnRF = sizeof(s_BaudnRF) / sizeof(NRFRATECVT);

NRFUARTDEV s_nRFUartDev[] = {
	{
		0,
		NRF_UART0,
		NULL,
		false,
		true,
	},
#ifdef NRF52840_XXAA
	{
		0,
		NRF_UART1,
		NULL,
		false,
		true,
	},
#endif
};

static const int s_NbUartDev = sizeof(s_nRFUartDev) / sizeof(NRFUARTDEV);

static int s_nRF51RxTimeOutCnt = 0;
uint32_t g_nRF51RxDropCnt = 0;
uint32_t g_nRF51RxErrCnt = 0;

#define NRFUART_CFIFO_SIZE		CFIFO_MEMSIZE(16)

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

static void UART_IRQHandler(NRFUARTDEV *pDev)
{
	uint8_t buff[NRFUART_CFIFO_SIZE];
	int len = 0;
	int cnt = 0;

	if (pDev->pReg->EVENTS_RXDRDY || pDev->pReg->EVENTS_RXTO)
	{
		s_nRF51RxTimeOutCnt = 0;
		//int l = 0;
		uint8_t *d;

		cnt = 0;
		//while (nRFUARTWaitForRxReady(&s_nRFUartDev, 10) && cnt < NRF51UART_FIFO_MAX) {
		//while (s_nRFUartDev.pReg->EVENTS_RXDRDY && cnt < NRF51UART_FIFO_MAX) {
		do {
			pDev->bRxReady = false;
			pDev->pReg->EVENTS_RXDRDY = 0;
			d = CFifoPut(pDev->pUartDev->hRxFifo);
			if (d == NULL)
			{
				pDev->bRxReady = true;
				g_nRF51RxDropCnt++;
				break;
			}
			*d = pDev->pReg->RXD;
			cnt++;
		}// while (nRFUARTWaitForRxReady(&s_nRFUartDev, 10));
		 while (pDev->pReg->EVENTS_RXDRDY && cnt < NRF51UART_FIFO_MAX) ;

		if (pDev->pUartDev->EvtCallback)
		{
			len = CFifoUsed(pDev->pUartDev->hRxFifo);
			if (pDev->pReg->EVENTS_RXTO)
			{
				pDev->pReg->EVENTS_RXTO = 0;
				pDev->bRxReady = false;
				cnt = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXTIMEOUT, buff, len);
			}
			else
				cnt = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, buff, len);
		}
	}

	if (pDev->pReg->EVENTS_TXDRDY)
	{

		cnt = 0;

		do {
			pDev->pReg->EVENTS_TXDRDY = 0;

			uint8_t *p = CFifoGet(pDev->pUartDev->hTxFifo);
			if (p == NULL)
			{
				pDev->bTxReady = true;
				break;
			}
			pDev->bTxReady = false;
			pDev->pReg->TXD = *p;
			cnt++;
		} while (pDev->pReg->EVENTS_TXDRDY && cnt < NRF51UART_FIFO_MAX);

		if (pDev->pUartDev->EvtCallback)
		{
			//uint8_t buff[NRFUART_CFIFO_SIZE];

			//len = min(NRFUART_CFIFO_SIZE, CFifoAvail(s_nRFUartDev.pUartDev->hTxFifo));
			len = CFifoAvail(pDev->pUartDev->hTxFifo);
			len = pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_TXREADY, buff, len);
			if (len > 0)
			{
				//s_nRFUartDev.bTxReady = false;
				//nRFUARTTxData(&s_nRFUartDev.pUartDev->SerIntrf, buff, len);
			}
		}
	}

	if (pDev->pReg->EVENTS_ERROR)
	{
		pDev->pReg->EVENTS_ERROR = 0;
		if (pDev->pReg->ERRORSRC & 1)	// Overrrun
		{
			g_nRF51RxErrCnt++;
			len = 0;
			cnt = 0;
			//int l = 0;
			uint8_t *d;
			do {
				pDev->pReg->EVENTS_RXDRDY = 0;
				d = CFifoPut(pDev->pUartDev->hRxFifo);
				if (d == NULL)
				{
					pDev->bRxReady = true;
					break;
				}
				pDev->bRxReady = false;
				*d = pDev->pReg->RXD;
				cnt++;
			} //while (nRFUARTWaitForRxReady(&s_nRFUartDev, 10));
			while (pDev->pReg->EVENTS_RXDRDY && cnt < NRF51UART_FIFO_MAX);

			if (pDev->pUartDev->EvtCallback)
			{
				len = CFifoUsed(pDev->pUartDev->hRxFifo);
				pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_RXDATA, buff, len);
			}
		}
		pDev->pReg->ERRORSRC = pDev->pReg->ERRORSRC;
		len = 0;
		if (pDev->pUartDev->EvtCallback)
		{
			pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_LINESTATE, buff, len);
		}
		pDev->pReg->TASKS_STARTRX = 1;
	}

	if (pDev->pReg->EVENTS_CTS)
	{
		pDev->pReg->EVENTS_CTS = 0;
		pDev->pUartDev->LineState &= ~UART_LINESTATE_CTS;
        if (pDev->pUartDev->EvtCallback)
        {
            buff[0] = 0;//UART_LINESTATE_CTS;
    //        buff[1] = 0;//UART_LINESTATE_CTS;
            len = 1;
            pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_LINESTATE, buff, len);
        }
		//NRF_UART0->TASKS_STARTTX = 1;
		//s_nRFUartDev.bTxReady = true;
	}

	if (pDev->pReg->EVENTS_NCTS)
	{
		pDev->pReg->EVENTS_NCTS = 0;
		pDev->pUartDev->LineState |= UART_LINESTATE_CTS;
		//NRF_UART0->TASKS_STOPTX = 1;
        if (pDev->pUartDev->EvtCallback)
        {
            buff[0] = UART_LINESTATE_CTS;
//            buff[1] = UART_LINESTATE_CTS;
            len = 1;
            pDev->pUartDev->EvtCallback(pDev->pUartDev, UART_EVT_LINESTATE, buff, len);
        }
	}
}

void UART0_IRQHandler()
{
	UART_IRQHandler(&s_nRFUartDev[0]);
}

#ifdef NRF52840_XXAA
void UART1_IRQHandler()
{
	UART_IRQHandler(&s_nRFUartDev[1]);
}
#endif

int nRFUARTSetRate(DEVINTRF *pDev, int Rate)
{
	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->pDevData;

	int rate = s_BaudnRF[s_NbBaudnRF].Baud;

	for (int i = 0; i < s_NbBaudnRF; i++)
	{
		if (s_BaudnRF[i].Baud >= Rate)
		{
		    dev->pReg->BAUDRATE = (s_BaudnRF[i].nRFBaud << UART_BAUDRATE_BAUDRATE_Pos);

		    rate = s_BaudnRF[i].Baud;
		    break;
		}
	}

	return rate;
}

int nRFUARTRxData(DEVINTRF *pDev, uint8_t *pBuff, int Bufflen)
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

	return cnt;
}

int nRFUARTTxData(DEVINTRF *pDev, uint8_t *pData, int Datalen)
{
    NRFUARTDEV *dev = (NRFUARTDEV *)pDev->pDevData;
    int cnt = 0;
    int rtry = pDev->MaxRetry;

    while (Datalen > 0 && rtry-- > 0)
    {
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
        EnableInterrupt(state);

        if (dev->bTxReady)
        {
            //if (nRFUARTWaitForTxReady(dev, 1000))
            {
                dev->pReg->EVENTS_TXDRDY = 0;
                dev->bTxReady = true;
                uint8_t *p = CFifoGet(dev->pUartDev->hTxFifo);
                if (p)
                {
                    dev->bTxReady = false;
                    dev->pReg->TXD = *p;
                }
            }
        }
    }
    return cnt;
}

bool UARTInit(UARTDEV *pDev, const UARTCFG *pCfg)
{
//	NRFUARTDEV *dev = (NRFUARTDEV*)pDev->SerIntrf.pDevData;
	// Config I/O pins
	if (pDev == NULL || pCfg == NULL)
		return false;

	if (pCfg->pIoMap == NULL || pCfg->IoMapLen <= 0)
		return false;

	if (pCfg->DevNo < 0 || pCfg->DevNo >= s_NbUartDev)
	{
		return false;
	}

	int devno = pCfg->DevNo;

	if (pCfg->pRxMem && pCfg->RxMemSize > 0)
	{
		pDev->hRxFifo = CFifoInit(pCfg->pRxMem, pCfg->RxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hRxFifo = CFifoInit(s_nRFUARTRxFifoMem, NRFUART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	if (pCfg->pTxMem && pCfg->TxMemSize > 0)
	{
		pDev->hTxFifo = CFifoInit(pCfg->pTxMem, pCfg->TxMemSize, 1, pCfg->bFifoBlocking);
	}
	else
	{
		pDev->hTxFifo = CFifoInit(s_nRFUARTTxFifoMem, NRFUART_CFIFO_SIZE, 1, pCfg->bFifoBlocking);
	}

	IOPINCFG *pincfg = (IOPINCFG*)pCfg->pIoMap;

	//NRF_GPIO->OUTSET = (1 << pincfg[UARTPIN_TX_IDX].PinNo);
	IOPinCfg(pincfg, pCfg->IoMapLen);
//    nrf_gpio_pin_set(pCfg->PinCfg[UARTPIN_TX_IDX].PinNo);
//    nrf_gpio_cfg_output(pCfg->PinCfg[UARTPIN_TX_IDX].PinNo);
//    nrf_gpio_cfg_input(pCfg->PinCfg[UARTPIN_RX_IDX].PinNo, NRF_GPIO_PIN_PULLUP);

	pDev->DevIntrf.pDevData = &s_nRFUartDev[devno];
	s_nRFUartDev[devno].pUartDev = pDev;

	//NRF_UART0->POWER = UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos;

	s_nRFUartDev[devno].pReg->PSELRXD = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFUartDev[devno].pReg->PSELTXD = pincfg[UARTPIN_TX_IDX].PinNo;

    // Set baud
    pDev->Rate = nRFUARTSetRate(&pDev->DevIntrf, pCfg->Rate);

    s_nRFUartDev[devno].pReg->CONFIG &= ~(UART_CONFIG_PARITY_Msk << UART_CONFIG_PARITY_Pos);
	if (pCfg->Parity == UART_PARITY_NONE)
	{
		s_nRFUartDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos;
	}
	else
	{
		s_nRFUartDev[devno].pReg->CONFIG |= UART_CONFIG_PARITY_Included << UART_CONFIG_PARITY_Pos;
	}

	s_nRFUartDev[devno].pReg->ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	s_nRFUartDev[devno].pReg->EVENTS_RXDRDY = 0;
	s_nRFUartDev[devno].pReg->EVENTS_TXDRDY = 0;
	s_nRFUartDev[devno].pReg->EVENTS_ERROR = 0;
	s_nRFUartDev[devno].pReg->ERRORSRC = NRF_UART0->ERRORSRC;
	s_nRFUartDev[devno].pReg->EVENTS_CTS = 0;

    if (pCfg->FlowControl == UART_FLWCTRL_HW)
	{
    	s_nRFUartDev[devno].pReg->CONFIG |= (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
    	s_nRFUartDev[devno].pReg->PSELCTS = pincfg[UARTPIN_CTS_IDX].PinNo;
    	s_nRFUartDev[devno].pReg->PSELRTS = pincfg[UARTPIN_RTS_IDX].PinNo;
		NRF_GPIO->OUTCLR = (1 << pincfg[UARTPIN_CTS_IDX].PinNo);
		NRF_GPIO->OUTCLR = (1 << pincfg[UARTPIN_RTS_IDX].PinNo);
        // Setup the gpiote to handle pin events on cts-pin.
        // For the UART we want to detect both low->high and high->low transitions in order to
        // know when to activate/de-activate the TX/RX in the UART.
        // Configure pin.
//        nrf_drv_gpiote_in_config_t cts_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
//        nrf_drv_gpiote_in_init(p_comm_params->cts_pin_no, &cts_config, gpiote_uart_event_handler);
	}
	else
	{
		s_nRFUartDev[devno].pReg->CONFIG &= ~(UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
		s_nRFUartDev[devno].pReg->PSELRTS = -1;
		s_nRFUartDev[devno].pReg->PSELCTS = -1;
	}


	s_nRFUartDev[devno].RxPin = pincfg[UARTPIN_RX_IDX].PinNo;
	s_nRFUartDev[devno].TxPin = pincfg[UARTPIN_TX_IDX].PinNo;
	s_nRFUartDev[devno].CtsPin = pincfg[UARTPIN_CTS_IDX].PinNo;
	s_nRFUartDev[devno].RtsPin = pincfg[UARTPIN_RTS_IDX].PinNo;
	s_nRFUartDev[devno].bRxReady = false;
	s_nRFUartDev[devno].bTxReady = true;

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
	pDev->DevIntrf.Disable = nRFUARTDisable;
	pDev->DevIntrf.Enable = nRFUARTEnable;
	pDev->DevIntrf.GetRate = nRFUARTGetRate;
	pDev->DevIntrf.SetRate = nRFUARTSetRate;
	pDev->DevIntrf.StartRx = nRFUARTStartRx;
	pDev->DevIntrf.RxData = nRFUARTRxData;
	pDev->DevIntrf.StopRx = nRFUARTStopRx;
	pDev->DevIntrf.StartTx = nRFUARTStartTx;
	pDev->DevIntrf.TxData = nRFUARTTxData;
	pDev->DevIntrf.StopTx = nRFUARTStopTx;
	pDev->DevIntrf.Busy = false;
	pDev->DevIntrf.MaxRetry = UART_RETRY_MAX;

    s_nRFUartDev[devno].pReg->TASKS_STARTTX = 1;
    s_nRFUartDev[devno].pReg->TASKS_STARTRX = 1;

    s_nRFUartDev[devno].pReg->INTENCLR = 0xffffffffUL;

	if (pCfg->bIntMode)
	{
		s_nRFUartDev[devno].pReg->INTENSET = (UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
						  (UART_INTENSET_RXTO_Set << UART_INTENSET_RXTO_Pos) |
						  (UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
						  (UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos) |
						  (UART_INTENSET_CTS_Set << UART_INTENSET_CTS_Pos) |
						  (UART_INTENSET_NCTS_Set << UART_INTENSET_NCTS_Pos);

		NVIC_ClearPendingIRQ(UART0_IRQn);
		NVIC_SetPriority(UART0_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(UART0_IRQn);
    }

	return true;
}

void nRFUARTDisable(DEVINTRF *pDev)
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

void nRFUARTEnable(DEVINTRF *pDev)
{
	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->pDevData;

	dev->pReg->PSELRXD = dev->RxPin;
	dev->pReg->PSELTXD = dev->TxPin;
	dev->pReg->PSELCTS = dev->CtsPin;
	dev->pReg->PSELRTS = dev->RtsPin;

	CFifoFlush(dev->pUartDev->hTxFifo);

	dev->bTxReady = true;
	dev->pReg->ENABLE  |= (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	dev->pReg->TASKS_STARTRX = 1;
	dev->pReg->TASKS_STARTTX = 1;
}

void UARTSetCtrlLineState(UARTDEV *pDev, uint32_t LineState)
{
//	NRFUARTDEV *dev = (NRFUARTDEV *)pDev->SerIntrf.pDevData;

}

UARTDEV *UARTGetInstance(int DevNo)
{
	return s_nRFUartDev[DevNo].pUartDev;
}

