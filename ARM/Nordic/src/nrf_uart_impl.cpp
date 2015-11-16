/*--------------------------------------------------------------------------
File   : nrf_uart_impl.cpp

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
#include "app_uart.h"

#include "istddef.h"
#include "nrf_uart.h"

extern char s_Buffer[];	// defined in sbuffer.c
extern int s_BufferSize;

nRFUart::nRFUart()
{

}

nRFUart::~nRFUart()
{

}

void uart_evt_handler(app_uart_evt_t *evt)
{

}

int nRFUart::Rx(uint8_t *pBuff, uint32_t Len)
{

}

// Stop receive
int nRFUart::Tx(uint8_t *pData, uint32_t Len)
{

}

// Initiate transmit
bool nRFUart::StartTx(int DevAddr)
{

}

// Transmit Data only, no Start/Stop condition
int nRFUart::TxData(uint8_t *pData, int DataLen)
{

}

// Stop transmit
void nRFUart::StopTx(void)
{

}


