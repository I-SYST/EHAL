/*--------------------------------------------------------------------------
File   : serialintrf.cpp

Author : Hoang Nguyen Hoan          Dec. 25, 2011

Desc   : Generic serial interface class
		 This class is used to implement serial communication interfaces
		 such as I2C, UART, etc...  Not limited to wired interface

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
#include "serialintrf.h"

int SerialIntrfRx(SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen)
{
	int count = 0;
	int nrtry = pDev->MaxRetry;

	do {
		if (pBuff && pDev->StartRx(pDev, DevAddr)) {
			count = pDev->RxData(pDev, pBuff, BuffLen);
			pDev->StopRx(pDev);
		}
	} while(count <= 0 && nrtry-- > 0);

	return count;
}

int SerialIntrfTx(SERINTRFDEV *pDev, int DevAddr, uint8_t *pBuff, int BuffLen)
{
	int count = 0;
	int nrtry = pDev->MaxRetry;

	do {
		if (pBuff && pDev->StartTx(pDev, DevAddr)) {
			count = pDev->TxData(pDev, pBuff, BuffLen);
			pDev->StopTx(pDev);
		}
	} while (count <= 0 && nrtry-- > 0);

	return count;
}

int SerialIntrfRead(SERINTRFDEV *pDev, int DevAddr, uint8_t *pTxData, int TxLen,
                    uint8_t *pRxBuff, int RxLen)
{
    int count = 0;
    int nrtry = pDev->MaxRetry;

    if (pRxBuff == NULL)
        return 0;

    do {
        if (pDev->StartRx(pDev, DevAddr))
        {
            if (pTxData)
            {
                count = pDev->TxData(pDev, pTxData, TxLen);
            }
            count = pDev->RxData(pDev, pRxBuff, RxLen);
            pDev->StopRx(pDev);
        }
    } while (count <= 0 && nrtry-- > 0);

    return count;
}

/*
// Receive full frame
int SerialIntrf::Rx(int DevAddr, uint8_t *pBuff, int BuffLen)
{
	int count = 0;

	if (pBuff && StartRx(DevAddr))
	{
		count = RxData(pBuff, BuffLen);
		StopRx();
	}

	return count;
}

// Transmit full frame
int SerialIntrf::Tx(int DevAddr, uint8_t *pData, int DataLen)
{
	int count = 0;

	if (pData && StartTx(DevAddr))
	{
		count = TxData(pData, DataLen);
		StopTx();
	}

	return count;
}
*/
