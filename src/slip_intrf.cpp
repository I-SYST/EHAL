/**-------------------------------------------------------------------------
@file	slip_intrf.cpp

@brief	Implementation of SLIP device interface

@author	Hoang Nguyen Hoan
@date	Aug. 7, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#include "slip_intrf.h"

#define SLIP_ESC_CODE				0xDB
#define SLIP_ESC_END_CODE			0xDC
#define SLIP_ESC_ESC_CODE			0xDD

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
void SlipIntrfDisable(DEVINTRF * const pDevIntrf)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		dev->pPhyIntrf->Disable(dev->pPhyIntrf);
	}
}

/**
 * @brief	Wake up the interface.
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 */
void SlipIntrfEnable(DEVINTRF * const pDevIntrf)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		dev->pPhyIntrf->Enable(dev->pPhyIntrf);
	}
}

/**
 * @brief	Get data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return	Transfer rate per second
 */
int SlipIntrfGetRate(DEVINTRF * const pDevIntrf)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		return dev->pPhyIntrf->GetRate(dev->pPhyIntrf);
	}

	return 0;
}

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
int SlipIntrfSetRate(DEVINTRF * const pDevIntrf, int Rate)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		return dev->pPhyIntrf->SetRate(dev->pPhyIntrf, Rate);
	}

	return 0;
}

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
bool SlipIntrfStartRx(DEVINTRF * const pDevIntrf, int DevAddr)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		return dev->pPhyIntrf->StartRx(dev->pPhyIntrf, DevAddr);
	}

	return false;
}

/**
 * @brief	Receive and decode data into pBuff passed in parameter.
 *
 * This function is blocking until full packet is received, i.e. SLIP_END_CODE is received
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 * @param	pBuff 	  : Pointer to memory area to receive data.
 * @param	BuffLen   : Length of buffer memory in bytes
 *
 * @return	Number of bytes read
 */
int SlipIntrfRxDataBlocking(DEVINTRF * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;
	int cnt = 0;
	uint8_t d;

	uint8_t *p = pBuff;

	while (BuffLen > 0)
	{
		if (dev->pPhyIntrf->RxData(dev->pPhyIntrf, pBuff, 1) > 0)
		{
			if (*pBuff == SLIP_END_CODE)
			{
				return cnt;
			}
			if (*pBuff == SLIP_ESC_CODE)
			{
				while (dev->pPhyIntrf->RxData(dev->pPhyIntrf, &d, 1) <= 0);
				if (d == SLIP_ESC_END_CODE)
				{
					*pBuff = SLIP_END_CODE;
				}
				else if (d == SLIP_ESC_ESC_CODE)
				{
					*pBuff = SLIP_ESC_CODE;
				}
			}
			pBuff++;
			BuffLen--;
			cnt++;
		}
	}

	return cnt;
}

/**
 * @brief	Receive and decode data into pBuff passed in parameter.
 *
 * This function is non blocking. It will return after one read.  Application must
 * manage received buffer and verify the last byte received for the SLIP_END_CODE.
 * Call again to receive more byte until SLIP_END_CODE is received.
 *
 * NOTE: Make sure buffer is large enough to contain full packet + SLIP_END_CODE byte
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 * @param	pBuff 	  : Pointer to memory area to receive data.
 * @param	BuffLen   : Length of buffer memory in bytes
 *
 * @return	Number of bytes read
 */
int SlipIntrfRxDataNonBlocking(DEVINTRF * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;
	int cnt = 0;
	uint8_t d;

	if (*pBuff == SLIP_ESC_CODE)
	{
		if (dev->pPhyIntrf->RxData(dev->pPhyIntrf, &d, 1) <= 0)
		{
			return 0;
		}
		if (d == SLIP_ESC_END_CODE)
		{
			*pBuff = SLIP_END_CODE;
		}
		else if (d == SLIP_ESC_ESC_CODE)
		{
			*pBuff = SLIP_ESC_CODE;
		}
		pBuff++;
		BuffLen--;
		cnt++;
	}

	while (BuffLen > 0)
	{
		if (dev->pPhyIntrf->RxData(dev->pPhyIntrf, pBuff, 1) <= 0)
		{
			*pBuff = 0xFF;

			break;
		}

		if (*pBuff == SLIP_END_CODE)
		{
			cnt++;
			break;
		}
		if (*pBuff == SLIP_ESC_CODE)
		{
			if (dev->pPhyIntrf->RxData(dev->pPhyIntrf, &d, 1) <= 0)
			{
				break;
			}
			if (d == SLIP_ESC_END_CODE)
			{
				*pBuff = SLIP_END_CODE;
			}
			else if (d == SLIP_ESC_ESC_CODE)
			{
				*pBuff = SLIP_ESC_CODE;
			}
		}
		pBuff++;
		BuffLen--;
		cnt++;
	}

	return cnt;
}

/**
 * @brief	Completion of read data phase. Do require post processing
 * after data has been received via RxData
 * This function must clear the busy state for reentrancy
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 */
void SlipIntrfStopRx(DEVINTRF * const pDevIntrf)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		dev->pPhyIntrf->StopRx(dev->pPhyIntrf);
	}
}

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
bool SlipIntrfStartTx(DEVINTRF * const pDevIntrf, int DevAddr)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		return dev->pPhyIntrf->StartTx(dev->pPhyIntrf, DevAddr);
	}

	return false;
}


/**
 * @brief	Transfer data from pData passed in parameter.  Assuming StartTx was
 * called prior calling this function to send the actual data
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 * @param	pData 	: Pointer to memory area of data to send.
 * @param	DataLen : Length of data memory in bytes
 *
 * @return	Number of bytes sent including the SLIP code
 */
#if 1
int SlipIntrfTxData(DEVINTRF * const pDevIntrf, uint8_t *pData, int DataLen)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;
	int cnt = 0;
	uint8_t d[2] = {SLIP_ESC_CODE, 0};

	if (dev->pPhyIntrf)
	{
		uint8_t *p = pData;
		int i = 0;

		while (DataLen > 0)
		{
			// Find conflicting code
			if (*p == SLIP_END_CODE)
			{
				d[1] = SLIP_ESC_END_CODE;
			}
			else if (*p == SLIP_ESC_CODE)
			{
				d[1] = SLIP_ESC_ESC_CODE;
			}

			if (d[1] != 0)
			{
				// Flush data
				i--;
				while (i > 0)
				{
					int l = dev->pPhyIntrf->TxData(dev->pPhyIntrf, pData, i);
					i -= l;
					pData += l;
					DataLen -= l;
					cnt += l;
				}
				pData++;

				// Send escape code
				i = 2;
				uint8_t *p1 = d;
				while (i > 0)
				{
					int l = dev->pPhyIntrf->TxData(dev->pPhyIntrf, p1, i);
					p1 += l;
					i -= l;
					cnt += l;
				}

				i = 0;
				d[1] = 0;
			}
			p++;
			i++;

			if (i >= DataLen)
			{
				// End of packet, send end code
				while (i > 0)
				{
					int l = dev->pPhyIntrf->TxData(dev->pPhyIntrf, pData, i);
					i -= l;
					pData += l;
					DataLen -= l;
					cnt += l;
				}
				d[0] = SLIP_END_CODE;

				while (dev->pPhyIntrf->TxData(dev->pPhyIntrf, d, 1) < 1);
				cnt++;
			}
		}
	}

	return cnt;
}
#else

int SlipIntrfTxData(DEVINTRF * const pDevIntrf, uint8_t *pData, int DataLen)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;
	int cnt = 0;
	uint8_t d[2] = {SLIP_ESC_CODE, 0};

	if (dev->pPhyIntrf)
	{
		uint8_t *p = pData;
		int i = 0;

		while (DataLen > 0)
		{
			// Find conflicting code
			if (*p == SLIP_END_CODE)
			{
				d[1] = SLIP_ESC_END_CODE;
				uint8_t *p1 = d;
				i = 2;
				while (i > 0)
				{
					int l = dev->pPhyIntrf->TxData(dev->pPhyIntrf, p1, i);
					p1 += l;
					i -= l;
					cnt += l;
				}
			}
			else if (*p == SLIP_ESC_CODE)
			{
				d[1] = SLIP_ESC_ESC_CODE;
				uint8_t *p1 = d;
				i = 2;
				while (i > 0)
				{
					int l = dev->pPhyIntrf->TxData(dev->pPhyIntrf, p1, i);
					p1 += l;
					i -= l;
					cnt += l;
				}
			}
			else
			{
				while (dev->pPhyIntrf->TxData(dev->pPhyIntrf, p, 1) < 1);
			}

			DataLen--;
			p++;
			cnt++;
		}
		d[0] = SLIP_END_CODE;

		while (dev->pPhyIntrf->TxData(dev->pPhyIntrf, d, 1) < 1);
	}

	return cnt;
}
#endif

/**
 * @brief	Completion of sending data via TxData.  Do require post processing
 * after all data was transmitted via TxData.
 * This function must clear the busy state for re-entrancy
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 */
void SlipIntrfStopTx(DEVINTRF * const pDevIntrf)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		dev->pPhyIntrf->StopTx(dev->pPhyIntrf);
	}
}

/**
 * @brief	This function perform a reset of interface.  Must provide empty
 * function of not used.
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 */
void SlipIntrfReset(DEVINTRF * const pDevIntrf)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		dev->pPhyIntrf->Reset(dev->pPhyIntrf);
	}
}

/**
 * @brief	Power off device for power saving.
 *
 * This function will power off device completely. Not all device provide this
 * type of functionality.  Once power off is call, full initialization cycle is
 * required.  Therefore their is no PowerOn counter part of this function contrary
 * to the Enable/Disable functions.
 *
 * @param	pDevIntrf : Pointer to an instance of the Device Interface
 */
void SlipIntrfPowerOff(DEVINTRF * const pDevIntrf)
{
	SLIPDEV *dev = (SLIPDEV *)pDevIntrf->pDevData;

	if (dev->pPhyIntrf)
	{
		dev->pPhyIntrf->PowerOff(dev->pPhyIntrf);
	}
}



bool SlipInit(SLIPDEV * const pDev, DEVINTRF * const pPhyIntrf, bool bBlocking)
{
	if (pDev == nullptr || pPhyIntrf == nullptr)
	{
		return false;
	}

	pDev->DevIntrf.pDevData = pDev;
	pDev->pPhyIntrf = pPhyIntrf;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.Disable = SlipIntrfDisable;
	pDev->DevIntrf.Enable = SlipIntrfEnable;
	pDev->DevIntrf.GetRate = SlipIntrfGetRate;
	pDev->DevIntrf.SetRate = SlipIntrfSetRate;
	pDev->DevIntrf.StartRx = SlipIntrfStartRx;

	if (bBlocking == true)
	{
		pDev->DevIntrf.RxData = SlipIntrfRxDataBlocking;
	}
	else
	{
		pDev->DevIntrf.RxData = SlipIntrfRxDataNonBlocking;
	}

	pDev->DevIntrf.StopRx = SlipIntrfStopRx;
	pDev->DevIntrf.StartTx = SlipIntrfStartTx;
	pDev->DevIntrf.TxData = SlipIntrfTxData;
	pDev->DevIntrf.StopTx = SlipIntrfStopTx;
	pDev->DevIntrf.IntPrio = 0;
	pDev->DevIntrf.EvtCB = nullptr;
	pDev->DevIntrf.MaxRetry = 5;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.PowerOff = SlipIntrfPowerOff;
	pDev->DevIntrf.EnCnt = 1;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

bool Slip::Init(DeviceIntrf * const pIntrf, bool bBlocking)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	memset(&vDevData, 0, sizeof(SLIPDEV));

	return SlipInit(&vDevData, *pIntrf, bBlocking);
}

