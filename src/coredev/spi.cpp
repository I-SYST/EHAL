/**-------------------------------------------------------------------------
@file	spi.cpp

@brief	Generic SPI (Serial Peripheral Interface) driver definitions.

Current implementation
 	 - Master mode
 	 - Slave mode
 	 - Polling
 	 - Interrupt

@author	Hoang Nguyen Hoan
@date	July 26, 2018

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
#include "coredev/spi.h"

/**
 * @brief	Set SPI slave data for read command.
 *
 * This function sets internal pointer to the location of data to be returned to I2C master upon
 * receiving read command.
 *
 * @param	pDev	: Pointer SPI driver data initialized be SPIInit function
 * @param	SlaveIdx: Slave address index to assign the buffer
 * @param	pBuff	: Pointer to buffer to receive data from master
 * @param	BuffLen	: Total buffer length in bytes
 *
 * @return	None
 */
void SPISetSlaveRxBuffer(SPIDEV * const pDev, int SlaveIdx, uint8_t * const pBuff, int BuffLen) {
	if (SlaveIdx < 0 || SlaveIdx >= SPI_SLAVEMODE_MAX_DEV || pDev == NULL)
		return;

	pDev->pRxBuff[SlaveIdx] = pBuff;
	pDev->RxBuffLen[SlaveIdx] = BuffLen;
}

/**
 * @brief	Set I2C slave buff for write command.
 *
 * This function sets internal pointer to the location of buffer to data from I2C master upon
 * receiving write command.
 *
 * @param	pDev	: Pointer I2C driver data initialized be I2CInit function
 * @param	SlaveIdx: Slave address index to assign the data buffer
 * @param	pData	: Pointer to data buffer to send to master
 * @param	DataLen	: Total data length in bytes
 *
 * @return	None
 */
void SPISetSlaveTxData(SPIDEV * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen)
{
	if (SlaveIdx < 0 || SlaveIdx >= SPI_SLAVEMODE_MAX_DEV || pDev == NULL)
		return;

	pDev->pTxData[SlaveIdx] = pData;
	pDev->TxDataLen[SlaveIdx] = DataLen;
}



