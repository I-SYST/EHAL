/*--------------------------------------------------------------------------
File   : sdcard.c

Author : Hoang Nguyen Hoan          June 9, 2011

Desc   : SD card driver

Copyright (c) 2011, I-SYST, all rights reserved

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

#include <string.h>

#include "istddef.h"
#include "crc.h"
#include "sdcard.h"

static inline int SDRx(SDDEV *pDev, uint8_t *pData, int DataLen)
{
	return SerialIntrfRx(pDev->pSerIntrf, 0, pData, DataLen);
}

static inline int SDTx(SDDEV *pDev, uint8_t *pData, int DataLen)
{
	return SerialIntrfTx(pDev->pSerIntrf, 0, pData, DataLen);
}

bool SDInit(SDDEV *pDev, SDCFG *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
		return false;

	if (pCfg->pSerIntrf == NULL)
		return false;

	uint8_t data[80];
	uint8_t r = 0xff;

	pDev->pSerIntrf = pCfg->pSerIntrf;

	// Send 80 x 0xff to reset card to native mode
	memset(data, 0xff, 80);
	SDRx(pDev, data, 80);
/*	LpcSSPStartTx(pDev->pCtrlInterf, 0);

	for (int i = 0; i < 80; i++)
	{
		LpcSSPTxData(pDev->pSspDev, &r, 1);
	}
	LpcSSPStopTx(pDev->pSspDev);*/


	// Activate SPI mode
	r = SDCmd(pDev, 0, 0);
	if (r & 0xfe)
	{
		// Card not usable or not found
		return false;
	}

	// CMD8 must be sent before activating with ACMD41
	uint32_t acmd = 0x10000000;
	r = SDCmd(pDev, 8, 0x1AA);
	if ((r & 0xfe) == 0)
	{
		r = SDGetResponse(pDev, data, 5);
		if (r > 0 && data[2] == 1 && data[3] == 0xAA)
		{
			acmd = 0x50000000;
		}
	}

	// Start card initialisation
	// send ACMD41
	int i = 10;
	do {
		r = SDCmd(pDev, 55, 0x0000000);
		if ((r & 0xfe) == 0)
		{
			r = SDCmd(pDev, 41, acmd);
			SDGetResponse(pDev, data, 5);
		}
	} while (--i > 0 && r != 0);

	if (r != 0)
	{
		// ACMD41 not succeeded
		// use CMD1
		i = 0x1ffff;
		do {
			r = SDCmd(pDev, 1, 0);
		} while (r != 0 && --i > 0);
	}

	if (r == 0)
	{
		// Read OCR
		r = SDCmd(pDev, 58, 0x0000000);
		i = SDGetResponse(pDev, data, 5);
//		printf("CMD8 r = %02x, data = 0x%08x, len = %d\n\r", r, *(uint32_t*)data, i);
	}

	return true;
}
/*
uint8_t SDCmdCrc(uint8_t *pData)
{
	uint8_t i, e, f, crc;

	crc = 0;
	for (i = 0; i < 5; i++)
	{
		e = crc ^ pData[i];
		f = e ^ (e >> 4) ^ (e >> 7);
		crc = (f << 1) ^ (f << 4);
	}
	return crc;
}

uint16_t crc16(uint8_t *pData, int len)
{
	uint8_t  s, t;
	uint16_t crc;
	int i;

	crc = 0;
	for (i = 0; i < len; i++)
	{
		s = pData[i] ^ (crc >> 8);
		t = s ^ (s >> 4);
		crc = (crc << 8) ^ t ^ (t << 5) ^ (t << 12);
	}

	return crc;
}
*/
/*
	Send command to SD card

	@param	SDDEV *pDev 	: Reference to device data
	@param 	uint8_t Cmd 	: Command id number
	@param	uint32_t param 	: Cmd parameter value

	@return int responses code R1
*/
int SDCmd(SDDEV *pDev, uint8_t Cmd, uint32_t param)
{
	int t;
	uint8_t data[7];
	uint8_t r;

	memset(data, 0, sizeof(data));

	// Fill cmd buffer
	data[0] = Cmd | 0x40;
	data[1] = (param >> 24) & 0xff;
	data[2] = (param >> 16) & 0xff;
	data[3] = (param >> 8) & 0xff;
	data[4] = param & 0xff;
	data[5] = crc8(data, 5) | 1; // SDCmdCrc(data) | 1;
	data[6] = 0xff;

	// Send command
	//LpcSSPTx(pDev->pSspDev, 0, data, 6);
	SDTx(pDev, data, 6);

	// wait for response
	t = 100000;
	do {
		SDRx(pDev, &r, 1);
	} while (r ==0xff && --t > 0);

	//printf("Cmd r = %x\n\r", r);

	return r;
}

/*
	Read extra response data

	@param	SDDEV *pDev : Reference to device data
	@param	uint8_t *pData	: Reference to data buffer
	@paran 	int Len			: Data buffer length

	@return	total length (bytes) if succeeded
*/
int SDGetResponse(SDDEV *pDev, uint8_t *pData, int Len)
{
	if (pData == NULL)
		return 0;

	return SDRx(pDev, pData, Len);
}

int SDReadData(SDDEV *pDev, uint8_t *pData, int Len)
{
	int timeout, cnt, crc, calccrc;
	uint8_t d;

	if (pData == NULL || Len <= 0)
		return 0;

	timeout = 100000;

	d = 0xff;

	do {
		cnt = SDRx(pDev, &d, 1);
	} while (d != 0xfe && --timeout > 0 );

	if (timeout <= 0)
		return -1;

	if (d != 0xfe)
	{
		pData[0] = d;
		return -1;
	}

	cnt = 0;
	d = 0xff;

	cnt = SDRx(pDev, pData, Len);

	calccrc = crc16(pData, cnt);
	//printf("Calc CRC : %04x\n\r", calccrc);

	if (cnt <= Len)
	{
		crc = 0;
		SDRx(pDev, (uint8_t*)&crc, 1);
		crc <<= 8;
		SDRx(pDev, &d, 1);
		crc |= d;
	}
	if (crc != calccrc)
		return -2;

	return cnt;
}

int SDWriteData(SDDEV *pDev, uint8_t *pData, int Len)
{
	int i;
	uint16_t crc;
	uint8_t d[2] = { 0xff, 0xfe };

	if (pData == NULL)
		return -1;

	crc = crc16(pData, Len);


	SDTx(pDev, d, 2);

	i = SDTx(pDev, pData, Len);

	d[0] = crc >> 8;
	d[1] = crc & 0xff;

	SDTx(pDev, d, 2);

	int t = 1000000;
	do
	{
		SDRx(pDev, d, 1);
		if (d[0] != 0xff && d[0] != 0)
			break;
	} while (--t > 0);

	if ((d[0] & 0xe) == 0x4)
	{
		t = 100000;

		do {
			SDRx(pDev, d, 1);
			if (d[0] != 0)
				break;
		} while (t-- > 0);

		return i;
	}

	return 0;
}

uint32_t SDGetSize(SDDEV *pDev)
{
	uint8_t data[20];
	uint32_t c_size, c_size_mult, read_bl_len;

	int r = SDCmd(pDev, 9, 0);
	if (r == 0)
		SDReadData(pDev, data, 16);

	c_size = (uint16_t)data[7]<<2;
	c_size |= (data[8] >> 6) & 0x03; // bit 62-63
	c_size |= (data[6] & 3) << 10;
	c_size++;

	c_size_mult = 4 << ((data[10] >> 7) | ((data[9] & 3) << 1));

	read_bl_len = 1 << (data[5] & 0x0F);

//	printf("c_size : %d, c_mult : %d, read_bl_len = %d\n\r", c_size, c_size_mult, read_bl_len);

	return c_size * c_size_mult * read_bl_len;
}


int SDReadSingleBlock(SDDEV *pDev, uint32_t Addr, uint8_t *pData, int len)
{
	if (pData)
	{
		int r = SDCmd(pDev, 17, Addr);
		if (r == 0)
			return SDReadData(pDev, pData, len);
	}
	return 0;
}

int SDWriteSingleBlock(SDDEV *pDev, uint32_t Addr, uint8_t *pData, int Len)
{
	if (pData)
	{

		int r = SDCmd(pDev, 24, Addr);
		if (r == 0)
			return SDWriteData(pDev, pData, Len);
	}

	return 0;
}

