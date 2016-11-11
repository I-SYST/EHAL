/*--------------------------------------------------------------------------
File   : sdcard_impl.cpp

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
#include "iopincfg.h"
#include "sdcard.h"
#include "crc.h"
#include "atomic.h"

SDCard::SDCard()
{

}

SDCard::~SDCard()
{

}

bool SDCard::Init(SerialIntrf *pSerInterf, DISKIO_CACHE_DESC *pCacheBlk, int NbCacheBlk)
{
	uint8_t data[4];
	uint16_t r = 0xffff;
	//vpInterf = std::shared_ptr<SerialIntrf>(pSerInterf);
	vpInterf = pSerInterf;

	// Reset SD Card to SPI mode
	// Need to send reset sequence at a lower rate
	uint32_t speed = vpInterf->Rate();
	vpInterf->Rate(1000000);

	// Send at least 80 x 0xff to reset card to native mode
	for (int i = 0; i < 80; i++)
	{
		vpInterf->Tx(0, (uint8_t*)&r, 1);
	}

	vpInterf->Rate(speed);

	// Activate SPI mode
	r = Cmd(0, 0);
	if (r & 0xfe)
	{
		// Card not usable or not found
		return false;
	}

	// CMD8 must be sent before activating with ACMD41
	uint32_t acmd = 0x10000000;
	r = Cmd(8, 0x1AA);
	if ((r & 0xfe) == 0)
	{
		r = GetResponse(data, 5);
		if (r > 0 && data[2] == 1 && data[3] == 0xAA)
		{
			acmd = 0x50000000;
		}
	}

	// Start card initialisation
	// send ACMD41
	int i = 10;
	do {
		r = Cmd(55, 0x0000000);
		if ((r & 0xfe) == 0)
		{
			r = Cmd(41, acmd);
			GetResponse(data, 5);
		}
	} while (--i > 0 && r != 0);

	if (r != 0)
	{
		// ACMD41 not succeeded
		// use CMD1
		i = 0x1ffff;
		do {
			r = Cmd(1, 0);
		} while (r != 0 && --i > 0);
	}

	if (r == 0)
	{
		// Read OCR
		r = Cmd(58, 0x0000000);
		i = GetResponse(data, 5);
	}

	vDev.SectSize = 512;		// Default always
	vDev.TotalSect = GetSize() * 1024LL / vDev.SectSize;

	if (pCacheBlk && NbCacheBlk > 0)
	{
		SetCache(pCacheBlk, NbCacheBlk);
	}
	return true;
}

int SDCard::Cmd(uint8_t Cmd, uint32_t param)
{
	int t;
	uint8_t data[8];
	uint8_t r;

	// wait for busy
	t = 1000000;
	do {
		vpInterf->Rx(0, &r, 1);
	} while (r == 0 && --t > 0);

	memset(data, 0, sizeof(data));

	// Fill cmd buffer
	data[0] = Cmd | 0x40;
	data[1] = (param >> 24) & 0xff;
	data[2] = (param >> 16) & 0xff;
	data[3] = (param >> 8) & 0xff;
	data[4] = param & 0xff;
	data[5] = crc8(data, 5) | 1; //SDCmdCrc(data) | 1;
	data[6] = 0xff;

	// Send command
	vpInterf->Tx(0, data, 6);

	// wait for response
	t = 100000;
	do {
		vpInterf->Rx(0, &r, 1);
	} while (r ==0xff && --t > 0);

	return r;
}

int SDCard::GetResponse(uint8_t *pBuff, int BuffLen)
{
	int cnt = 0;

	cnt = vpInterf->Rx(0, pBuff, BuffLen);

	return cnt;
}

int SDCard::ReadData(uint8_t *pBuff, int BuffLen)
{
	int timeout, cnt, crc, calccrc;
	uint8_t d;

	if (pBuff == NULL)
		return 0;

	timeout = 100000;

	do {
		vpInterf->Rx(0, &d, 1);
	} while (d != 0xfe && --timeout > 0 );

	if (timeout <= 0)
		return -1;

	if (d != 0xfe)
	{
		pBuff[0] = d;
		return -1;
	}

	cnt = 0;

	d = 0xff;
	cnt = vpInterf->Rx(0, pBuff, BuffLen);

	calccrc = crc16(pBuff, cnt);

	if (cnt <= BuffLen)
	{
		crc = 0;
		vpInterf->Rx(0, (uint8_t*)&crc, 2);
		d = (crc >> 8) & 0xff;
		crc <<= 8;
		//vpInterf->Rx(0, &d, 1);

		crc |= d;
		crc &= 0xffff;
	}
	if (crc != calccrc)
		return -2;

	return cnt;
}

int SDCard::WriteData(uint8_t *pData, int Len)
{
	int cnt;
	uint16_t crc;
	uint8_t d[2] = { 0xff, 0xfe };

	if (pData == NULL)
		return -1;

	crc = crc16(pData, Len);

	vpInterf->Tx(0, d, 2);

	cnt = vpInterf->Tx(0, pData, Len);

	d[0] = crc >> 8;
	d[1] = crc & 0xff;
	vpInterf->Tx(0, d, 2);

	// Wait for respond data
	// It took about 99992 loop until receiving respond
	int t = 100000;
	do
	{
		vpInterf->Rx(0, d, 1);
	} while (d[0] == 0xff && --t > 0);

	if ((d[0] & 0x1f))
	{
		if ((d[0] & 0x1f) != 0x5)
		{
			// Failed write, read status
			int r = Cmd(13, 0);
			//printf("SDCard Write resp error: %x %d, r = %x\n\r", d[0], t, r);
			return 0;
		}
	}

	return cnt;
}

int SDCard::GetSectSize(void)
{
	return vDev.SectSize;
}

uint32_t SDCard::GetNbSect(void)
{
	return vDev.TotalSect;
}

// @return	size in KB
uint32_t SDCard::GetSize(void)
{
	uint8_t data[20];
	uint32_t c_size, c_size_mult, read_bl_len;
	uint32_t size = 0;

	memset(data, 0, 20);

	int r = Cmd(9, 0);
	if (r == 0)
		ReadData(data, 16);

	if ((data[0] & 0xc0) == 0)
	{
		// Vers 1.0
		c_size = (uint16_t)data[7]<<2;
		c_size |= (data[8] >> 6) & 0x03; // bit 62-63
		c_size |= (data[6] & 3) << 10;
		c_size++;
		c_size_mult = 4 << ((data[10] >> 7) | ((data[9] & 3) << 1));
		read_bl_len = 1 << (data[5] & 0x0F);
		size = c_size * c_size_mult * read_bl_len / 1024;
	}
	else
	{
		// Vers 2.0
		// Bits 48-69
		size = (uint64_t)(((data[7] & 0x3f) << 16u) | (data[8] << 8u) | data[9]) * 512;
	}

	return size;
}

int SDCard::ReadSingleBlock(uint32_t Addr, uint8_t *pData, int len)
{
	int retval = 0;

	if (pData)
	{
		uint32_t state = DisableInterrupt();
		int r = Cmd(17, Addr);
		if (r == 0)
			retval = ReadData(pData, len);
		EnableInterrupt(state);
	}
	return retval;
}

int SDCard::WriteSingleBlock(uint32_t Addr, uint8_t *pData, int Len)
{
	int retval = 0;

	if (pData)
	{
		uint32_t state = DisableInterrupt();
		int r = Cmd(24, Addr);
		if (r == 0)
			retval =  WriteData(pData, Len);
		EnableInterrupt(state);
	}

	return retval;
}
