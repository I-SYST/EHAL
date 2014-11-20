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

#include "iopincfg.h"
#include "sdcard.h"
#include "crc.h"

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

int SDGetResponse(SDDEV *pDev, uint8_t *pBuff, int BuffLen)
{
	uint8_t d;
	int cnt = 0;

	d = 0xff;
	LpcSSPStartRx(pDev->pSspDev, 0);
	cnt = LpcSSPRxData(pDev->pSspDev, pBuff, BuffLen);
	LpcSSPStopRx(pDev->pSspDev);

	return cnt;
}

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
	data[5] = SDCmdCrc(data) | 1;
	data[6] = 0xff;

	// Send command
	LpcSSPStartTx(pDev->pSspDev, 0);
	LpcSSPTxData(pDev->pSspDev, data, 6);
	LpcSSPStopTx(pDev->pSspDev);

	// wait for response
	t = 100000;
	do {
		LpcSSPStartRx(pDev->pSspDev, 0);
		LpcSSPRxData(pDev->pSspDev, &r, 1);
		LpcSSPStopRx(pDev->pSspDev);
	} while (r ==0xff && --t > 0);

	//printf("Cmd r = %x\n\r", r);

	return r;
}

bool SDInit(SDDEV *pDev, SDCFG *pCfg)
{
	if (pDev == NULL || pCfg == NULL)
		return false;

	if (pCfg->pSspDev == NULL)
		return false;

	uint8_t data[10];
	uint8_t r = 0xff;

	pDev->pSspDev = pCfg->pSspDev;

	// Send 80 x 0xff to reset card to native mode
	LpcSSPStartTx(pDev->pSspDev, 0);
	for (int i = 0; i < 80; i++)
	{
		LpcSSPTxData(pDev->pSspDev, &r, 1);
	}
	LpcSSPStopTx(pDev->pSspDev);

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
*/

SDCard::SDCard()
{

}

SDCard::~SDCard()
{

}

bool SDCard::Init(SerialIntrf *pSerInterf)
{
	uint8_t data[10];
	uint8_t r = 0xff;
	vpInterf = std::shared_ptr<SerialIntrf>(pSerInterf);
	//vpInterf = pSerInterf;

	// Send 80 x 0xff to reset card to native mode
	for (int i = 0; i < 80; i++)
	{
		vpInterf->Tx(0, &r, 1);
	}

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
//		printf("CMD8 r = %02x, data = 0x%08x, len = %d\n\r", r, *(uint32_t*)data, i);
	}

	//LpcSSP *p = (LpcSSP*)vpInterf.get();
	//vDev.pSspDev = (SSPDEV*)*p;//((LpcSSP*)&vpInterf);

	return true;
}

int SDCard::Cmd(uint8_t Cmd, uint32_t param)
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
	data[5] = crc8(data, 5) | 1; //SDCmdCrc(data) | 1;
	data[6] = 0xff;

	// Send command
	vpInterf->Tx(0, data, 6);

	// wait for response
	t = 100000;
	do {
		vpInterf->Rx(0, &r, 1);
	} while (r ==0xff && --t > 0);

	//printf("Cmd r = %x\n\r", r);

	return r;
}

int SDCard::GetResponse(uint8_t *pBuff, int BuffLen)
{
//	uint8_t d;
	int cnt = 0;

//	d = 0xff;
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
	//printf("Calc CRC : %04x\n\r", calccrc);

	if (cnt <= BuffLen)
	{
		crc = 0;
		vpInterf->Rx(0, (uint8_t*)&crc, 1);
		crc <<= 8;
		vpInterf->Rx(0, &d, 1);

		crc |= d;
		//printf("CRC : %04x, calc : %04x\n\r", crc, calccrc);
	}
	if (crc != calccrc)
		return -2;

	return cnt;
}

int SDCard::WriteData(uint8_t *pData, int Len)
{
	int i;
	uint16_t crc;
	uint8_t d[2] = { 0xff, 0xfe };

	if (pData == NULL)
		return -1;

	crc = crc16(pData, Len);

	vpInterf->Tx(0, d, 2);

	i = 0;

	i = vpInterf->Tx(0, pData, Len);

	d[0] = crc >> 8;
	d[1] = crc & 0xff;
	vpInterf->Tx(0, d, 2);

	int t = 100000;
	do
	{
		vpInterf->Rx(0, d, 1);
		if (d[0] != 0xff && d[0] != 0)
			break;
	} while (--t > 0);

	if ((d[0] & 0xe) == 0x4)
	{
		t = 100000;

		do {
			vpInterf->Rx(0, d, 1);
			if (d[0] != 0)
				break;
		} while (t-- > 0);

		return i;
	}

	return 0;
}

uint32_t SDCard::GetSize(void)
{
	uint8_t data[20];
	uint32_t c_size, c_size_mult, read_bl_len;

	int r = Cmd(9, 0);
	if (r == 0)
		ReadData(data, 16);

	c_size = (uint16_t)data[7]<<2;
	c_size |= (data[8] >> 6) & 0x03; // bit 62-63
	c_size |= (data[6] & 3) << 10;
	c_size++;

	c_size_mult = 4 << ((data[10] >> 7) | ((data[9] & 3) << 1));

	read_bl_len = 1 << (data[5] & 0x0F);

//	printf("c_size : %d, c_mult : %d, read_bl_len = %d\n\r", c_size, c_size_mult, read_bl_len);

	return c_size * c_size_mult * read_bl_len;
}

int SDCard::ReadSingleBlock(uint32_t Addr, uint8_t *pData, int len)
{
	if (pData)
	{
		int r = Cmd(17, Addr);
		if (r == 0)
			return ReadData(pData, len);
	}
	return 0;
}

int SDCard::WriteSingleBlock(uint32_t Addr, uint8_t *pData, int Len)
{
	if (pData)
	{
		int r = Cmd(24, Addr);
		if (r == 0)
			return WriteData(pData, Len);
	}

	return 0;
}
/*
uint32_t SDGetSize(HSDDEV hDev)
{
	if (hDev == NULL)
		return 0;

	return ((SDCard *)hDev)->GetSize();
}

int SDReadSingleBlock(HSDDEV hDev, uint32_t Addr, uint8_t *pData, int Len)
{
	if (hDev == NULL)
		return 0;

	return ((SDCard *)hDev)->ReadSingleBlock(Addr, pData, Len);
}

int SDWriteSingleBlock(HSDDEV hDev, uint32_t Addr, uint8_t *pData, int Len)
{
	if (hDev == NULL)
		return 0;

	return ((SDCard *)hDev)->WriteSingleBlock(Addr, pData, Len);
}



inline int WaitRxFifo(SDDEV *pDev, int TimoutCnt) {
	while (SSP_GetStatus(pDev->pSsp, SSP_STAT_RXFIFO_NOTEMPTY) == RESET && --TimoutCnt > 0);

	if (TimoutCnt <= 0)
		return 0;

	return 1;
}

inline int WaitTxFifo(SDDEV *pDev, int TimoutCnt) {
	while (SSP_GetStatus(pDev->pSsp, SSP_STAT_TXFIFO_NOTFULL) == RESET && --TimoutCnt > 0);

	if (TimoutCnt <= 0)
		return 0;

	return 1;
}

int SDInit(SDDEV *pDev)
{
	//SSP_CFG_Type ssp;
	//PINSEL_CFG_Type pincfg;
	uint8_t data[50];
	int i, r;

	// Configure I/O pin function
	//for (i = 0; i < SD_NB_IOPIN; i++)
	//	PINSEL_ConfigPin(&pDev->IOMap[i]);
	LpcSSPInit(&g_SspDev, &g_SspCfg);

	// Configure SSP in SPI mode
	//SSP_ConfigStructInit(&ssp);
	//ssp.ClockRate = 400000;	// in Hz
    //SSP_Init(pDev->pSsp, &ssp);

	//SSP_Cmd(pDev->pSsp, ENABLE);

	// Reset card to native mode
	for (i = 0; i < 80; i++)
		LpcSSPStartTx(&g_SspDev, );
		//SSP_SendData(pDev->pSsp, 0xff);

	// Activate SPI mode
	r = SDCmd(pDev, 0, 0);

	if ((r & 0xfe))
		return r;	// Card not usable or not found

	// CMD8 must be sent before activating with ACMD41
	uint32_t acmd = 0x10000000;
	r = SDCmd(pDev, 8, 0x1AA);
	if ((r & 0xfe) == 0)
	{
		i = SDReadResponse(pDev, data, 5);
		if (i > 0 && data[2] == 1 && data[3] == 0xAA)
		{
			acmd = 0x50000000;
		}
	}

	// Start card initialisation
	// send ACMD41
	i = 10;
	do {
		r = SDCmd(pDev, 55, 0x0000000);
		if ((r & 0xfe) == 0)
		{
			r = SDCmd(pDev, 41, acmd);
			SDReadResponse(pDev, data, 5);
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
		i = SDReadResponse(pDev, data, 5);
		//printf("CMD8 r = %02x, data = 0x%08x, len = %d\n\r", r, *(uint32_t*)data, i);
	}

    return r;
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
/*
int SDCmd(SDDEV *pDev, uint8_t Cmd, uint32_t param)
{
	int i, t;
	uint8_t data[6];
	uint8_t r;

	memset(data, 0, sizeof(data));

	// Fill cmd buffer
	data[0] = Cmd | 0x40;
	data[1] = (param >> 24) & 0xff;
	data[2] = (param >> 16) & 0xff;
	data[3] = (param >> 8) & 0xff;
	data[4] = param & 0xff;
	data[5] = SDCmdCrc(data) | 1;

	while (SSP_GetStatus(pDev->pSsp, SSP_STAT_BUSY) == SET);

	for (i = 0; i < 6; i++)
	{
		WaitTxFifo(pDev, 900000);
		SSP_SendData(pDev->pSsp, data[i]);
	}

	// wait for response
	t = 100000;
	do {
		SSP_SendData(pDev->pSsp, 0xff);
		WaitRxFifo(pDev, 500000);
		r = SSP_ReceiveData(pDev->pSsp);
	} while (r == 0xff && --t > 0);

	if (t <= 0)
		r = 0xff;

	return r;
}
*/
/*
	Read extra response data

	@param	SDDEV *pDev : Reference to device data
	@param	uint8_t *pData	: Reference to data buffer
	@paran 	int Len			: Data buffer length

	@return	total length (bytes) if succeeded
*/
/*
int SDReadResponse(SDDEV *pDev, uint8_t *pData, int Len)
{
	int i;

	if (pData == NULL)
		return 0;

	i = 0;
	// Read response
	do {
		SSP_SendData(pDev->pSsp, 0xff);
		WaitRxFifo(pDev, 500000);
		pData[i] = SSP_ReceiveData(pDev->pSsp);
	} while (++i < Len);

	return i;
}

int SDReadData(SDDEV *pDev, uint8_t *pData, int len)
{
	int timeout, cnt, crc, calccrc;
	uint8_t d;

	if (pData == NULL)
		return 0;

	timeout = 100000;

	do {
		SSP_SendData(pDev->pSsp, 0xff);
		WaitRxFifo(pDev, 500000);
		d = SSP_ReceiveData(pDev->pSsp);
	} while (d != 0xfe && --timeout > 0 );

	if (timeout <= 0)
		return -1;

	if (d != 0xfe)
	{
		pData[0] = d;
		return -1;
	}

	cnt = 0;

	do {
		SSP_SendData(pDev->pSsp, 0xff);
		WaitRxFifo(pDev, 500000);
		pData[cnt++] = SSP_ReceiveData(pDev->pSsp);
	} while (cnt < len);

	calccrc = crc16(pData, cnt);
	//printf("Calc CRC : %04x\n\r", calccrc);

	if (cnt <= len)
	{
		SSP_SendData(pDev->pSsp, 0xff);
		WaitRxFifo(pDev, 500000);
		crc = SSP_ReceiveData(pDev->pSsp) << 8;;
		SSP_SendData(pDev->pSsp, 0xff);
		WaitRxFifo(pDev, 500000);
		crc |= SSP_ReceiveData(pDev->pSsp);
		//printf("CRC : %04x\n\r", crc);
	}
	if (crc != calccrc)
		return -2;

	return cnt;
}

int SDWriteData(SDDEV *pDev, uint8_t *pData, int Len)
{
	int i;
	uint16_t crc;
	uint8_t d;

	if (pData == NULL)
		return -1;

	crc = crc16(pData, Len);


	SSP_SendData(pDev->pSsp, 0xff);

	WaitTxFifo(pDev, 900000);
	SSP_SendData(pDev->pSsp, 0xfe);

	i = 0;

	do {
		WaitTxFifo(pDev, 900000);
		SSP_SendData(pDev->pSsp, pData[i++]);
	} while (i < Len);

	WaitTxFifo(pDev, 900000);
	SSP_SendData(pDev->pSsp, (crc >> 8) & 0xff);
	WaitTxFifo(pDev, 900000);
	SSP_SendData(pDev->pSsp, crc & 0xff);

	while (SSP_GetStatus(pDev->pSsp, SSP_STAT_BUSY) == SET);

	int t = 1000000;
	do
	{
		SSP_SendData(pDev->pSsp, 0xff);
		WaitRxFifo(pDev, 500000);
		d = SSP_ReceiveData(pDev->pSsp);
		if (d != 0xff && d != 0)
			break;
	} while (--t > 0);

	if ((d & 0xe) == 0x4)
	{
		t = 100000;

		do {
			SSP_SendData(pDev->pSsp, 0xff);
			WaitRxFifo(pDev, 500000);
			if (SSP_ReceiveData(pDev->pSsp) != 0)
				break;
		} while (t-- > 0);

		return i;
	}

	return 0;
}

uint32_t SDGetSize(SDDEV *pDev)
{
	uint8_t i, by;
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

	//printf("c_size : %d, c_mult : %d, read_bl_len = %d\n\r", c_size, c_size_mult, read_bl_len);

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
*/
