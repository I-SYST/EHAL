/*--------------------------------------------------------------------------
File   : crc.c

Author : Hoang Nguyen Hoan          Dec. 27, 2011

Desc   : CRC calculations
		 reference : http://en.wikipedia.org/wiki/Computation_of_CRC

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
#include "crc.h"

/**
 * @brief   Calculate 8 bits CRC value
 *          Polynomial : (x7 + x3 + 1) × x (left-shifted CRC-7-CCITT)
 *          0x12 = (0x09 << 1) (MSBF/normal)
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 * 			SeedVal : Initial CRC seed value
 *
 * @return	8 bits CRC value
 */
uint8_t crc8_ccitt(uint8_t *pData, int Len, uint8_t SeedVal)
{
	uint8_t e, f, crc;

	crc = SeedVal;
	for (int i = 0; i < Len; i++)
	{
		e = crc ^ pData[i];
		f = e ^ (e >> 4) ^ (e >> 7);
		crc = (f << 1) ^ (f << 4);
	}
	return crc;
}

/**
 * @brief   Calculate 8 bits CRC value
 *
 * Brute force general crc calculation where polynomial value is passed
 * as parameter
 *
 *	ex. : ATM Polynomial : x8 + x2 + x + 1 => 0x107
 *
 * @param	Poly	: Polynomial value
 * 			pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 * 			SeedVal : Initial CRC seed value
 *
 * @return	8 bits CRC value
 */
uint8_t crc8(uint16_t Poly, uint8_t *pData, int Len, uint8_t SeedVal)
{
	uint16_t crc = SeedVal;
	Poly <<= 7;

	for (int i = 0; i < Len; i++)
	{
		crc ^= pData[i] << 8;

		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x8000)
			{
				crc ^= Poly;
			}
			crc <<= 1;
		}
	}

	return crc >> 8;
}

/**
 * @brief   Calculate 16 bits CRC value
 *          Polynomial : x16 + x15 + x2 + 1 (CRC-16-ANSI)
 *          0x8005 (MSBF/normal)
 *
 * @param   pData   : Pointer to data buffer to calculate
 *          Len     : Data length in bytes
 *          SeedVal : Initial CRC seed value
 *
 * @return 16 bits CRC value
 */
uint16_t crc16_ansi(uint8_t *pData, int Len, uint16_t SeedVal)
{
    uint8_t  s;
    uint16_t t, crc;

    crc = SeedVal;
    for (int i = 0; i < Len; i++)
    {
        s = pData[i] ^ (crc >> 8);
        t = s ^ (s >> 4);
        t ^= (t >> 2);
        t ^= (t >> 1);
        t &= 1;
        t |= (s << 1);
        crc = (crc << 8) ^ t ^ (t << 1) ^ (t << 15);
    }

    return crc;
}

/**
 * @brief   Calculate 16 bits CRC value
 *          Polynomial : x16 + x12 + x5 + 1 (CRC-16-CCITT)
 *          0x1021 (MSBF/normal)
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 *          SeedVal : Initial CRC seed value
 *
 * @return	16 bits CRC value
 */
uint16_t crc16_ccitt(uint8_t *pData, int Len, uint16_t SeedVal)
{
	uint8_t  s, t;
	uint16_t crc;

	crc = SeedVal;
	for (int i = 0; i < Len; i++)
	{
		s = pData[i] ^ (crc >> 8);
		t = s ^ (s >> 4);
		crc = (crc << 8) ^ t ^ (t << 5) ^ (t << 12);
	}

	return crc;
}

/**
 * @brief	Calculate 32 bits CRC value
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 *
 * @return	32 bits CRC value
 */
uint32_t crc32(uint8_t *pData, int Len)
{
	char byte; 	// current byte
	uint32_t crc; 	// CRC result
	int q0, q1, q2, q3; // temporary variables
	crc = 0xFFFFFFFF;

	for (int i = 0; i < Len; i++)
	{
		byte = *pData++;

		for (int j = 0; j < 2; j++)
		{
			if (((crc >> 28) ^ (byte >> 3)) & 0x00000001)
			{
				q3 = 0x04C11DB7;
			}
			else
			{
				q3 = 0x00000000;
			}

			if (((crc >> 29) ^ (byte >> 2)) & 0x00000001)
			{
				q2 = 0x09823B6E;
			}
			else
			{
				q2 = 0x00000000;
			}

			if (((crc >> 30) ^ (byte >> 1)) & 0x00000001)
			{
				q1 = 0x130476DC;
			}
			else
			{
				q1 = 0x00000000;
			}

			if (((crc >> 31) ^ (byte >> 0)) & 0x00000001)
			{
				q0 = 0x2608EDB8;
			}
			else
			{
				q0 = 0x00000000;
			}

			crc = (crc << 4) ^ q3 ^ q2 ^ q1 ^ q0;
			byte >>= 4;
		}
	}

	return crc;
}
