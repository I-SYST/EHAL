/*--------------------------------------------------------------------------
File   : sha1.c

Author : Hoang Nguyen Hoan          Aug. 17, 2014

Desc   : SHA-1 computation

Copyright (c) 2014, I-SYST inc., all rights reserved

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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "istddef.h"
#include "sha1.h"

/*
 * Test cases
 * Data   : null, zero length
 * SHA1   : da39a3ee 5e6b4b0d 3255bfef 95601890 afd80709
 *
 * Data   : "abc"
 * SHA1 : a9993e36 4706816a ba3e2571 7850c26c 9cd0d89d
 *
 * Data   : "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
 * SHA1 : 84983e44 1c3bd26e baae4aa1 f95129e5 e54670f1
 *
 * Data   : repeat 'a' 1000000 times
 * SHA1 : 34aa973c d4c4daa4 f61eeb2b dbad2731 6534016f
 *
 * Data   : "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"
 * SHA1 :  a49b2446 a02c645b f419f995 b6709125 3a04a259
 *
 */

#define K0	0x5a827999
#define K1	0x6ed9eba1
#define K2 	0x8f1bbcdc
#define K3	0xca62c1d6

#define H0	0x67452301
#define H1	0xefcdab89
#define H2	0x98badcfe
#define H3	0x10325476
#define H4	0xc3d2e1f0

inline uint32_t ROTR(uint32_t x, uint32_t n)
{
    return (x >> n) | (x << (32-n));
}

inline uint32_t ROTL(uint32_t x, uint32_t n)
{
	return (x << n) | (x >> (32 - n));
}

inline uint32_t CH(uint32_t x, uint32_t y, uint32_t z)
{
	return (x & y) ^ (~x & z);
}

inline uint32_t MAJ(uint32_t x, uint32_t y, uint32_t z)
{
	return (x & y) ^ (x & z) ^ (y & z);
}

inline uint32_t PAR(uint32_t x, uint32_t y, uint32_t z)
{
	return (x ^ y) ^ z;
}

static void Sha1Compute(uint32_t *W, uint32_t *H)
{
	uint32_t a, b, c, d, e;

	a = H[0];
	b = H[1];
	c = H[2];
	d = H[3];
	e = H[4];

	for (int t = 0; t < 80; t++)
	{
		uint32_t T;

		if (t > 15)
			W[t] = ROTL(((W[t-3] ^ W[t-8]) ^ W[t-14]) ^ W[t-16], 1);

		if (t < 20)
		{
			T = CH(b, c, d) + K0;
		}
		else if (t < 40)
		{
			T = PAR(b, c, d) + K1;
		}
		else if (t < 60)
		{
			T = MAJ(b, c, d) + K2;
		}
		else
		{
			T = PAR(b, c, d) + K3;
		}

		T += ROTL(a, 5) + e + W[t];

		e = d;
		d = c;
		c = ROTL(b, 30);
		b = a;
		a = T;
	}

	H[0] = (H[0] + a);
	H[1] = (H[1] + b);
	H[2] = (H[2] + c);
	H[3] = (H[3] + d);
	H[4] = (H[4] + e);
}

static int g_LastWIdx = 0;
static int g_LastOctet = 0;
static uint64_t g_TotalBitLen = 0;
static char g_Sha1Digest[34] = { 0,};
static uint32_t W[80];
static uint32_t H[5] = { H0, H1, H2, H3, H4 };

/*
 * Generate SHA digest code.  Call this function until all data are processed.
 * set bLast parameter to true for last data packet to process.
 *
 * Make sure to have enough memory for returning results.  pRes must have at
 * least 33 bytes.
 *
 * @param 	pSrc 	: Pointer to source data
 * 			SrcLen	: Source data length in bytes
 *			bLast	: set true to indicate last data packet
 * 			pRes	: Pointer to buffer to store results of 32 characters
 * 					  if NULL is passed, internal buffer will be used
 *
 * 	@return	Pointer to digest string. If pRes is NULL, internal buffer is returned
 * 			NULL if incomplete
 */
char *Sha1(uint8_t *pData, int DataLen, bool bLast, char *pRes)
{
	uint8_t *p = pData;
	int t = 0, j = 0;
	char *digest = g_Sha1Digest;

	g_TotalBitLen += DataLen << 3;

	if (g_LastOctet || g_LastWIdx)
	{
		// We have incomplete buffer from previous call
		t = g_LastWIdx;

		// Fill up left over from previous 32bits
		for (int j = g_LastOctet; j < 4; j++)
		{
			W[t] |= *p << (24 - (j << 3));
			DataLen--;
			p++;
		}
		t++;
		// Fill up th rest of the 512 bits message
		for (; t < 16 && DataLen > 3; t++)
		{
			W[t] = p[3] | (p[2] << 8) | (p[1] << 16) | (p[0] << 24);
			p += 4;
			DataLen -= 4;
		}
		if (t >= 16)
		{
			// We have complete 512
			Sha1Compute(W, H);
			memset(W, 0, sizeof(W));
			t = 0; j = 0;
		}
	}

	if (DataLen > 64)
	{
		// Process N complete 512 bits message
		int n = DataLen / 64;
		for (int i = 0; i < n; i++)
		{
			for (t = 0; t < 16; t++)
			{
				W[t] = p[3] | (p[2] << 8) | (p[1] << 16) | (p[0] << 24);
				p += 4;
				DataLen -= 4;
			}
			Sha1Compute(W, H);
		}
		t = 0;
		j = 0;
		memset(W, 0, sizeof(W));
	}
	g_LastWIdx = 0;
	g_LastOctet = 0;
	if (DataLen)
	{
		// Still have incompleted data
		for (t = 0; t < 16 && DataLen > 3; t++)
		{
			W[t] = p[3] | (p[2] << 8) | (p[1] << 16) | (p[0] << 24);
			p += 4;
			DataLen -= 4;
		}
		for (j = 0; j < DataLen; j++)
		{
			W[t] |= *p << (24 - (j << 3));
			p++;
		}
	}
	if (bLast == false)
	{
		// More data to come, remember where we are
		g_LastWIdx = t;
		g_LastOctet = j;

		return NULL;
	}

	if (bLast)
	{
		// All data processed.  add the 1 bit & data len
		W[t] |= 0x80 << (24 - (j << 3));
		t++;
		if (t > 14)
			Sha1Compute(W, H);
		{
			W[14] = g_TotalBitLen >> 32;
			W[15] = g_TotalBitLen & 0xffffffff;
		}
		Sha1Compute(W, H);

		if (pRes)
			digest = pRes;

		sprintf(digest, "%08X%08X%08X%08X%08X", H[0], H[1], H[2], H[3], H[4]);

		// Reset memory, ready for new processing

		H[0] = H0;
		H[1] = H1;
		H[2] = H2;
		H[3] = H3;
		H[4] = H4;

		memset(W, 0, sizeof(W));
		g_LastWIdx = 0;
		g_LastOctet = 0;
		g_TotalBitLen = 0;
	}

	return digest;
}
