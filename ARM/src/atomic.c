/*--------------------------------------------------------------------------
File   : atomic.c

Author : Hoang Nguyen Hoan          Mar. 22, 2015

Desc   : atomic implemention using interrupt for Cortex-m0

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
#include "atomic.h"

int __atomic_fetch_add_4(int *d, int val, int mem)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	*d += val;
	__set_PRIMASK(primask);

	return *d;
}

int __atomic_fetch_sub_4(int *d, int val, int mem)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	*d -= val;
	__set_PRIMASK(primask);

	return *d;
}
/*
uint32_t DisableInterrupt()
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	return primask;
}

void EnableInterrupt(uint32_t State)
{
	__set_PRIMASK(State);
}

*/
