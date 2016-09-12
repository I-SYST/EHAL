/*--------------------------------------------------------------------------
File   : gcc+lpc1767_9.ld

Author : Hoang Nguyen Hoan          Jan. 19, 2012

Desc   : Generic linker script for LPC1767-9 

Copyright (c) 2012, I-SYST, all rights reserved

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
OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../src")
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lCMSIS -lnosys -lstdc++ -lgcc -lc)

/*
 * Define memory regions.
 * LPC1768-69 has 512KB Flash & 64KB RAM
 */
MEMORY
{
   FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 512K
   RAM (rwx) : ORIGIN = 0x10000000, LENGTH = 32K
   RAMAHB (rwx) : ORIGIN = 0x2007c000, LENGTH = 32K
   /* The AHB sram is divided into 2 regions of 16KB*/
   RAMAHB0 (rwx) : ORIGIN = 0x2007c000, LENGTH = 16K	/* Bank 0 */
   RAMAHB1 (rwx) : ORIGIN = 0x20080000, LENGTH = 16K	/* Bank 1 */
}

/*ENTRY(ResetEntry)*/

/* Library configurations */
/*GROUP (
	
	libgcc.a 
	libc.a 
	libstdc++.a 
	libnosys.a
)
*/

SECTIONS 
{
	.data_RAMAHB :
	{
		. = ALIGN(4);        
		__data_ahb_start__ = .;
/*		*(RAMAHB0*)
		*(.data.RAMAHB0*)
		*(.data.RAMAHB1*)
		*(.RAMAHB1*)
*/		*(.data.RAM2*)
		*(.data.$RAM2*)
		*(.data.$RamAHB32*)
        *(RAM2*)
        *(RAMAHB*)
        *(.RAMAHB*)
		*(.data.RAMAHB*)
		. = ALIGN(4);        
   		__data_ahb_end__ = .;   	
	} > RAMAHB AT > FLASH
   	__data_ahb_loc__ = LOADADDR(.data_RAMAHB);
   	__data_ahb_size__ = SIZEOF(.data_RAMAHB);
}

INCLUDE "gcc_arm_flash.ld"

