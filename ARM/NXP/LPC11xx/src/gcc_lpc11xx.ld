/*
 * LPC11xx
 */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lgcc -lc -lCMSIS -lnosys)


MEMORY
{
  
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
  	RAM (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
  	RAM2 (rwx) : ORIGIN = 0x20004000, LENGTH = 2K
}

INCLUDE "gcc_arm_flash.ld"



