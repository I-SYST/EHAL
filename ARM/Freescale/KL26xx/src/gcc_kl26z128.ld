/*
 * LPC11xx
 */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lCMSIS -lnosys -lstdc++ -lgcc -lc)


MEMORY
{  
	FLASH (rx) 	: ORIGIN = 0x00000000, LENGTH = 128K
  	RAM (rwx) 	: ORIGIN = 0x1FFFF000, LENGTH = 16K
}

INCLUDE "gcc_arm_flash.ld"



