OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x12000, LENGTH = 0x6e000
	RAM (rwx) :  ORIGIN = 0x20000a00, LENGTH = 0x7600
}

INCLUDE "gcc_arm_flash.ld"
