OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../src")
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
SEARCH_DIR("../../../../../EHAL/ARM/src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x80000
	RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x8000
}


INCLUDE "gcc_arm_flash.ld"
