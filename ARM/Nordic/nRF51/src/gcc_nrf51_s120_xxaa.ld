OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x0001D000, LENGTH = 0x23000 
  RAM (rwx) :  ORIGIN = 0x20002800, LENGTH = 0x1800
}


INCLUDE "gcc_arm_flash.ld"
