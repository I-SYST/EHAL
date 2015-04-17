OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

/* SDK v8 */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x18000, LENGTH = 0x28000 
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8 kB, 8 kB is taken by S110. */
}

INCLUDE "gcc_arm_flash.ld"

