OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

/* 
Total Flash 256K, 
88K for s110, 15K for DFU
153K avail for application
*/

MEMORY
{
  FLASH (rx) : ORIGIN = 0x16000, LENGTH = 256K - 0x16000 - 15K 
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8 kB, 8 kB is taken by S110. */
}

INCLUDE "gcc_arm.ld"

