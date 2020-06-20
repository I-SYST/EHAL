OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
SEARCH_DIR("../../../../../../../src")
SEARCH_DIR("../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../../../EHAL/ARM/src")
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00100000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00020000
}

INCLUDE "gcc_arm_flash.ld"

