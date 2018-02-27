OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
SEARCH_DIR("../../EHAL/ARM/src")
SEARCH_DIR("../../../EHAL/ARM/src")
SEARCH_DIR("../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../EHAL/ARM/nRF51/CMSIS/Release")
SEARCH_DIR("../../../../../EHAL/ARM/nRF51/CMSIS/Release")
SEARCH_DIR("../../../../../../EHAL/ARM/nRF51/CMSIS/Release")
GROUP(-lCMSIS -lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

INCLUDE "gcc_arm_flash.ld"



