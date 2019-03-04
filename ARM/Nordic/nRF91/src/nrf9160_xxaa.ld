/* nRF9160 Linker script to configure memory regions. */

SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
SEARCH_DIR("../../EHAL/ARM/src")
SEARCH_DIR("../../../EHAL/ARM/src")
SEARCH_DIR("../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../../../../../EHAL/ARM/src")
SEARCH_DIR("../../../../EHAL/ARM/Nordic/nRF91/CMSIS/Release")
SEARCH_DIR("../../../../../EHAL/ARM/Nordic/nRF91/CMSIS/Release")
SEARCH_DIR("../../../../../../EHAL/ARM/Nordic/nRF91/CMSIS/Release")
SEARCH_DIR("../../../../../../../EHAL/ARM/Nordic/nRF91/CMSIS/Release")
SEARCH_DIR("../../../../../../../../EHAL/ARM/Nordic/nRF91/CMSIS/Release")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x100000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x40000
}

INCLUDE "gcc_arm_flash.ld"
