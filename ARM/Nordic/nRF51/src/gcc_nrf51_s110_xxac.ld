/* Linker script to configure memory regions. */

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
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00016000, LENGTH = 0x2A000 
  RAM (rwx) :  ORIGIN = 0x20002000, LENGTH = 0x6000 
}


INCLUDE "gcc_nrf51_common.ld"
