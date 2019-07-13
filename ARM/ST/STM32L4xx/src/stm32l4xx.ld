SEARCH_DIR(.)
SEARCH_DIR("../../../src")
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
GROUP(-lnosys -lstdc++ -lgcc -lc -lCMSIS)


MEMORY
{
  
	FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 1M
  	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
  	RAM2 (rwx) : ORIGIN = 0x10000000, LENGTH = 32K
 }

INCLUDE "gcc_arm_flash.ld"

