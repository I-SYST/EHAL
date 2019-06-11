SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
GROUP(-lCMSIS -lnosys -lstdc++ -lgcc -lc)


MEMORY
{
  
	FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 64K
  	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
 }

INCLUDE "gcc_arm_flash.ld"

