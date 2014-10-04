OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

/* 
Total RAM 256K, 
*/

MEMORY
{
  RAM (rwx) : ORIGIN = 0x20004000, LENGTH = 0x00030000
}
/* Library configurations */
GROUP (
	libgcc.a 
	libc.a 
	libstdc++.a 
	libnosys.a
)

INCLUDE "gcc_arm_ram.ld"

