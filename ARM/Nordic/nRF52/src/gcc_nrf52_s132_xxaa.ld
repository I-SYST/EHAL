OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x20000, LENGTH = 0x60000
	RAM (rwx) :  ORIGIN = 0x20002200, LENGTH = 0xde00
}

SECTIONS
{
	.fs_data :
	{
		PROVIDE(__start_fs_data = .);
		KEEP(*(.fs_data))
		PROVIDE(__stop_fs_data = .);
	} > RAM AT > FLASH
} INSERT AFTER .data;

INCLUDE "gcc_arm_flash.ld"