OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x0001B000, LENGTH = 0x25000 
	RAM (rwx) :  ORIGIN = 0x20002100, LENGTH = 0x1F00 
}

SECTIONS
{
	.fs_data : ALIGN(4)
	{
		PROVIDE( __start_fs_data = .);
		KEEP(*(.fs_data))
		PROVIDE( __stop_fs_data = .);
	} > RAM AT > FLASH
	.pwr_mgmt_data :
	{
    	PROVIDE(__start_pwr_mgmt_data = .);
    	KEEP(*(.pwr_mgmt_data))
    	PROVIDE(__stop_pwr_mgmt_data = .);
	} > RAM AT > FLASH
} INSERT AFTER .data

INCLUDE "gcc_arm_flash.ld"
