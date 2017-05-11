OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x12000, LENGTH = 0x6e000
	RAM (rwx) :  ORIGIN = 0x20000a00, LENGTH = 0x7600
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

SECTIONS
{
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(SORT(.pwr_mgmt_data*)))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > FLASH
} INSERT AFTER .text

INCLUDE "gcc_arm_flash.ld"
