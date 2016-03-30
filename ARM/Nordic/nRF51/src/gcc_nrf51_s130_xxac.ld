/*
S130 Peripheral mode
*/

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
SEARCH_DIR("../../../../../src")
SEARCH_DIR("../../../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x0001B000, LENGTH = 0x25000 
  RAM (rwx) :  ORIGIN = 0x20001F00, LENGTH = 0x6100 
}

SECTIONS
{
  .fs_data_out ALIGN(4):
  {
    PROVIDE( __start_fs_data = .);
    KEEP(*(fs_data))
    PROVIDE( __stop_fs_data = .);
  } = 0
}

INCLUDE "gcc_arm_flash.ld"
