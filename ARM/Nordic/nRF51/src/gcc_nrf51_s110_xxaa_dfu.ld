OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../src")
GROUP(-lgcc -lc -lnosys -lCMSIS)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x3C000, LENGTH = 0x3C00 /* bootloader */
  bootloader_settings (rwx) : ORIGIN = 0x3FC00, LENGTH = 0x400 /* bootloader specific settings */
  NRF_UICR_BOOT_START (rwx) : ORIGIN = 0x10001014, LENGTH = 0x4 /* bootloader start address in UICR register */
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8 kB, 8 kB is taken by S110. */
}

SECTIONS {
    .bootloader_settings_block 0x0003FC00 :
    {
        KEEP(*(.bootloaderSettings))
    } > bootloader_settings
  
    .NRF_UICR_BOOT_START_BLOCK 0x10001014 :
    {
        KEEP(*(.uicrBootStartAddress))
    } > NRF_UICR_BOOT_START
    
}

INCLUDE "gcc_arm_flash.ld"

