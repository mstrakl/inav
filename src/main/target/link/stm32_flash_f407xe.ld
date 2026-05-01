/*
*****************************************************************************
**
**  File        : stm32_flash_f407xe.ld
**
**  Abstract    : Linker script for STM32F407xE Device with
**                512KByte FLASH, 128KByte RAM 64KByte CCM (RAM)
**
*****************************************************************************
*/

/* Stack & Heap sizes */
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x1800;

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x08000000 to 0x0807FFFF 512K full flash,
0x08000000 to 0x0805FFFF 384K firmware,
0x08060000 to 0x0807FFFF 128K config,       // FLASH_Sector_7
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 384K
    FLASH_CONFIG (r)  : ORIGIN = 0x08060000, LENGTH = 128K

    RAM (rwx)         : ORIGIN = 0x20000000, LENGTH = 128K
    CCM (rwx)         : ORIGIN = 0x10000000, LENGTH = 64K
    BACKUP_SRAM (rwx) : ORIGIN = 0x40024000, LENGTH = 4K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)

INCLUDE "stm32_flash.ld"
