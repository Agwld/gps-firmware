/*
 * Linker script for STM32G431CBTx (128 KB flash / 32 KB RAM) - the fitted
 * part on the gps-mainboard (rev 1.0.0).
 *
 * FLASH is 124 KB: the last two 2 KB pages (0x0801F000-0x0801FFFF) are
 * reserved for the flash_store config/gate records and must never hold
 * code. Two pages, not one, so the store can rewrite the compacted state
 * into a spare page and only erase the old one once that rewrite has
 * fully landed (flash_store_erase_and_compact()) - erasing its only copy
 * before rewriting it would lose every gate and the mag-cal outright if
 * a brownout/reset landed mid-rewrite. Kept out of the FLASH memory
 * region entirely (rather than just documented) so the linker can never
 * place a section there.
 *
 * RAM   = SRAM1 (16 KB) + SRAM2 (6 KB), contiguous at 0x20000000.
 * CCMRAM (10 KB at 0x10000000) holds task stacks, hot fusion state and the
 * RAM-executed flash program/erase wrapper functions (.ccmfunc). NOTE:
 * this does NOT make the flash write/erase itself stall-free - the HAL
 * routines these wrappers call (HAL_FLASH_Program, HAL_FLASHEx_Erase)
 * still execute from FLASH, so the CPU fetch unit still stalls for the
 * duration of the operation regardless of where the wrapper lives; see
 * the comment on flash_store_program_record() in flash_store.c for why
 * this is accepted rather than fixed.
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size = 0x0;      /* fully static allocation - no heap */
_Min_Stack_Size = 0x400;   /* MSP: startup + ISRs */

MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 124K
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 22K
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 10K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM (READONLY) :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Initialised data: load in FLASH, run in RAM */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* CCM: RAM-executed flash routines + hot data + task stacks.
     Copied/zeroed by startup via the _siccm/_sccm symbols. */
  _siccm = LOADADDR(.ccm_data);
  .ccm_data :
  {
    . = ALIGN(4);
    _sccm = .;
    *(.ccmfunc)
    *(.ccmfunc*)
    *(.ccm_data)
    *(.ccm_data*)
    . = ALIGN(4);
    _eccm = .;
  } >CCMRAM AT> FLASH

  .ccm_bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sccmbss = .;
    *(.ccm_bss)
    *(.ccm_bss*)
    . = ALIGN(4);
    _eccmbss = .;
  } >CCMRAM

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
