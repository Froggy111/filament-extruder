/* ENTRY Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);    /* end of RAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400;     /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  FLASH    (rx)    : ORIGIN = 0x08000000, LENGTH = 512K
  DTCMRAM  (xrw)   : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1   (xrw)   : ORIGIN = 0x24000000, LENGTH = 320K
  RAM_D2   (xrw)   : ORIGIN = 0x30000000, LENGTH = 32K
  RAM_D3   (xrw)   : ORIGIN = 0x38000000, LENGTH = 16K
}

SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
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

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  /* Initialized data sections go into DTCM (Fastest access for initialized vars) */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .; 
    *(.data)
    *(.data*)

    . = ALIGN(4);
    _edata = .;
  } >DTCMRAM AT> FLASH

  /* 
   * .bss (Uninitialized data) -> MOVED TO RAM_D1 
   * This solves the overflow. RAM_D1 is 320KB, plenty of space.
   */
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
  } >RAM_D1

  /* User Heap/Stack check -> MOVED TO RAM_D1 to follow .bss */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* 
   * Ethernet Descriptors MUST go to RAM_D2.
   * We will configure MPU to make RAM_D2 UNCACHEABLE.
   * This prevents DMA coherency issues.
   */
  .RxDescripSection (NOLOAD) : { 
    . = ALIGN(4);
    *(.RxDescripSection) 
  } > RAM_D2
  
  .TxDescripSection (NOLOAD) : { 
    . = ALIGN(4);
    *(.TxDescripSection) 
  } > RAM_D2

  /* 
   * Ethernet Buffers/Pools can stay in RAM_D1.
   * The STM32HAL driver handles Cache Invalidation for buffers.
   */
  .Rx_PoolSection (NOLOAD) : { 
    . = ALIGN(32);
    *(.Rx_PoolSection) 
  } > RAM_D1

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
