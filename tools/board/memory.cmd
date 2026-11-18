-uinterruptVectors
--stack_size=4096
--heap_size=4096

/* Main flash only: this build never supplies device security configuration. */
MEMORY
{
    FLASH (RX)  : origin = 0x00000000, length = 0x00020000
    SRAM  (RWX) : origin = 0x20200000, length = 0x00008000
}

SECTIONS
{
    .intvecs   : > 0x00000000
    .text      : palign(8) {} > FLASH
    .const     : palign(8) {} > FLASH
    .cinit     : palign(8) {} > FLASH
    .pinit     : palign(8) {} > FLASH
    .rodata    : palign(8) {} > FLASH
    .ARM.exidx : palign(8) {} > FLASH
    .init_array : palign(8) {} > FLASH
    .binit     : palign(8) {} > FLASH
    .TI.ramfunc : load = FLASH, palign(8), run = SRAM, table(BINIT)
    .vtable    : > SRAM
    .args      : > SRAM
    .data      : > SRAM
    .bss       : > SRAM
    .sysmem    : > SRAM
    .stack     : > SRAM (HIGH)
}
