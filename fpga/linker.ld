/* linker.ld */ 
MEMORY
{
    /* TCM Base: 0x4000_0000, Size: 65536 bytes (64KB) */ 
    TCM (rwx) : ORIGIN = 0x40000000, LENGTH = 64K
}

ENTRY(_start)

SECTIONS
{
.text : { 
        *(.text.init) 
        *(.text .text.*)
    } > TCM
    
    .rodata : {
        *(.rodata .rodata.*)
    } > TCM

    .data : {
        *(.data .data*)
    } > TCM

    .bss : {
        PROVIDE(__bss_start = .);
        *(.bss .bss.*);
        *(COMMON)
        PROVIDE(__bss_end = .);
    } > TCM

    PROVIDE(__stack_top = ORIGIN(TCM) + LENGTH(TCM)); 
}