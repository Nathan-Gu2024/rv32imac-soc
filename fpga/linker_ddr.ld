MEMORY { DDR (rwx) : ORIGIN = 0x00000000, LENGTH = 512M }
ENTRY(_start)
SECTIONS {
    .text : { *(.text_init) *(.text .text.*) } > DDR
    .rodata : { *(.rodata .rodata.*) } > DDR
    .data : { *(.data .data*) } > DDR
    .bss : { PROVIDE(__bss_start = .); *(.bss .bss.*); *(COMMON); PROVIDE(__bss_end = .); } > DDR
    PROVIDE(__stack_top = ORIGIN(DDR) + LENGTH(DDR));
}
