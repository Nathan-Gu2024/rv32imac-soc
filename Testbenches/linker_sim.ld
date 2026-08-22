MEMORY { DDR (rwx) : ORIGIN = 0x00000000, LENGTH = 512M }
ENTRY(_start)
/* Simulation-only variant of fpga/linker/linker_ddr.ld: same .text base
 * (0x00100000, matching the TCM boot stub's jump target) but a much
 * smaller effective stack-top window (2MB instead of the real 512MB DDR
 * top) so the sim's mock DDR memory array can be small. Never used for
 * hardware builds. */
SECTIONS {
    .text 0x00100000 : { *(.text_init) *(.text .text.*) } > DDR
    .rodata : { *(.rodata .rodata.*) } > DDR
    .data : { *(.data .data*) *(.sdata .sdata.*) } > DDR
    .bss : { PROVIDE(__bss_start = .); *(.bss .bss.*) *(.sbss .sbss.*); *(COMMON); PROVIDE(__bss_end = .); } > DDR
    PROVIDE(__stack_top = 0x00200000);
}
