MEMORY { DDR (rwx) : ORIGIN = 0x00000000, LENGTH = 512M }
ENTRY(_start)
/* Low 256KB (0x0-0x3FFFF) aliases with the Zynq-7000's Low OCM remap and
 * is not reliably reachable over the PL's AXI HP0 port (confirmed via
 * AXI Protocol Checker: reads/writes to this range can hang forever at
 * the bus level, deterministically, regardless of retry). Start program
 * placement comfortably above that boundary; __stack_top below still
 * uses the true DDR region so it lands at the real top of DDR. The TCM
 * boot stub's jump target must match this (see fpga/ddr.mem). */
SECTIONS {
    .text 0x00100000 : { *(.text_init) *(.text .text.*) } > DDR
    .rodata : { *(.rodata .rodata.*) } > DDR
    .data : { *(.data .data*) *(.sdata .sdata.*) } > DDR
    .bss : { PROVIDE(__bss_start = .); *(.bss .bss.*) *(.sbss .sbss.*); *(COMMON); PROVIDE(__bss_end = .); } > DDR
    PROVIDE(__stack_top = ORIGIN(DDR) + LENGTH(DDR));
}
