
uart.elf:     file format elf32-littleriscv


Disassembly of section .text:

40000000 <_start>:
40000000:	00010117          	auipc	sp,0x10
40000004:	00010113          	mv	sp,sp
40000008:	00000297          	auipc	t0,0x0
4000000c:	0be28293          	addi	t0,t0,190 # 400000c6 <__bss_end>
40000010:	00000317          	auipc	t1,0x0
40000014:	0b630313          	addi	t1,t1,182 # 400000c6 <__bss_end>
40000018:	0062f763          	bgeu	t0,t1,40000026 <_start+0x26>
4000001c:	0002a023          	sw	zero,0(t0)
40000020:	0291                	addi	t0,t0,4
40000022:	fe62ede3          	bltu	t0,t1,4000001c <_start+0x1c>
40000026:	20ad                	jal	40000090 <main>

40000028 <halt>:
40000028:	a001                	j	40000028 <halt>

4000002a <uart_putchar>:
4000002a:	1101                	addi	sp,sp,-32 # 4000ffe0 <__bss_end+0xff1a>
4000002c:	ce06                	sw	ra,28(sp)
4000002e:	cc22                	sw	s0,24(sp)
40000030:	1000                	addi	s0,sp,32
40000032:	87aa                	mv	a5,a0
40000034:	fef407a3          	sb	a5,-17(s0)
40000038:	0001                	nop
4000003a:	400017b7          	lui	a5,0x40001
4000003e:	0791                	addi	a5,a5,4 # 40001004 <__bss_end+0xf3e>
40000040:	439c                	lw	a5,0(a5)
40000042:	dfe5                	beqz	a5,4000003a <uart_putchar+0x10>
40000044:	400017b7          	lui	a5,0x40001
40000048:	fef44703          	lbu	a4,-17(s0)
4000004c:	c398                	sw	a4,0(a5)
4000004e:	0001                	nop
40000050:	40f2                	lw	ra,28(sp)
40000052:	4462                	lw	s0,24(sp)
40000054:	6105                	addi	sp,sp,32
40000056:	8082                	ret

40000058 <uart_print>:
40000058:	1101                	addi	sp,sp,-32
4000005a:	ce06                	sw	ra,28(sp)
4000005c:	cc22                	sw	s0,24(sp)
4000005e:	1000                	addi	s0,sp,32
40000060:	fea42623          	sw	a0,-20(s0)
40000064:	a819                	j	4000007a <uart_print+0x22>
40000066:	fec42783          	lw	a5,-20(s0)
4000006a:	00178713          	addi	a4,a5,1 # 40001001 <__bss_end+0xf3b>
4000006e:	fee42623          	sw	a4,-20(s0)
40000072:	0007c783          	lbu	a5,0(a5)
40000076:	853e                	mv	a0,a5
40000078:	3f4d                	jal	4000002a <uart_putchar>
4000007a:	fec42783          	lw	a5,-20(s0)
4000007e:	0007c783          	lbu	a5,0(a5)
40000082:	f3f5                	bnez	a5,40000066 <uart_print+0xe>
40000084:	0001                	nop
40000086:	0001                	nop
40000088:	40f2                	lw	ra,28(sp)
4000008a:	4462                	lw	s0,24(sp)
4000008c:	6105                	addi	sp,sp,32
4000008e:	8082                	ret

40000090 <main>:
40000090:	1141                	addi	sp,sp,-16
40000092:	c606                	sw	ra,12(sp)
40000094:	c422                	sw	s0,8(sp)
40000096:	0800                	addi	s0,sp,16
40000098:	400007b7          	lui	a5,0x40000
4000009c:	0a478513          	addi	a0,a5,164 # 400000a4 <main+0x14>
400000a0:	3f65                	jal	40000058 <uart_print>
400000a2:	a001                	j	400000a2 <main+0x12>
