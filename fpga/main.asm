
main.elf:     file format elf32-littleriscv


Disassembly of section .text:

40000000 <_start>:
40000000:	00010117          	auipc	sp,0x10
40000004:	00010113          	mv	sp,sp
40000008:	00000297          	auipc	t0,0x0
4000000c:	04428293          	addi	t0,t0,68 # 4000004c <__bss_end>
40000010:	00000317          	auipc	t1,0x0
40000014:	03c30313          	addi	t1,t1,60 # 4000004c <__bss_end>
40000018:	0062f763          	bgeu	t0,t1,40000026 <_start+0x26>
4000001c:	0002a023          	sw	zero,0(t0)
40000020:	0291                	addi	t0,t0,4
40000022:	fe62ede3          	bltu	t0,t1,4000001c <_start+0x1c>
40000026:	2011                	jal	4000002a <main>

40000028 <halt>:
40000028:	a001                	j	40000028 <halt>

4000002a <main>:
4000002a:	1101                	addi	sp,sp,-32 # 4000ffe0 <__bss_end+0xff94>
4000002c:	ce06                	sw	ra,28(sp)
4000002e:	cc22                	sw	s0,24(sp)
40000030:	1000                	addi	s0,sp,32
40000032:	4795                	li	a5,5
40000034:	fef42623          	sw	a5,-20(s0)
40000038:	6789                	lui	a5,0x2
4000003a:	fec42703          	lw	a4,-20(s0)
4000003e:	c398                	sw	a4,0(a5)
40000040:	4781                	li	a5,0
40000042:	853e                	mv	a0,a5
40000044:	40f2                	lw	ra,28(sp)
40000046:	4462                	lw	s0,24(sp)
40000048:	6105                	addi	sp,sp,32
4000004a:	8082                	ret
