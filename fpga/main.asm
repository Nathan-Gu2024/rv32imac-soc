
main.elf:     file format elf32-littleriscv


Disassembly of section .text:

40000000 <_start>:
40000000:	00010117          	auipc	sp,0x10
40000004:	00010113          	mv	sp,sp
40000008:	00000297          	auipc	t0,0x0
4000000c:	0b028293          	addi	t0,t0,176 # 400000b8 <__bss_end>
40000010:	00000317          	auipc	t1,0x0
40000014:	0a830313          	addi	t1,t1,168 # 400000b8 <__bss_end>
40000018:	0062f763          	bgeu	t0,t1,40000026 <_start+0x26>
4000001c:	0002a023          	sw	zero,0(t0)
40000020:	0291                	addi	t0,t0,4
40000022:	fe62ede3          	bltu	t0,t1,4000001c <_start+0x1c>
40000026:	2825                	jal	4000005e <main>

40000028 <halt>:
40000028:	a001                	j	40000028 <halt>

4000002a <delay>:
4000002a:	7179                	addi	sp,sp,-48
4000002c:	d606                	sw	ra,44(sp)
4000002e:	d422                	sw	s0,40(sp)
40000030:	1800                	addi	s0,sp,48
40000032:	fca42e23          	sw	a0,-36(s0)
40000036:	fe042623          	sw	zero,-20(s0)
4000003a:	a031                	j	40000046 <delay+0x1c>
4000003c:	fec42783          	lw	a5,-20(s0)
40000040:	0785                	addi	a5,a5,1
40000042:	fef42623          	sw	a5,-20(s0)
40000046:	fec42783          	lw	a5,-20(s0)
4000004a:	fdc42703          	lw	a4,-36(s0)
4000004e:	fee7e7e3          	bltu	a5,a4,4000003c <delay+0x12>
40000052:	0001                	nop
40000054:	0001                	nop
40000056:	50b2                	lw	ra,44(sp)
40000058:	5422                	lw	s0,40(sp)
4000005a:	6145                	addi	sp,sp,48
4000005c:	8082                	ret

4000005e <main>:
4000005e:	1101                	addi	sp,sp,-32 # 4000ffe0 <__bss_end+0xff28>
40000060:	ce06                	sw	ra,28(sp)
40000062:	cc22                	sw	s0,24(sp)
40000064:	1000                	addi	s0,sp,32
40000066:	fe042623          	sw	zero,-20(s0)
4000006a:	4785                	li	a5,1
4000006c:	fef42423          	sw	a5,-24(s0)
40000070:	6789                	lui	a5,0x2
40000072:	fec42703          	lw	a4,-20(s0)
40000076:	c398                	sw	a4,0(a5)
40000078:	fec42703          	lw	a4,-20(s0)
4000007c:	fe842783          	lw	a5,-24(s0)
40000080:	97ba                	add	a5,a5,a4
40000082:	fef42223          	sw	a5,-28(s0)
40000086:	fe842783          	lw	a5,-24(s0)
4000008a:	fef42623          	sw	a5,-20(s0)
4000008e:	fe442783          	lw	a5,-28(s0)
40000092:	fef42423          	sw	a5,-24(s0)
40000096:	001e87b7          	lui	a5,0x1e8
4000009a:	48078513          	addi	a0,a5,1152 # 1e8480 <_start-0x3fe17b80>
4000009e:	3771                	jal	4000002a <delay>
400000a0:	fec42703          	lw	a4,-20(s0)
400000a4:	08000793          	li	a5,128
400000a8:	fce7f4e3          	bgeu	a5,a4,40000070 <main+0x12>
400000ac:	fe042623          	sw	zero,-20(s0)
400000b0:	4785                	li	a5,1
400000b2:	fef42423          	sw	a5,-24(s0)
400000b6:	bf6d                	j	40000070 <main+0x12>
