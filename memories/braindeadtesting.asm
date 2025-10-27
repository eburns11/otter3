.text

#addi	x1,zero,1
#addi	x2,zero,2
#addi	x3,zero,3
#addi	x4,zero,4
#addi	x5,zero,5
#addi	x6,zero,6
#addi	x7,zero,7
#addi	x8,zero,8
#addi	x9,zero,9
#addi	x10,zero,10

#nop
#nop
#add	x11,x1,x10
#add	x12,x4,x8
#add	x13,x6,x7
#nop
#nop

#addi	x20,zero,0x80
#nop
#nop
#sw	x1,0(x20)
#sw	x2,4(x20)
#sw	x3,8(x20)
#sw	x4,12(x20)
#sw	x5,16(x20)
#lw	x5,0(x20)
#lw	x4,4(x20)
#lw	x3,8(x20)
#lw	x2,12(x20)
#lw	x1,16(x20)

begin:	addi	s0,zero,1
addi	s1,zero,2
add	s2,s0,s1
add	s3,s1,s2
add	s4,s2,s3
add	s5,s3,s4
addi	s6,zero,400
li	t1,0xDEADBEEF
sw	t1,0(s6)

lw	s7,0(s6)
addi	s8,s7,1

beq	zero,zero,begin

j	begin
