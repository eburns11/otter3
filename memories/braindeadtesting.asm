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

addi x7, zero, 7

lui  x8,0x10 #(change this address to a Data memory address in your Otter)

addi  x10,zero,10

nop

nop

or x11,x7,x10

nop

nop

sw x11,0(x8)

.data
bruh: .word 0