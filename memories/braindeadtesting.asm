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
#nop

#addi	x14,x10,4
#addi	x1,x2,-1

addi x7, zero, 7

lui  x8,0x10 #(change this address to a Data memory address in your Otter)

addi  x10,zero,10

nop

nop

or x11,x7,x10

nop

nop

sw x11,0(x8)