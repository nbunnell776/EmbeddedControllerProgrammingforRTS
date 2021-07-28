	.syntax unified
	.text
	.global strcopy
	.type strcopy, %function

strcopy:
	LDRB R2, [R1], #1
    STRB R2, [R0], #1
    CMP  R2, #0
    BNE  strcopy
    BX   lr
