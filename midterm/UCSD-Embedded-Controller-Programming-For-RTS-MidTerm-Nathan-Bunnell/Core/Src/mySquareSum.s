/*
	mySquareSum.s
	Function takes the input parameter of R0 and will loop from it
		down to 0, squaring subsequent each number into R1 then
		adding it to a running sum in R2 that is finally returned
		as R0

*/

	.syntax unified
	.text
	.global sumOfSquares
	.type sumOfSquares, %function

sumOfSquares:
	LDR R2, =0				// Ensure R2 is cleared as the function is entered

loop:
	MUL R1, R0, R0
	ADD R2, R1
	SUB R0, R0, #1
	CMP R0, #0
	BGT loop

return:
	MOV R0, R2
	BX lr
