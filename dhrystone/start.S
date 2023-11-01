	.section .text
	.global start
	.global main

start:
	/* print "START\r\n" */
	li a0, UARTADDR
	li a1, 'S'
	li a2, 'T'
	li a3, 'A'
	li a4, 'R'
	li a5, '\r'
	li a6, '\n'
	sh a1, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a2, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a3, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a4, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a2, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a5, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a6, 4(a0)

	/* set stack pointer */
	li sp, STACK

        # store return address
        addi sp, sp, -4
        sw ra, (sp)

	/* jump to main C code */
	jal ra,main

	/* print "DONE\r\n" */
	li a0,UARTADDR
	addi a1,zero,'D'
	addi a2,zero,'O'
	addi a3,zero,'N'
	addi a4,zero,'E'
	addi a5,zero,'\r'
	addi a6,zero,'\n'

1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a1, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a2, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a3, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a4, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a5, 4(a0)
1:      lh t0, 4(a0)
        bnez t0, 1b

	sh a6, 4(a0)

        # return to monitor
        lw ra, (sp)
        ret

