
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  radio.p
//  Justin M Selfridge
//  Reads radio PWM input on the PRU1 subsystem.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.origin 0
.entrypoint START

#define CONST_PRUCFG         C4
#define CONST_PRUSHAREDRAM   C28
#define PRU0_CTRL            0x22000
#define PRU1_CTRL            0x24000
#define CTPPR0               0x28

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**
*  The '#' represents a channel number (index: 1-8)
*  r31.t(#-1)    Bits for current input reading (index: 0-7)
*  r0.t(#-1)     Bits for previous input reading (index: 0-7)
*  r(#)          PWM counters for each input channel
*  r9            Dummy register used to sync the timing
**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**
*  READ(#):                        read this input channel
*    QBBC  LOW(#), r31.t(#-1)      check current input, if low, jump to LOW
*    ADD   r(#), r(#), 1           else, always increment counter
*    QBBS  READ(#+1), r0.t(#-1)    check prev input, if also high, READ the next channel
*    SET   r0.t(#-1)               else, prev input was low, so set the flag high
*    QBA   READ(#+1)               and finally, READ the next channel
*  LOW(#):                         alternatively, the current input value is low
*    MOV   r9, 0                   do nothing, this command is for proper timing
*    QBBC  READ(#+1), r0.t(#-1)    check prev, if also low, READ the next channel
*    SBCO  r(#), MEM, OFFSET, 4    else, prev input was high, so set shared memory
*    CLR   r0.t(#-1)               and, set prev flag to low
*    MOV   r(#), 0                 and, clear PWM counter
*    QBA   READ(#+1)               and finally, READ next channel
**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

START:
    LBCO    r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4
    SBCO    r0, CONST_PRUCFG, 4, 4
    MOV     r0, 0x120
    MOV     r1, PRU1_CTRL + CTPPR0
    SBBO    r0, r1, 0, 4

    MOV     r1, 0
    MOV     r2, 0
    MOV     r3, 0
    MOV     r4, 0
    MOV     r5, 0
    MOV     r6, 0
    MOV     r7, 0
    MOV     r8, 0

    SBCO    r1, CONST_PRUSHAREDRAM,  4, 4
    SBCO    r2, CONST_PRUSHAREDRAM,  8, 4
    SBCO    r3, CONST_PRUSHAREDRAM, 12, 4
    SBCO    r4, CONST_PRUSHAREDRAM, 16, 4
    SBCO    r5, CONST_PRUSHAREDRAM, 20, 4
    SBCO    r6, CONST_PRUSHAREDRAM, 24, 4
    SBCO    r7, CONST_PRUSHAREDRAM, 28, 4
    SBCO    r8, CONST_PRUSHAREDRAM, 32, 4

READ1:
    QBBC    LOW1, r31.t0
    ADD     r1, r1, 1
    QBBS    READ2, r0.t0
    SET     r0.t0
    QBA     READ2

LOW1: 
    MOV     r9, 0
    QBBC    READ2, r0.t0
    SBCO    r1, CONST_PRUSHAREDRAM, 4, 4
    CLR     r0.t0
    MOV     r1, 0
    QBA     READ2

READ2:
    QBBC    LOW2, r31.t1
    ADD     r2, r2, 1
    QBBS    READ3, r0.t1
    SET     r0.t1
    QBA     READ3

LOW2:
    MOV     r9, 0
    QBBC    READ3, r0.t1
    SBCO    r2, CONST_PRUSHAREDRAM, 8, 4
    CLR     r0.t1
    MOV     r2, 0
    QBA     READ3

READ3:
    QBBC    LOW3, r31.t2
    ADD     r3, r3, 1
    QBBS    READ4, r0.t2
    SET     r0.t2
    QBA     READ4

LOW3:
    MOV     r9, 0
    QBBC    READ4, r0.t2
    SBCO    r3, CONST_PRUSHAREDRAM, 12, 4
    CLR     r0.t2
    MOV     r3, 0
    QBA     READ4

READ4:
    QBBC    LOW4, r31.t3
    ADD     r4, r4, 1
    QBBS    READ5, r0.t3
    SET     r0.t3
    QBA     READ5

LOW4:
    MOV     r9, 0
    QBBC    READ5, r0.t3
    SBCO    r4, CONST_PRUSHAREDRAM, 16, 4
    CLR     r0.t3
    MOV     r4, 0
    QBA     READ5

READ5:
    QBBC    LOW5, r31.t4
    ADD     r5, r5, 1
    QBBS    READ6, r0.t4
    SET     r0.t4
    QBA     READ6

LOW5:
    MOV     r9, 0
    QBBC    READ6, r0.t4
    SBCO    r5, CONST_PRUSHAREDRAM, 20, 4
    CLR     r0.t4
    MOV     r5, 0
    QBA     READ6

READ6:
    QBBC    LOW6, r31.t5
    ADD     r6, r6, 1
    QBBS    READ7, r0.t5
    SET     r0.t5
    QBA     READ7

LOW6:
    MOV     r9, 0
    QBBC    READ7, r0.t5
    SBCO    r6, CONST_PRUSHAREDRAM, 24, 4
    CLR     r0.t5
    MOV     r6, 0
    QBA     READ7

READ7:
    QBBC    LOW7, r31.t6
    ADD     r7, r7, 1
    QBBS    READ8, r0.t6
    SET     r0.t6
    QBA     READ8

LOW7:
    MOV     r9, 0
    QBBC    READ8, r0.t6
    SBCO    r7, CONST_PRUSHAREDRAM, 28, 4
    CLR     r0.t6
    MOV     r7, 0
    QBA     READ8

READ8:
    QBBC    LOW8, r31.t7
    ADD     r8, r8, 1
    QBBS    READ1, r0.t7
    SET     r0.t7
    QBA     READ1

LOW8:
    MOV     r9, 0
    QBBC    READ1, r0.t7
    SBCO    r8, CONST_PRUSHAREDRAM, 32, 4
    CLR     r0.t7
    MOV     r8, 0
    QBA     READ1

HALT



