
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  input.p
//  Justin M Selfridge
//  Reads RC PWM inputs on the PRU0 subsystem.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.origin 0
.entrypoint START

#define CONST_PRUCFG         C4
#define CONST_PRUSHAREDRAM   C28
#define PRU0_CTRL            0x22000
#define PRU1_CTRL            0x24000
#define CTPPR0               0x28

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**
*  The (##) represents an input channel number (index: 01-10)
*  The (%%) represents a register address (values: 00-07,14,15)
*  r31.t(%%)     Bits for current input reading
*  r0.t(##)      Bits for previous input reading
*  r(##)         PWM counters for each input channel
*  r11           Dummy register used to sync the timing
**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**
*  READ(##):                        read this input channel
*    QBBC  LOW(##), r31.t(%%)       check current input, if low, jump to LOW
*    ADD   r(##), r(##), 1          else, always increment counter
*    QBBS  READ(##+1), r0.t(##)     check prev input, if also high, READ the next channel
*    SET   r0.t(##)                 else, prev input was low, so set the flag high
*    QBA   READ(##+1)               and finally, READ the next channel
*  LOW(##):                         alternatively, the current input value is low
*    MOV   r11, 0                   do nothing, this command is for proper timing
*    QBBC  READ(##+1), r0.t(##)     check prev, if also low, READ the next channel
*    SBCO  r(##), MEM, OFFSET, 4    else, prev input was high, so set shared memory
*    CLR   r0.t(##)                 and, set prev flag to low
*    MOV   r(##), 0                 and, clear PWM counter
*    QBA   READ(##+1)               and finally, READ next channel
**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**
*  Based on the ideal PCB wiring configuration, the index of
*  the register does not match the index of the input pins.
*  The folowing illustrates the mapping between the two. 
*  PIN:  01  02  03  04  05  06  07  08  09  10
*  REG:  07  05  01  00  06  04  02  03  15  14
**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

START:
    LBCO    r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4
    SBCO    r0, CONST_PRUCFG, 4, 4
    MOV     r0, 0x120
    MOV     r1, PRU0_CTRL + CTPPR0
    SBBO    r0, r1, 0, 4

    MOV     r1,  0
    MOV     r2,  0
    MOV     r3,  0
    MOV     r4,  0
    MOV     r5,  0
    MOV     r6,  0
    MOV     r7,  0
    MOV     r8,  0
    MOV     r9,  0
    MOV     r10, 0

    SBCO    r1,  CONST_PRUSHAREDRAM,  4, 4
    SBCO    r2,  CONST_PRUSHAREDRAM,  8, 4
    SBCO    r3,  CONST_PRUSHAREDRAM, 12, 4
    SBCO    r4,  CONST_PRUSHAREDRAM, 16, 4
    SBCO    r5,  CONST_PRUSHAREDRAM, 20, 4
    SBCO    r6,  CONST_PRUSHAREDRAM, 24, 4
    SBCO    r7,  CONST_PRUSHAREDRAM, 28, 4
    SBCO    r8,  CONST_PRUSHAREDRAM, 32, 4
    SBCO    r9,  CONST_PRUSHAREDRAM, 36, 4
    SBCO    r10, CONST_PRUSHAREDRAM, 40, 4

READ_01:
    QBBC    LOW_01, r31.t7
    ADD     r1, r1, 1
    QBBS    READ_02, r0.t1
    SET     r0.t1
    QBA     READ_02

LOW_01: 
    MOV     r11, 0
    QBBC    READ_02, r0.t1
    SBCO    r1, CONST_PRUSHAREDRAM, 4, 4
    CLR     r0.t1
    MOV     r1, 0
    QBA     READ_02

READ_02:
    QBBC    LOW_02, r31.t5
    ADD     r2, r2, 1
    QBBS    READ_03, r0.t2
    SET     r0.t2
    QBA     READ_03

LOW_02:
    MOV     r11, 0
    QBBC    READ_03, r0.t2
    SBCO    r2, CONST_PRUSHAREDRAM, 8, 4
    CLR     r0.t2
    MOV     r2, 0
    QBA     READ_03

READ_03:
    QBBC    LOW_03, r31.t1
    ADD     r3, r3, 1
    QBBS    READ_04, r0.t3
    SET     r0.t3
    QBA     READ_04

LOW_03:
    MOV     r11, 0
    QBBC    READ_04, r0.t3
    SBCO    r3, CONST_PRUSHAREDRAM, 12, 4
    CLR     r0.t3
    MOV     r3, 0
    QBA     READ_04

READ_04:
    QBBC    LOW_04, r31.t0
    ADD     r4, r4, 1
    QBBS    READ_05, r0.t4
    SET     r0.t4
    QBA     READ_05

LOW_04:
    MOV     r11, 0
    QBBC    READ_05, r0.t4
    SBCO    r4, CONST_PRUSHAREDRAM, 16, 4
    CLR     r0.t4
    MOV     r4, 0
    QBA     READ_05

READ_05:
    QBBC    LOW_05, r31.t6
    ADD     r5, r5, 1
    QBBS    READ_06, r0.t5
    SET     r0.t5
    QBA     READ_06

LOW_05:
    MOV     r11, 0
    QBBC    READ_06, r0.t5
    SBCO    r5, CONST_PRUSHAREDRAM, 20, 4
    CLR     r0.t5
    MOV     r5, 0
    QBA     READ_06

READ_06:
    QBBC    LOW_06, r31.t4
    ADD     r6, r6, 1
    QBBS    READ_07, r0.t6
    SET     r0.t6
    QBA     READ_07

LOW_06:
    MOV     r11, 0
    QBBC    READ_07, r0.t6
    SBCO    r6, CONST_PRUSHAREDRAM, 24, 4
    CLR     r0.t6
    MOV     r6, 0
    QBA     READ_07

READ_07:
    QBBC    LOW_07, r31.t2
    ADD     r7, r7, 1
    QBBS    READ_08, r0.t7
    SET     r0.t7
    QBA     READ_08

LOW_07:
    MOV     r11, 0
    QBBC    READ_08, r0.t7
    SBCO    r7, CONST_PRUSHAREDRAM, 28, 4
    CLR     r0.t7
    MOV     r7, 0
    QBA     READ_08

READ_08:
    QBBC    LOW_08, r31.t3
    ADD     r8, r8, 1
    QBBS    READ_09, r0.t8
    SET     r0.t8
    QBA     READ_09

LOW_08:
    MOV     r11, 0
    QBBC    READ_09, r0.t8
    SBCO    r8, CONST_PRUSHAREDRAM, 32, 4
    CLR     r0.t8
    MOV     r8, 0
    QBA     READ_09

READ_09:
    QBBC    LOW_09, r31.t15
    ADD     r9, r9, 1
    QBBS    READ_10, r0.t9
    SET     r0.t9
    QBA     READ_10

LOW_09:
    MOV     r11, 0
    QBBC    READ_10, r0.t9
    SBCO    r9, CONST_PRUSHAREDRAM, 36, 4
    CLR     r0.t9
    MOV     r9, 0
    QBA     READ_10

READ_10:
    QBBC    LOW_10, r31.t14
    ADD     r10, r10, 1
    QBBS    READ_01, r0.t10
    SET     r0.t10
    QBA     READ_01

LOW_10:
    MOV     r11, 0
    QBBC    READ_01, r0.t10
    SBCO    r10, CONST_PRUSHAREDRAM, 40, 4
    CLR     r0.t10
    MOV     r10, 0
    QBA     READ_01

HALT



