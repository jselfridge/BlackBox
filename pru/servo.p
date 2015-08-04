
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  servo.p
//  Justin M Selfridge
//  Sends servo PWM outputs on the PRU0 subsystem.
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
*  r30.t(#-1)    Bits for output signal (index: 0-7)
*  r0            Counter for total PWM cycles
*  r(#)          Counters for the pulse of individual channels
**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**
*  OUTER_LOOP:                          outer loop repeats at total PWM frequency
*    LBCO    r0, MEM, 0, 40             get the PWM cycles
*    MOV     r30.b0, 0xFF               set all outputs high
*  INNER_LOOP:                          inner loop monitors PWM pulse duration
*    SUB     r0, r0, 1                  decrement outer loop counter
*    SUB     r1, r1, 1                  decrement individual channel counter
*        ||                             repeat for other channels
*  SKIP(#):                             assess a new channel
*    QBNE    SKIP(#+1), r(#+1), 0       check counter, if not zero, skip to next channel
*    CLR     r30.t(#)                   else, turn off the output
*        ||                             repeat for other channels
*  SKIP8:                               asses final loop
*    QBEQ    OUTER_LOOP, r0, 0          check total counter, if zero, start another outer loop
*    QBA     INNER_LOOP                 else, go back for another inner loop
**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

START:
    LBCO    r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4
    SBCO    r0, CONST_PRUCFG, 4, 4
    MOV     r0, 0x120
    MOV     r1, PRU0_CTRL + CTPPR0
    SBBO    r0, r1, 0, 4

OUTER_LOOP:
       LBCO    r0, CONST_PRUSHAREDRAM, 40, 80
    MOV     r30.b0, 0xFF

INNER_LOOP:
    SUB     r0, r0, 1
    SUB     r1, r1, 1
    SUB     r2, r2, 1
    SUB     r3, r3, 1
    SUB     r4, r4, 1
    SUB     r5, r5, 1
    SUB     r6, r6, 1
    SUB     r7, r7, 1
    SUB     r8, r8, 1

    QBNE    SKIP1, r1, 0
    CLR     r30.t0

SKIP1:
    QBNE    SKIP2, r2, 0
    CLR     r30.t1

SKIP2: 
    QBNE    SKIP3, r3, 0
    CLR     r30.t2

SKIP3:
    QBNE    SKIP4, r4, 0
    CLR     r30.t3

SKIP4:
    QBNE    SKIP5, r5, 0
    CLR     r30.t4

SKIP5:
    QBNE    SKIP6, r6, 0
    CLR     r30.t5

SKIP6:
    QBNE    SKIP7, r7, 0
    CLR     r30.t6

SKIP7:
    QBNE    SKIP8, r8, 0
    CLR     r30.t7

SKIP8:
    QBEQ    OUTER_LOOP, r0, 0
    QBA     INNER_LOOP



