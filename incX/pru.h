
//============================================================
//  pru.h
//  Justin M Selfridge
//============================================================
#ifndef _PRU_H_
#define _PRU_H_
#include <uav.h>


// PRU driver includes
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


// Input (radio) variables
#define IN_OFFSET   2049 
#define IN_CH       8
#define IN_MIN      1000
#define IN_MID      1500
#define IN_MAX      2000


// Servo variables
#define OUT_OFFSET  2059
#define OUT_CH      8
#define OUT_MIN     1000
#define OUT_MAX     2000


// PRU functions
void  pru_init       ( void );
void  pru_exit       ( void );
float pru_read_pulse ( int ch );
float pru_read_norm  ( int ch );
void  pru_send_pulse ( int ch, int pwm );
void  pru_send_norm  ( int ch, float norm );


#endif



