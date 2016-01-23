
//============================================================
//  sio.h
//  Justin M Selfridge
//============================================================
#ifndef _SIO_H_
#define _SIO_H_
#include <main.h>


// PRU driver include (shouldn't need this here)
//#include <prussdrv.h>
//#include <pruss_intc_mapping.h>


// Input (radio) variables
//#define IN_OFFSET   2049
//#define IN_CH       10
//#define IN_MIN      1000
//#define IN_MID      1500
//#define IN_MAX      2000


// Servo variables
//#define OUT_OFFSET  2060
//#define OUT_CH      10
//#define OUT_MIN     1000
//#define OUT_MAX     2000

// System input and output structure
typedef struct sio_struct {
  ushort pwm_in  [10];
  ushort pwm_out [10];
  double norm_in   [10];
  double norm_out  [10];
} sio_struct;
sio_struct sio;


// Global variables
//uint* memoryPtr;


// PRU functions
//void  pru_init       ( void );
//void  pru_exit       ( void );
//float pru_read_pulse ( int ch );
//float pru_read_norm  ( int ch );
//void  pru_send_pulse ( int ch, int pwm );
//void  pru_send_norm  ( int ch, float norm );


#endif



