
//============================================================
//  sio.h
//  Justin M Selfridge
//============================================================
#ifndef _SIO_H_
#define _SIO_H_
#include <main.h>


// PRU driver include (shouldn't need this here if it's in main.h)
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


// Input signal variables
#define IN_OFFSET   2049
#define IN_CH       10
#define IN_MIN      6667  // ( 1000 pwm ) * (200/30)
#define IN_MAX      13333 // ( 2000 pwm ) * (200/30)


// Output signal variables
#define OUT_OFFSET  2060
#define OUT_CH      10
#define OUT_MIN     8695  // ( 1000 pwm ) * (200/23)
#define OUT_MAX     17390 // ( 2000 pwm ) * (200/23)


// Signal structure
typedef struct signal_struct {
  ushort reg  [10];
  ushort pwm  [10];
  double norm [10];
} signal_struct;
signal_struct input;
signal_struct output;

// System input and output structure
//typedef struct sio_struct {
  //signal_struct input;
  //signal_struct output;
//} sio_struct;
//sio_struct sio;


// Global variables
uint* memoryPtr;


// System input/output functions
void    sio_init    ( void );
void    sio_exit    ( void );
void    sio_update  ( void );
void    sio_setreg  ( ushort ch, ushort reg );
void    sio_setpwm  ( ushort ch, ushort pwm );
void    sio_setnorm ( ushort ch, float norm );

//void    sio_setreg  ( ushort ch, ushort reg );
//void    sio_output  ( void );
//void  sio_getpwm
//void  sio_setpwm
//void  sio_inputs
//void  sio_outputs
//void  sio_getnorm
//void  sio_setnorm
//void  sio_update    // Gets all new data inputs and sends all new data outputs


//float pru_read_norm  ( int ch );
//void  pru_send_pulse ( int ch, int pwm );
//void  pru_send_norm  ( int ch, float norm );


#endif



