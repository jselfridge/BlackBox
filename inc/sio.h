
//============================================================
//  sio.h
//  Justin M Selfridge
//============================================================
#ifndef _SIO_H_
#define _SIO_H_
#include <main.h>

/*
// Input signal variables
#define IN_CH       10
#define IN_OFFSET   2049
#define IN_REG2PWM  (30.0/200.0)
#define IN_PWM2REG  (200.0/30.0)
#define IN_MIN      ( 1000 * IN_PWM2REG )
#define IN_MAX      ( 2000 * IN_PWM2REG )


// Output signal variables
#define OUT_CH      10
#define OUT_OFFSET  2060
#define OUT_REG2PWM (23.0/200.0)
#define OUT_PWM2REG (200.0/23.0)
#define OUT_MIN     ( 1000 * OUT_PWM2REG )
#define OUT_MAX     ( 2000 * OUT_PWM2REG )

/*
// Channel enumerations
enum ch_index {
  CH1 = 0,
  CH2 = 1,
  CH3 = 2,
  CH4 = 3,
  CH5 = 4,
  CH6 = 5,
  CH7 = 6,
  CH8 = 7,
  CH9 = 8,
  CH0 = 9
} ch_index;


// Radio enumerations
enum radio_index {
  CH_R = CH1,
  CH_P = CH2,
  CH_Y = CH3,
  CH_T = CH4,
  CH_S = CH6,
  CH_D = CH7,
  CH_A = CH8
} radio_index;


// Signal structure
typedef struct signal_struct {
  ushort reg  [10];
  ushort pwm  [10];
  double norm [10];
} signal_struct;
signal_struct input;
signal_struct output;


// Global variables (remove as quickly as possible)
uint* memoryPtr;
*/

// System input/output functions
void    sio_init    ( void );
void    sio_exit    ( void );
//void    sio_update  ( void );
//void    sio_setreg  ( ushort ch, ushort reg  );
//void    sio_setpwm  ( ushort ch, ushort pwm  );
//void    sio_setnorm ( ushort ch, double norm );
//double  sio_norm    ( ushort reg, char dir   );


#endif



