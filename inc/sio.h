
//============================================================
//  sio.h
//  Justin M Selfridge
//============================================================
#ifndef _SIO_H_
#define _SIO_H_
#include <main.h>


// Define disarm values (move to flags code?)
#define SIO_OFF0  1000
#define SIO_OFF1  1200
#define SIO_OFF2  1400
#define SIO_OFF3  1600
#define SIO_OFF4  1000
#define SIO_OFF5  0000
#define SIO_OFF6  1000
#define SIO_OFF7  1000
#define SIO_OFF8  1000
#define SIO_OFF9  1000


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
  CH_S = CH5,
  CH_D = CH6,
  CH_A = CH7
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
ushort off [10]; // Move to flags code?


// System input/output functions
void    sio_init    ( void );
void    sio_exit    ( void );
void    sio_update  ( void );
void    sio_setreg  ( ushort ch, ushort reg  );
void    sio_setpwm  ( ushort ch, ushort pwm  );
void    sio_setnorm ( ushort ch, double norm );
double  sio_norm    ( ushort reg, char dir   );
void    sio_disarm  ( void );


#endif



