

#ifndef IO_H
#define IO_H


#include <sys/types.h>


#define IN_CH       10
#define IN_OFFSET   2049
#define IN_REG2PWM  (30.0/200.0)
#define IN_PWM2REG  (200.0/30.0)
#define IN_MIN      ( 1000 * IN_PWM2REG )
#define IN_MAX      ( 2000 * IN_PWM2REG )

#define OUT_CH      10
#define OUT_OFFSET  2060
#define OUT_REG2PWM (23.0/200.0)
#define OUT_PWM2REG (200.0/23.0)
#define OUT_MIN     ( 1000 * OUT_PWM2REG )
#define OUT_MAX     ( 2000 * OUT_PWM2REG )


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

enum radio_index {
  CH_R = CH1,
  CH_P = CH2,
  CH_Y = CH3,
  CH_T = CH4,
  CH_S = CH5,
  CH_D = CH6,
  CH_A = CH7
} radio_index;


typedef struct io_struct {
  ushort reg  [10];
  ushort pwm  [10];
  double norm [10];
  pthread_mutex_t mutex;
} io_struct;
io_struct input;
io_struct output;


uint *memoryPtr;


void  io_init    ( void );
void  io_exit    ( void );
void  io_update  ( void );
void  io_setreg  ( ushort ch, ushort reg  );
void  io_setpwm  ( ushort ch, ushort pwm  );
void  io_setnorm ( ushort ch, double norm );


#endif



