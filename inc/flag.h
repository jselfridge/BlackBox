

#ifndef FLAG_H
#define FLAG_H


#include <stdbool.h>
#include <sys/types.h>


#define FLAG_HOLD_R  3.0
#define FLAG_HOLD_P  3.0
#define FLAG_HOLD_Y  3.0
#define FLAG_HOLD_T  0.0


typedef struct flag_struct {
  ushort upper[4];
  ushort lower[4];
  ushort limit[4];
} flag_struct;
flag_struct flag;


bool running;
bool armed;


void  flag_init   ( void );
void  flag_exit   ( void );
//void  flag_update ( void );


#endif



