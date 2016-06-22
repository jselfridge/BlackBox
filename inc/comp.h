

#ifndef COMP_H
#define COMP_H


#include <sys/types.h>


typedef struct comp_struct {
  double  dt;
  double  alpha;
  double  roll;
  double  pitch;
  pthread_mutex_t mutex;
} comp_struct;
comp_struct comp;


void  comp_init    ( void );
void  comp_exit    ( void );
void  comp_update  ( void );


#endif



