

#ifndef PID_H
#define PID_H


#include <pthread.h>


#define PID_XXX  0.0


pthread_mutex_t mutex_pid;


typedef struct pid_struct {
  double P;
  double I;
  double D;
} pid_struct;


pid_struct pid_xxx;


void  pid_init    ( void );
void  pid_exit    ( void );
void  pid_setp    ( pid_struct *pid, double P );
void  pid_seti    ( pid_struct *pid, double I );
void  pid_setd    ( pid_struct *pid, double D );
void  pid_update  ( pid_struct *pid );


#endif



