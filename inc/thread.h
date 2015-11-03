
//============================================================
//  thread.h
//  Justin M Selfridge
//============================================================
#ifndef _THREAD_H_
#define _THREAD_H_
#include <main.h>


// Thread structure
typedef struct thread_struct {
  pthread_t id;
  int       fd;
  int       missed;
  ushort    priority;
  ushort    pin;
  ulong     period;
  uint      start_sec;
  ulong     start_usec;
  uint      finish_sec;
  ulong     finish_usec;
  ulong     dur;
  double    perc;
} thread_struct;
thread_struct thr_gyro; 
//thread_struct thr_stab; 
//thread_struct thr_nav; 
//thread_struct thr_telem; 


// Thread functions
void  thread_init      ( void );
void  thread_periodic  ( thread_struct *thr );
void  thread_pause     ( thread_struct *thr );
void  thread_start     ( thread_struct *thr );
void  thread_finish    ( thread_struct *thr );
void  thread_exit      ( void );
void *thread_gyro      ( );
//void *thread_stab      ( );
//void *thread_nav       ( );
//void *thread_telem     ( );


#endif



