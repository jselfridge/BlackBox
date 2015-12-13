
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
  ulong     period;
  uint      start_sec;
  ulong     start_usec;
  uint      finish_sec;
  ulong     finish_usec;
  ulong     dur;
  double    perc;
} thread_struct;
thread_struct thr_mems;
thread_struct thr_comp;
thread_struct thr_debug;


// Thread functions
void  thread_init      ( void );
void  thread_periodic  ( thread_struct *thr );
void  thread_pause     ( thread_struct *thr );
void  thread_start     ( thread_struct *thr );
void  thread_finish    ( thread_struct *thr );
void  thread_exit      ( void );


// Thread handlers
void *thread_mems       ( );
void *thread_comp       ( );
void *thread_debug     ( );


#endif



