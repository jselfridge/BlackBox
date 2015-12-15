
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
//thread_struct thr_comp;
thread_struct thr_debug;


// Thread functions
void  thr_init      ( void );
void  thr_periodic  ( thread_struct *thr );
void  thr_pause     ( thread_struct *thr );
void  thr_start     ( thread_struct *thr );
void  thr_finish    ( thread_struct *thr );
void  thr_exit      ( void );


// Thread handlers
void *thread_mems       ( );
//void *thread_comp       ( );
void *thread_debug     ( );


#endif



