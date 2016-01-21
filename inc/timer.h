
//============================================================
//  timer.h
//  Justin M Selfridge
//============================================================
#ifndef _TIMER_H_
#define _TIMER_H_
#include <main.h>


// Define timer priorities
#define PRIO_GYR    96
#define PRIO_ACC    96
#define PRIO_MAG    94
#define PRIO_DEBUG  80


// Define timer frequencies
#define HZ_GYR    200
#define HZ_ACC    100
#define HZ_MAG     50
#define HZ_DEBUG   10


// Timer structure
typedef struct timer_struct {
  pthread_t id;
  int       fd;
  ushort    prio;
  ulong     per;
  uint      start_sec;
  ulong     start_usec;
  uint      finish_sec;
  ulong     finish_usec;
  ulong     dur;
} timer_struct;


// Timer instances
timer_struct tmr_gyr;
timer_struct tmr_acc;
timer_struct tmr_mag;
timer_struct tmr_debug;


// Mutex variables
//pthread_mutex_t mutex_cal;
//pthread_mutex_t mutex_fusion;


// Thread functions
void  tmr_init      ( void );
void  tmr_setup     ( void );
void  tmr_attr      ( pthread_attr_t *attr );
//void  tmr_periodic  ( thread_struct *thr );
//void  tmr_pause     ( thread_struct *thr );
//void  tmr_start     ( thread_struct *thr );
//void  tmr_finish    ( thread_struct *thr );
//void  tmr_exit      ( void );


// Thread handlers
//void *thread_imu    ( );
//void *thread_fusion  ( );
//void *thread_debug   ( );


#endif



