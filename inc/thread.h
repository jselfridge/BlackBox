
//============================================================
//  thread.h
//  Justin M Selfridge
//============================================================
#ifndef _THREAD_H_
#define _THREAD_H_
#include <main.h>


// Define statements
#define HZ_IMU     300
#define HZ_ATT     100
//#define HZ_STAB     50
//#define HZ_NAV      10
//#define HZ_TELEM    10
#define HZ_DEBUG    10
//#define HZ_FLAGS     2
//#define HZ_HEART     1


// Thread structure
typedef struct thread_struct {
  pthread_t id;
  int       fd;
  ushort    priority;
  ulong     period;
  uint      start_sec;
  ulong     start_usec;
  uint      finish_sec;
  ulong     finish_usec;
  ulong     dur;
} thread_struct;

// Thread instances
thread_struct thr_imu;
//thread_struct thr_fusion;
//thread_struct thr_sysio;
//thread_struct thr_ctrl;
//thread_struct thr_telem;
thread_struct thr_debug;


// Mutex variables
//pthread_mutex_t mutex_imu;
//pthread_mutex_t mutex_fusion;
//pthread_mutex_t mutex_sysio;


// Thread functions
void  thr_init      ( void );
void  thr_periodic  ( thread_struct *thr );
void  thr_pause     ( thread_struct *thr );
void  thr_start     ( thread_struct *thr );
void  thr_finish    ( thread_struct *thr );
void  thr_exit      ( void );


// Thread handlers
void *thread_imu    ( );
//void *thread_fusion ( );
//void *thread_sysio  ( );
//void *thread_ctrl   ( );
//void *thread_telem  ( );
void *thread_debug  ( );


#endif



