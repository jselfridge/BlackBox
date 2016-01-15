
//============================================================
//  loop.h
//  Justin M Selfridge
//============================================================
#ifndef _LOOP_H_
#define _LOOP_H_
#include <main.h>


// Define statements
//#define HZ_IMU     200
//#define HZ_ATT     100
//#define HZ_CTRL     50
//#define HZ_TELEM    20
#define HZ_DEBUG    10


// Loop structure
typedef struct loop_struct {
  pthread_t pid;
  timer_t   tid;  // NEW
  int       fd;
  ushort    priority;
  ulong     period;
  uint      start_sec;
  ulong     start_usec;
  uint      finish_sec;
  ulong     finish_usec;
  ulong     dur;
  struct sigevent sev;  // new
  struct sigaction sa;  // new
} loop_struct;


// Loop instances
//loop_struct thr_imu;
//loop_struct thr_fusion;
//loop_struct thr_sysio;
//loop_struct thr_ctrl;
//loop_struct thr_telem;
loop_struct XXX_debug;

/*
// Mutex variables
//pthread_mutex_t mutex_imu;
//pthread_mutex_t mutex_fusion;
//pthread_mutex_t mutex_sysio;
*/

// Loop functions
void  loop_init      ( void );
//void  thr_periodic  ( thread_struct *thr );
//void  thr_pause     ( thread_struct *thr );
//void  thr_start     ( thread_struct *thr );
//void  thr_finish    ( thread_struct *thr );
void  loop_exit      ( void );

/*
// Thread handlers
//void *thread_imu    ( );
//void *thread_fusion ( );
//void *thread_sysio  ( );
//void *thread_ctrl   ( );
//void *thread_telem  ( );
void *thread_debug  ( );
*/

#endif



