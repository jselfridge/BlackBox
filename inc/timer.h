
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
#define HZ_GYR     500
#define HZ_ACC     500
#define HZ_MAG     100
#define HZ_DEBUG    10


// Timer structure
typedef struct timer_struct {
  char      *name;
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


// Mutex declarations
pthread_mutex_t gyr_mutex;
pthread_mutex_t acc_mutex;
pthread_mutex_t mag_mutex;


// Thread functions
void  tmr_init      ( void );
void  tmr_setup     ( void );
void  tmr_attr      ( pthread_attr_t *attr );
void  tmr_thread    ( timer_struct *tmr, pthread_attr_t *attr, void *fcn );
void  tmr_exit      ( void );
void  tmr_create    ( timer_struct *tmr );
void  tmr_pause     ( timer_struct *tmr );
void  tmr_start     ( timer_struct *tmr );
void  tmr_finish    ( timer_struct *tmr );


// Function handlers
void *fcn_gyr    (  );
void *fcn_acc    (  );
void *fcn_mag    (  );
void *fcn_debug  (  );




#endif



