
//============================================================
//  timer.h
//  Justin M Selfridge
//============================================================
#ifndef _TIMER_H_
#define _TIMER_H_
#include <main.h>


// Define timer priorities
#define PRIO_IMU    96
#define PRIO_AHR    92
#define PRIO_SIO    88
#define PRIO_FLAG   84
#define PRIO_CTRL   84
#define PRIO_DEBUG  80


// Define timer frequencies
#define HZ_IMU_FAST  1000
#define HZ_IMU_SLOW   100
#define HZ_AHR        100
#define HZ_SIO        100
#define HZ_CTRL       100
#define HZ_DEBUG       10
#define HZ_FLAG        10


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
  uint      dur;
} timer_struct;


// Timer declarations
timer_struct tmr_imu;
timer_struct tmr_ahr;
timer_struct tmr_sio;
timer_struct tmr_flag;
timer_struct tmr_ctrl;
timer_struct tmr_debug;


// Mutex declarations
pthread_mutex_t mutex_raw;
pthread_mutex_t mutex_avg;
pthread_mutex_t mutex_cal;
pthread_mutex_t mutex_quat;
pthread_mutex_t mutex_eul;
pthread_mutex_t mutex_input;
pthread_mutex_t mutex_output;
pthread_mutex_t mutex_ctrl;


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
void *fcn_imu    (  );
void *fcn_ahr    (  );
void *fcn_sio    (  );
void *fcn_flag   (  );
void *fcn_ctrl   (  );
void *fcn_debug  (  );


#endif



