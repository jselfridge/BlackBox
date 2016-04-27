

#ifndef TIMER_H
#define TIMER_H


#include <pthread.h>
#include <sys/types.h>


#define PRIO_IMU    96
#define PRIO_AHRS   94
#define PRIO_IO     92
#define PRIO_CTRL   90
#define PRIO_GPS    88
#define PRIO_GCSTX  86
#define PRIO_GCSRX  86
#define PRIO_FLAG   84
#define PRIO_DEBUG  82

#define HZ_IMU_FAST    250
#define HZ_IMU_SLOW     50
#define HZ_IO           50
#define HZ_AHRS         50
#define HZ_CTRL         50
#define HZ_FLAG         10
#define HZ_GPS          10
#define HZ_GCSTX        10
#define HZ_GCSRX        10
#define HZ_DEBUG        10


pthread_mutex_t mutex_input;
pthread_mutex_t mutex_output;
pthread_mutex_t mutex_i2c1;
pthread_mutex_t mutex_i2c2;
pthread_mutex_t mutex_gyrA;
pthread_mutex_t mutex_accA;
pthread_mutex_t mutex_magA;
pthread_mutex_t mutex_gyrB;
pthread_mutex_t mutex_accB;
pthread_mutex_t mutex_magB;
pthread_mutex_t mutex_ahrs;
pthread_mutex_t mutex_eul;
pthread_mutex_t mutex_lpfeul;
pthread_mutex_t mutex_quat;
pthread_mutex_t mutex_gps;
pthread_mutex_t mutex_gcs;
pthread_mutex_t mutex_ctrl;


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


timer_struct tmr_io;
timer_struct tmr_flag;
timer_struct tmr_imu;
timer_struct tmr_imuA;
timer_struct tmr_imuB;
timer_struct tmr_ahrs;
timer_struct tmr_gps;
timer_struct tmr_gcstx;
timer_struct tmr_gcsrx;
timer_struct tmr_ctrl;
timer_struct tmr_debug;


void  tmr_init   ( void );
void  tmr_mutex  ( void );
void  tmr_setup  ( void );
void  tmr_attr   ( pthread_attr_t *attr );
void  tmr_begin  ( pthread_attr_t *attr );
void  tmr_exit   ( void );

void  tmr_thread ( timer_struct *tmr, pthread_attr_t *attr, void *fcn );
void  tmr_create ( timer_struct *tmr );
void  tmr_pause  ( timer_struct *tmr );
void  tmr_start  ( timer_struct *tmr );
void  tmr_finish ( timer_struct *tmr );

void *fcn_io     (  );
void *fcn_flag   (  );
void *fcn_imu    (  );
void *fcn_imuA   (  );
void *fcn_imuB   (  );
void *fcn_ahrs   (  );
void *fcn_gps    (  );
void *fcn_gcstx  (  );
void *fcn_gcsrx  (  );
void *fcn_ctrl   (  );
void *fcn_debug  (  );


#endif



