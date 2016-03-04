

#ifndef _TIMER_H_
#define _TIMER_H_
#include <main.h>


// Define timer priorities
#define PRIO_IMU    96
//#define PRIO_AHRS   94
#define PRIO_SIO    92
//#define PRIO_CTRL   90
//#define PRIO_GPS    88
//#define PRIO_GCSTX  86
//#define PRIO_GCSRX  86
//#define PRIO_UART1  88
//#define PRIO_UART2  88
//#define PRIO_UART4  88
//#define PRIO_UART5  88
#define PRIO_FLAG   84
#define PRIO_DEBUG  82


// Define timer frequencies
#define HZ_IMU_FAST   1000
#define HZ_IMU_SLOW    100
#define HZ_SIO          50
//#define HZ_AHRS         50
//#define HZ_CTRL         50
#define HZ_FLAG         10
//#define HZ_GPS          10
//#define HZ_GCSTX        10
//#define HZ_GCSRX        10
#define HZ_DEBUG        10
//#define HZ_UART1        10
//#define HZ_UART2        10
//#define HZ_UART4        10
//#define HZ_UART5        10


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
timer_struct tmr_sio;
timer_struct tmr_flag;
timer_struct tmr_imuA;
timer_struct tmr_imuB;
//timer_struct tmr_ahrs;
//timer_struct tmr_gps;
//timer_struct tmr_gcstx;
//timer_struct tmr_gcsrx;
//timer_struct tmr_uart1;
//timer_struct tmr_uart2;
//timer_struct tmr_uart4;
//timer_struct tmr_uart5;
//timer_struct tmr_ctrl;
timer_struct tmr_debug;


// Mutex declarations
pthread_mutex_t mutex_input;
pthread_mutex_t mutex_output;
pthread_mutex_t mutex_gyrA;
pthread_mutex_t mutex_accA;
pthread_mutex_t mutex_magA;
pthread_mutex_t mutex_gyrB;
pthread_mutex_t mutex_accB;
pthread_mutex_t mutex_magB;
//pthread_mutex_t mutex_quat;
//pthread_mutex_t mutex_eul;
//pthread_mutex_t mutex_ahrs;
//pthread_mutex_t mutex_gps;
//pthread_mutex_t mutex_gcs;  // Is this needed???
//pthread_mutex_t mutex_ctrl;


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
void *fcn_sio    (  );
void *fcn_flag   (  );
void *fcn_imuA   (  );
void *fcn_imuB   (  );
//void *fcn_ahrs   (  );
//void *fcn_gps    (  );
//void *fcn_gcstx  (  );
//void *fcn_gcsrx  (  );
//void *fcn_ctrl   (  );
void *fcn_debug  (  );

//void *fcn_uart1  (  );
//void *fcn_uart2  (  );
//void *fcn_uart4  (  );
//void *fcn_uart5  (  );

#endif



