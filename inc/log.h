
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Define statements
#define MAX_LOG_DUR  (20*60)


// Log enumerations
enum log_index {
<<<<<<< HEAD:inc/log.X
  LOG_NOTE = 0,
  LOG_GYR  = 1,
  LOG_ACC  = 2,
  LOG_MAG  = 3,
  LOG_FUS  = 4,
  LOG_SIO  = 5,
  LOG_CTRL = 6
=======
  LOG_IMU  = 0,
  LOG_FUS  = 1
>>>>>>> altlog:inc/log.h
} log_index;


// Log IMU structure
typedef struct log_imu_struct {
  ulong count;
  ulong limit;
  float *time;
  uint  *dur;
  short *raw;
  float *avg;
  float *cal;
} log_imu_struct;
log_imu_struct log_gyr;
log_imu_struct log_acc;
log_imu_struct log_mag;


// Log structure
<<<<<<< HEAD:inc/log.X
typedef struct {
<<<<<<< HEAD:inc/log.X
  FILE* note;
  FILE* gyr;
  FILE* acc;
  FILE* mag;
  FILE* fus;
  FILE* sio;
  FILE* stab;
  FILE* nav;
  char* dir;
  char* path;
  bool  open;
  bool  enabled;
  long  offset;
} log_struct;
//log_struct datalog; 
=======
=======
typedef struct datalog_struct {
>>>>>>> altlog:inc/log.h
  bool   enabled;
  //bool   open;
  float  offset;
  char   *dir;
  char   *path;
<<<<<<< HEAD:inc/log.X
} log_struct;
log_struct datalog;
>>>>>>> altlog:inc/log.h
=======
} datalog_struct;
datalog_struct datalog;
>>>>>>> altlog:inc/log.h


// Log functions
void  log_init   ( void );
void  log_exit   ( void );
void  log_record ( enum log_index index );
//void  log_BLAH  ( enum log_index index );


#endif



