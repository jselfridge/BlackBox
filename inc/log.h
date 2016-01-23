
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
  LOG_IMU  = 0,
  LOG_AHRS = 1,
  LOG_SIO  = 2
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


// Log system input/output structure
typedef struct log_sio_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ushort *reg;
  ushort *pwm;
  double *norm;
} log_sio_struct;
log_sio_struct log_input;
log_sio_struct log_output;


// Log structure
typedef struct datalog_struct {
  bool   enabled;
  //bool   open;
  float  offset;
  char   *dir;
  char   *path;
} datalog_struct;
datalog_struct datalog;


// Log functions
void  log_init   ( void );
void  log_exit   ( void );
void  log_record ( enum log_index index );
//void  log_BLAH  ( enum log_index index );


#endif



