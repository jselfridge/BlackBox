
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
  LOG_FUS  = 1
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
typedef struct {
  bool   enabled;
  //bool   open;
  //float  offset;
  char   *dir;
  char   *path;
} log_struct;
log_struct datalog;


// Log functions
void  log_init   ( void );
void  log_exit   ( void );
//void  log_record ( enum log_index index );
//void  log_write  ( enum log_index index );
//void  log_open   ( void );


#endif



