
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Define statements
#define MAX_LOG_DUR  1

// Log enumerations
enum log_index {
  LOG_GYR  = 0,
  LOG_ACC  = 1,
  LOG_MAG  = 2
} log_index;


// Log IMU structure
typedef struct log_imu_struct {
  int   fd;
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
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
  bool   open;
  float  offset;
  char   *dir;
  char   *path;
} log_struct;
log_struct datalog; 


// Log functions
void  log_init   ( void );
void  log_exit   ( void );
//void  log_write  ( enum log_index index );
//void  log_open   ( void );
//void  log_record ( enum log_index index );


#endif



