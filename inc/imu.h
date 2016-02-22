
//============================================================
//  imu.h
//  Justin M Selfridge
//============================================================
#ifndef _IMU_H_
#define _IMU_H_
#include <main.h>


// Define statements

#define USE_IMUA   false
#define USE_IMUB   true

#define GYR_HIST   10
#define ACC_HIST   10
#define MAG_HIST   10

#define GYR_LPF    20.0
#define ACC_LPF    20.0
#define MAG_LPF     2.0

#define GYR_FSR    500
#define ACC_FSR    4
#define GYR_SCALE  ( 500.0 / 32768.0 ) * ( PI / 180.0 )

#define PI  M_PI


// IMU data structure
typedef struct imu_data_struct {
  float gain;
  int   bias  [3];
  int   range [3];
  short raw   [3];
  float avg   [3];
  float cal   [3];
  pthread_mutex_t lock;
} imu_data_struct;
imu_data_struct gyrA;
imu_data_struct accA;
imu_data_struct magA;
imu_data_struct gyrB;
imu_data_struct accB;
imu_data_struct magB;


// IMU structure
typedef struct imu_struct {
  int    fd;
  char   id;
  ushort bus;
  ushort addr;
  ushort loops;
  ushort count;
  bool   getmag;
  imu_data_struct* gyr;
  imu_data_struct* acc;
  imu_data_struct* mag;
} imu_struct;
imu_struct imuA;
imu_struct imuB;


// IMU functions
void  imu_init    ( void );
void  imu_exit    ( void );
void  imu_param   ( imu_struct *imu );
void  imu_getcal  ( imu_struct *imu );
void  imu_setic   ( imu_struct *imu );
void  imu_update  ( imu_struct *imu );


#endif



