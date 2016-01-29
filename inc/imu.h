
//============================================================
//  imu.h
//  Justin M Selfridge
//============================================================
#ifndef _IMU_H_
#define _IMU_H_
#include <main.h>


// Define statements

#define GYR_HIST   1
#define ACC_HIST   1
#define MAG_HIST   1

#define GYR_LPF    0.0
#define ACC_LPF    0.0
#define MAG_LPF    0.0

#define GYR_FSR   500
#define ACC_FSR   4
#define GYR_SCALE  ( 500.0 / 32768.0 ) * ( PI / 180.0 )

#define X   0
#define Y   1
#define Z   2

#define PI  M_PI


// IMU data structure
typedef struct imu_data_struct {
  float gain;
  int   bias  [3];
  int   range [3];
  short raw   [3];
  float avg   [3];
  float cal   [3];
} imu_data_struct;
imu_data_struct gyr;
imu_data_struct acc;
imu_data_struct mag;


// IMU structure
typedef struct imu_struct {
  ushort id;
  ushort addr;
  ushort loops;
  ushort count;
  bool   getmag;
  imu_data_struct* gyr;
  imu_data_struct* acc;
  imu_data_struct* mag;
} imu_struct;
imu_struct imu;


// IMU functions
void  imu_init    ( void );
void  imu_exit    ( void );
void  imu_param   ( void );
void  imu_getcal  ( void );
void  imu_setic   ( void );
void  imu_data    ( void );
//void    imu_fusion   ( imu_struct *imu );


#endif



