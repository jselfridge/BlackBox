
//============================================================
//  imu.h
//  Justin M Selfridge
//============================================================
#ifndef _IMU_H_
#define _IMU_H_
#include <main.h>


// Define statements

#define GYR_HIST   10
#define ACC_HIST   10
#define MAG_HIST   10

#define GYR_LPF   10.0f
#define ACC_LPF    1.0f
#define MAG_LPF    0.1f

#define GYR_FSR   500
#define ACC_FSR   4

#define GYR_SCALE  ( 500.0f / 32768.0f ) * ( PI / 180.0f )
//#define GYR_ERROR  5.0f * ( PI / 180.0 )
//#define GYR_DRIFT  0.2f * ( PI / 180.0 )
//#define IMU_BETA   sqrt( 3.0f / 4.0f ) * GYR_ERROR
//#define IMU_ZETA   sqrt( 3.0f / 4.0f ) * GYR_DRIFT
//#define R_BIAS     -3.9f * ( PI / 180.0 )
//#define P_BIAS     -1.0f * ( PI / 180.0 )
//#define Y_BIAS      0.0f * ( PI / 180.0 )

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
  imu_data_struct* gyr;
  imu_data_struct* acc;
  imu_data_struct* mag;
} imu_struct;
imu_struct imu;


// IMU functions
void  imu_init    ( void ); //imu_struct *imu );
void  imu_exit    ( void );
void  imu_param   ( void );
void  imu_getcal  ( void );
void  imu_setic   ( void );
void  imu_gyr     ( void );
void  imu_acc     ( void );
void  imu_mag     ( void );
//void    imu_fusion   ( imu_struct *imu );


#endif



