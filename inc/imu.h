
//============================================================
//  imu.h
//  Justin M Selfridge
//============================================================
#ifndef _IMU_H_
#define _IMU_H_
#include <main.h>


// Define statements

#define MEMS_HZ   200
#define COMP_HZ    50

#define MEMS_HIST  1
#define COMP_HIST  1

#define MEMS_LPF  (0.0f)
#define COMP_LPF  (0.0f)

#define GYRO_FSR  500
#define ACC_FSR  4

#define GYRO_SCALE  ( 500.0f / 32768.0f ) * ( PI / 180.0f )
//#define GYRO_ERROR  5.0f * ( PI / 180.0 )
//#define GYRO_DRIFT  0.2f * ( PI / 180.0 )
//#define IMU_BETA    sqrt( 3.0f / 4.0f ) * GYRO_ERROR
//#define IMU_ZETA    sqrt( 3.0f / 4.0f ) * GYRO_DRIFT
//#define R_BIAS      -3.9f * ( PI / 180.0 )
//#define P_BIAS      -1.0f * ( PI / 180.0 )
//#define Y_BIAS       0.0f * ( PI / 180.0 )
#define X   0
#define Y   1
#define Z   2
#define PI  M_PI


// IMU structure
typedef struct {
  ushort  bus;
  ushort  mems_hz;
  ushort  comp_hz;
  float   mems_dt;
  float   comp_dt;
  float   mems_lpf;
  float   comp_lpf;
  float   mems_tc;
  float   comp_tc;
  float   mems_gain;
  float   comp_gain;
  int     moffset   [3];
  int     aoffset   [3];
  int     mrange    [3];
  int     arange    [3];
  short   rawGyro   [3];
  short   rawAcc    [3];
  short   rawMag    [3];
  short   histGyro  [3][MEMS_HIST];
  short   histAcc   [3][MEMS_HIST];
  short   histMag   [3][COMP_HIST];
  float   avgGyro   [3];
  float   avgAcc    [3];
  float   avgMag    [3];
  float   calGyro   [3];
  float   calAcc    [3];
  float   calMag    [3];
  //double  Quat      [4];
  //double  dQuat     [4];
  //double  Eul       [3];
  //double  dEul      [3];
  //double  bias      [3];
  //double  fx;
  //double  fz;
} imu_struct;
imu_struct imu1;


// IMU functions
void    imu_init     ( imu_struct* imu, ushort bus );
void    imu_exit     ( void );
void    imu_param    ( imu_struct* imu );
void    imu_setcal   ( imu_struct* imu );
//void    imu_conv     ( imu_struct* imu );
void    imu_setic    ( imu_struct* imu );
bool    imu_avail    ( imu_struct* imu );
void    imu_mems     ( imu_struct* imu );
void    imu_comp     ( imu_struct* imu );
//void    imu_fusion   ( imu_struct* imu );
//short   imu_row_map  ( const signed char* row );
//short   imu_orient   ( const signed char* mtx );


#endif



