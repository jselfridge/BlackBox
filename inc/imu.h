
//============================================================
//  imu.h
//  Justin M Selfridge
//============================================================
#ifndef _IMU_H_
#define _IMU_H_
#include <main.h>


// Define statements
//#define GYRO_SCALE  ( 500.0f / 32768.0f ) * ( PI / 180.0f )
//#define GYRO_ERROR  5.0f * ( PI / 180.0 )
//#define GYRO_DRIFT  0.2f * ( PI / 180.0 )
//#define IMU_BETA    sqrt( 3.0f / 4.0f ) * GYRO_ERROR
//#define IMU_ZETA    sqrt( 3.0f / 4.0f ) * GYRO_DRIFT
//#define R_BIAS      -3.9f * ( PI / 180.0 )
//#define P_BIAS      -1.0f * ( PI / 180.0 )
//#define Y_BIAS       0.0f * ( PI / 180.0 )
//#define X   0
//#define Y   1
//#define Z   2
//#define PI  M_PI


// IMU structure
typedef struct {
  int     bus;
  //int     moffset   [3];
  //int     aoffset   [3];
  //int     mrange    [3];
  //int     arange    [3];
  short   rawGyro   [3];
  short   rawAcc    [3];
  short   rawMag    [3];
  //long    rawQuat   [4];
  //short   histMag   [3][12];
  //short   histAcc   [3][12];
  //short   histGyro  [3][12];
  //double  avgMag    [3];
  //double  avgAcc    [3];
  //double  avgGyro   [3];
  //double  normMag   [3];
  //double  normAcc   [3];
  //double  normGyro  [3];
  //double  normQuat  [4];
  //double  Quat      [4];
  //double  dQuat     [4];
  //double  Eul       [3];
  //double  dEul      [3];
  //double  bias      [3];
  //double  fx;
  //double  fz;
  //double  weight    [12];
  //unsigned long magTime; 
  //unsigned long dmpTime;
  //signed char rot [9];  // Add this later
} imu_struct;
imu_struct imu1;
//imu_struct imu2;


// IMU functions
void    imu_init     ( imu_struct* imu );
void    imu_exit     ( void );
void    imu_param    ( imu_struct* imu );
//void    imu_setcal   ( imu_struct* imu );
//void    imu_conv     ( imu_struct* imu );
//void    imu_setic    ( imu_struct* imu );
//int     imu_avail    ( void );
void    imu_gyro      ( imu_struct* imu );
void    imu_acc       ( imu_struct* imu );
void    imu_mag       ( imu_struct* imu );
//void    imu_raw      ( imu_struct* imu );
//void    imu_avg      ( imu_struct* imu );
//void    imu_norm     ( imu_struct* imu );
//void    imu_fusion   ( imu_struct* imu );
//void    imu_sample   ( imu_struct* imu );
//short   imu_row_map  ( const signed char* row );
//short   imu_orient   ( const signed char* mtx );


#endif



