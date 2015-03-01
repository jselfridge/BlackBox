
//============================================================
//  imu.h
//  Justin M Selfridge
//============================================================
#ifndef _IMU_H_
#define _IMU_H_
#include <main.h>


// Define statements
#define GYRO_SCALE  ( 500.0f / 32768.0f ) * ( PI / 180.0f )
#define GYRO_ERROR  5.0f * ( PI / 180.0 )
#define GYRO_DRIFT  0.2f * ( PI / 180.0 )
#define MPU_BETA    sqrt( 3.0f / 4.0f ) * GYRO_ERROR
#define MPU_ZETA    sqrt( 3.0f / 4.0f ) * GYRO_DRIFT
#define R_BIAS      -3.9f * ( PI / 180.0 )
#define P_BIAS      -1.0f * ( PI / 180.0 )
#define Y_BIAS       0.0f * ( PI / 180.0 )
#define X   0
#define Y   1
#define Z   2
#define PI  M_PI


// MPU structure
typedef struct {
  int bus;
  //signed char rot [9];  // Add this later
  int moffset [3];
  int aoffset [3];
  int mrange  [3];
  int arange  [3];
  unsigned long magTime; 
  unsigned long dmpTime;
  short  rawMag  [3];
  short  rawAcc  [3];
  short  rawGyro [3];
  long   rawQuat [4];
  double normMag  [3];
  double normAcc  [3];
  double normGyro [3];
  double normQuat [4];
  double Quat  [4];
  double dQuat [4];
  double Eul   [3];
  double dEul  [3];
  double bias [3];
  double fx;
  double fz;
} mpu_struct;
mpu_struct mpu1;
//mpu_struct mpu2;


// Global variables
double curr_head;
double prev_head;
double heading;
double factor;
ushort counter;


// MPU functions
void    imu_init     ( mpu_struct* mpu );
void    imu_exit     ( void );
void    imu_param    ( mpu_struct* mpu );
void    imu_setcal   ( mpu_struct* mpu );
void    imu_conv     ( mpu_struct* mpu );
void    imu_setic    ( mpu_struct* mpu );
int     imu_avail    ( void );
void    imu_raw      ( mpu_struct* mpu );
void    imu_norm     ( mpu_struct* mpu );
void    imu_fusion   ( mpu_struct* mpu );
void    imu_sample   ( mpu_struct* mpu );
short   imu_row_map  ( const signed char* row );
short   imu_orient   ( const signed char* mtx );


#endif



