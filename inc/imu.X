
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

#define GYR_LPF   (0.0f)
#define ACC_LPF   (0.0f)
#define MAG_LPF   (0.0f)

#define GYR_FSR   500
#define ACC_FSR   4

#define GYR_SCALE  ( 500.0f / 32768.0f ) * ( PI / 180.0f )
#define GYR_ERROR  5.0f * ( PI / 180.0 )
#define GYR_DRIFT  0.2f * ( PI / 180.0 )
#define IMU_BETA   sqrt( 3.0f / 4.0f ) * GYR_ERROR
#define IMU_ZETA   sqrt( 3.0f / 4.0f ) * GYR_DRIFT
#define R_BIAS      0.0f * ( PI / 180.0 )
#define P_BIAS      0.0f * ( PI / 180.0 )
#define Y_BIAS      0.0f * ( PI / 180.0 )

#define X   0
#define Y   1
#define Z   2
#define PI  M_PI


// IMU structure
typedef struct {
  ushort  id;
  ushort  addr;
  ushort  count;
  ushort  loops;
  ushort  hz_imu;
  ushort  hz_att;
  float   dt_imu;
  float   dt_att;
  float   lpf_gyr;
  float   lpf_acc;
  float   lpf_mag;
  float   tc_gyr;
  float   tc_acc;
  float   tc_mag;
  float   gain_gyr;
  float   gain_acc;
  float   gain_mag;
  int     moffset   [3];
  int     aoffset   [3];
  int     mrange    [3];
  int     arange    [3];
  short   rawGyr    [3];
  short   rawAcc    [3];
  short   rawMag    [3];
  short   histGyr   [3][GYR_HIST];
  short   histAcc   [3][ACC_HIST];
  short   histMag   [3][MAG_HIST];
  float   avgGyr    [3];
  float   avgAcc    [3];
  float   avgMag    [3];
  float   calGyr    [3];
  float   calAcc    [3];
  float   calMag    [3];
  double  Quat      [4];
  double  dQuat     [4];
  double  Eul       [3];
  double  dEul      [3];
  double  bias      [3];
  double  fx;
  double  fz;
} imu_struct;
imu_struct imu1;
//imu_struct imu2;


// IMU functions
void    imu_init     ( void );
void    imu_setup    ( imu_struct* imu );
void    imu_param    ( imu_struct* imu );
void    imu_getcal   ( imu_struct* imu );
void    imu_setic    ( imu_struct* imu );
void    imu_data     ( imu_struct* imu );
void    imu_fusion   ( imu_struct* imu );
void    imu_exit     ( void );


#endif


