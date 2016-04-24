

#ifndef AHRS_H
#define AHRS_H


#include <sys/types.h>


#define GYR_ERROR  5.0f * ( PI / 180.0 )
#define GYR_DRIFT  0.2f * ( PI / 180.0 )
#define IMU_BETA   sqrt( 3.0f / 4.0f ) * GYR_ERROR
#define IMU_ZETA   sqrt( 3.0f / 4.0f ) * GYR_DRIFT


typedef struct ahrs_data_struct {
  double  dt;
  double  gyr    [3];
  double  acc    [3];
  double  mag    [3];
  double  quat   [4];
  double  dquat  [4];
  double  eul    [3];
  double  ang    [3];
  double  lpfeul [3];
  double  lpfang [3];
  double  bias   [3];
  double  orient [3];
  double  fx;
  double  fz;
} ahrs_data_struct;
ahrs_data_struct ahrs;


void  ahrs_init    ( void );
void  ahrs_exit    ( void );
void  ahrs_update  ( void );
void  ahrs_fusion  ( void );
void  ahrs_kalman  ( void );


#endif



