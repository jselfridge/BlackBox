
//============================================================
//  ahrs.h
//  Justin M Selfridge
//============================================================
#ifndef _AHRS_H_
#define _AHRS_H_
#include <main.h>


// Define statements
#define GYR_ERROR  5.0f * ( PI / 180.0 )
#define GYR_DRIFT  0.2f * ( PI / 180.0 )
#define IMU_BETA   sqrt( 3.0f / 4.0f ) * GYR_ERROR
#define IMU_ZETA   sqrt( 3.0f / 4.0f ) * GYR_DRIFT


// AHRS data structure
typedef struct ahrs_data_struct {
  double  dt;
  double  quat   [4];
  double  dquat  [4];
  double  eul    [3];
  double  deul   [3];
  double  bias   [3];
  double  orient [3];
  double  fx;
  double  fz;
} ahrs_data_struct;
ahrs_data_struct ahrs;


// IMU functions
void  ahrs_init    ( void );
void  ahrs_exit    ( void );
void  ahrs_update  ( void );
void  ahrs_fusion  ( void );
void  ahrs_kalman  ( void );


#endif



