
//============================================================
//  ahr.h
//  Justin M Selfridge
//============================================================
#ifndef _AHR_H_
#define _AHR_H_
#include <main.h>


// Define statements
#define GYR_ERROR  5.0f * ( PI / 180.0 )
#define GYR_DRIFT  0.2f * ( PI / 180.0 )
#define IMU_BETA   sqrt( 3.0f / 4.0f ) * GYR_ERROR
#define IMU_ZETA   sqrt( 3.0f / 4.0f ) * GYR_DRIFT
#define R_BIAS      0.0 * ( PI / 180.0 )
#define P_BIAS      0.0 * ( PI / 180.0 )
#define Y_BIAS      0.0 * ( PI / 180.0 )
// ORIG BIAS: R -3.9  P -1.0  Y 0.0


// AHRS data structure
typedef struct ahr_data_struct {
  double  dt;
  double  quat  [4];
  double  dquat [4];
  double  eul   [3];
  double  deul  [3];
  double  bias  [3];
  double  fx;
  double  fz;
} ahr_data_struct;
ahr_data_struct ahr;


// IMU functions
void  ahr_init    ( void );
void  ahr_exit    ( void );
void  ahr_run     ( void );
void  ahr_fusion  ( void );
void  ahr_kalman  ( void );


#endif



