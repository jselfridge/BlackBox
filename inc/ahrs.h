

#ifndef AHRS_H
#define AHRS_H


#include <sys/types.h>
#include "imu.h"


#define IMU_BETA   ( sqrt( 3.0f / 4.0f ) * ( PI / 180.0 ) * 5.0f )
#define IMU_ZETA   ( sqrt( 3.0f / 4.0f ) * ( PI / 180.0 ) * 0.2f )


typedef struct ahrs_struct {
  double  dt;
  double  quat   [4];
  double  dquat  [4];
  double  eul    [3];
  double  deul   [3];
  double  bias   [3];
  //double  orient [3];
  double  fx;
  double  fz;
  pthread_mutex_t mutex;
} ahrs_struct;
ahrs_struct ahrsA;
ahrs_struct ahrsB;


void  ahrs_init    ( void );
void  ahrs_setup   ( ahrs_struct *ahrs );
void  ahrs_exit    ( void );
void  ahrs_update  ( ahrs_struct *ahrs, imu_struct *imu );


#endif



