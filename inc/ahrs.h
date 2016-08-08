

#ifndef AHRS_H
#define AHRS_H


#include <sys/types.h>
#include "imu.h"


// B: 5.0  Z: 0.2
#define IMU_BETA   ( sqrt( 3.0f / 4.0f ) * ( PI / 180.0 ) * 2.0f )
#define IMU_ZETA   ( sqrt( 3.0f / 4.0f ) * ( PI / 180.0 ) * 0.0f )


// WORK IN PROGRESS
//extern int instability_fix;
//extern volatile float beta;            // algorithm gain
//extern volatile float q0, q1, q2, q3;  // quaternion of sensor frame relative to auxiliary frame


typedef struct ahrs_struct {
  char    id;
  double  dt;
  double  quat   [4];
  double  dquat  [4];
  double  eul    [3];
  double  deul   [3];
  double  bias   [3];
  double  orient [3];
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


// WORK IN PROGRESS
void  MadgwickAHRSupdate     ( ahrs_struct *ahrs, imu_struct *imu );
void  MadgwickAHRSupdateIMU  ( ahrs_struct *ahrs, imu_struct *imu );
float invSqrt                ( float x ); 

#endif



