

#ifndef AHRS_H
#define AHRS_H


#include <sys/types.h>
#include "imu.h"


#define AHRS_GAIN    0.10


typedef struct ahrs_struct {
  char    id;
  double  dt;
  double  quat   [4];
  double  dquat  [4];
  double  eul    [3];
  double  deul   [3];
  double  offset [3];
  pthread_mutex_t mutex;
} ahrs_struct;
ahrs_struct ahrsA;
ahrs_struct ahrsB;


void  ahrs_init    ( void );
void  ahrs_setup   ( ahrs_struct *ahrs );
void  ahrs_exit    ( void );
void  ahrs_update  ( ahrs_struct *ahrs, imu_struct *imu );


#endif



