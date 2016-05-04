

#ifndef IMU_H
#define IMU_H


#include <math.h>
#include <stdbool.h>
#include <sys/types.h>


#define IMUA_ENABLED   true
#define IMUB_ENABLED   true

#define GYR_FSR    500
#define ACC_FSR    4
#define GYR_SCALE  ( 500.0 / 32768.0 ) * ( PI / 180.0 )
#define PI         M_PI


typedef struct imu_data_struct {
  double dt;
  double gain;
  int    bias   [3];
  int    range  [3];
  short  raw    [3];
  double scaled [3];
  double filter [3];
} imu_data_struct;
imu_data_struct gyr;
imu_data_struct acc;
imu_data_struct mag;
imu_data_struct gyrA;
imu_data_struct accA;
imu_data_struct magA;
imu_data_struct gyrB;
imu_data_struct accB;
imu_data_struct magB;


typedef struct imu_struct {
  int    fd;
  char   id;
  ushort bus;
  ushort addr;
  ushort loops;
  ushort count;
  bool   getmag;
  imu_data_struct *gyr;
  imu_data_struct *acc;
  imu_data_struct *mag;
} imu_struct;
imu_struct imu;
imu_struct imuA;
imu_struct imuB;


void  imu_init    ( void );
void  imu_exit    ( void );
void  imu_param   ( imu_struct *imu );
void  imu_getcal  ( imu_struct *imu );
void  imu_update  ( imu_struct *imu );


#endif



