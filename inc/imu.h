

#ifndef IMU_H
#define IMU_H


#include <math.h>
#include <stdbool.h>
#include <sys/types.h>


#define IMUA_ENABLED   false
#define IMUB_ENABLED   false

#define GYR_FSR    500
#define ACC_FSR    4
#define MAG_LOOPS  4
#define GYR_SCALE  ( 500.0 / 32768.0 ) * ( PI / 180.0 )
#define PI         M_PI


typedef struct imu_data_struct {
  int    bias   [3];
  int    range  [3];
  double lpf    [3];
  short  raw    [3];
  double scaled [3];
  double filter [3];
  pthread_mutex_t mutex;
} imu_data_struct;
imu_data_struct gyrA;
imu_data_struct accA;
imu_data_struct magA;
imu_data_struct gyrB;
imu_data_struct accB;
imu_data_struct magB;


typedef struct imu_comp_struct {
  double bias [2];
  double gain;
  double roll;
  double pitch;
  pthread_mutex_t mutex;
} imu_comp_struct;
imu_comp_struct compA;
imu_comp_struct compB;


typedef struct imu_ahrs_struct {
  double  gain;
  double  bias   [3];
  double  quat   [4];
  double  dquat  [4];
  double  eul    [3];
  double  deul   [3];
  pthread_mutex_t mutex;
} imu_ahrs_struct;
imu_ahrs_struct ahrsA;
imu_ahrs_struct ahrsB;


typedef struct imu_struct {
  int    fd;
  char   id;
  ushort bus;
  ushort addr;
  ushort count;
  bool   getmag;
  imu_data_struct *gyr;
  imu_data_struct *acc;
  imu_data_struct *mag;
  imu_comp_struct *comp;
  imu_ahrs_struct *ahrs;
  pthread_mutex_t mutex;
} imu_struct;
imu_struct imuA;
imu_struct imuB;


typedef struct rot_state_struct {
  double att [3];
  double ang [3];
  pthread_mutex_t mutex;
} rot_state_struct;
rot_state_struct rot;


void  imu_init    ( void );
void  imu_exit    ( void );
void  imu_update  ( imu_struct *imu );
void  imu_state   ( void );


#endif



