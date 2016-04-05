

#ifndef _IMU_H_
#define _IMU_H_
#include <filter.h>
#include <main.h>


// Define hardware
#define IMUA_ENABLED   true
#define IMUB_ENABLED   false

// Define LPF history length
//#define GYR_HIST   1
//#define ACC_HIST   1
//#define MAG_HIST   1

// Define LPF cutoff freq
#define GYR_LPF    0.0
#define ACC_LPF    0.0
#define MAG_LPF    0.0

// Define MPU constants
#define GYR_FSR    500
#define ACC_FSR    4
#define GYR_SCALE  ( 500.0 / 32768.0 ) * ( PI / 180.0 )
#define PI  M_PI


// Forward declaration
struct filter_struct;


// IMU data structure
typedef struct imu_data_struct {
  int    bias       [3];
  int    range      [3];
  short  raw        [3];
  double scaled     [3];
  double filter     [3];
  struct filter_struct *lpf;
} imu_data_struct;
imu_data_struct gyrA;
imu_data_struct accA;
imu_data_struct magA;
imu_data_struct gyrB;
imu_data_struct accB;
imu_data_struct magB;


// IMU structure
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
imu_struct imuA;
imu_struct imuB;


// IMU functions
void  imu_init    ( void );
void  imu_exit    ( void );
void  imu_param   ( imu_struct *imu );
void  imu_getcal  ( imu_struct *imu );
void  imu_setic   ( imu_struct *imu );
void  imu_update  ( imu_struct *imu );


#endif



