

#ifndef _FILTER_H_
#define _FILTER_H_
#include <main.h>


// Globals to convert to GCS parameters
/*
double  g_lpf        0.0;
double  a_lpf        0.0;
double  m_lpf        0.0;
*/

// Define history array size
/*
#define GYR_HIST  1;
#define ACC_HIST  1;
#define MAG_HIST  1;
*/

// FIlter data structure
typedef struct filter_struct {
  uint   hist;
  double dt;
  double gain;
  double *data;
} filter_struct;


/*
imu_data_struct gyrA;
imu_data_struct accA;
imu_data_struct magA;
imu_data_struct gyrB;
imu_data_struct accB;
imu_data_struct magB;
*/


// Filter functions
void    filter_init    ( void );
void    filter_exit    ( void );
double  filter_lpf     ( filter_struct *signal, double val );

/*
void  imu_setic   ( imu_struct *imu );
void  imu_update  ( imu_struct *imu );
*/

#endif



