

#include "filter.h"


/**
 *  filter_init
 *  Initializes the system signal filters.
 */
void filter_init ( void )  {

  if(DEBUG)  printf( "Initializing filters \n" );

  // Calculate time steps
  double dt_gyr, dt_acc, dt_mag;
  dt_gyr = 1.0 / HZ_IMU_FAST;
  dt_acc = 1.0 / HZ_IMU_FAST;
  dt_mag = 1.0 / HZ_IMU_SLOW;

  // Determine time constants
  double tc_gyr, tc_acc, tc_mag;
  if ( LPF_GYR != 0.0 )  tc_gyr = 1.0 / ( 2.0 * PI * LPF_GYR );  else  tc_gyr = 0.0;
  if ( LPF_ACC != 0.0 )  tc_acc = 1.0 / ( 2.0 * PI * LPF_ACC );  else  tc_acc = 0.0;
  if ( LPF_MAG != 0.0 )  tc_mag = 1.0 / ( 2.0 * PI * LPF_MAG );  else  tc_mag = 0.0;

  // Determine gains values
  double gain_gyr, gain_acc, gain_mag;
  gain_gyr = dt_gyr / ( tc_gyr + dt_gyr );
  gain_acc = dt_acc / ( tc_acc + dt_acc );
  gain_mag = dt_mag / ( tc_mag + dt_mag );

  // Assign filter parameters
  gyrA.dt = dt_gyr;  gyrA.gain = gain_gyr;
  accA.dt = dt_acc;  accA.gain = gain_acc;
  magA.dt = dt_mag;  magA.gain = gain_mag;
  gyrB.dt = dt_gyr;  gyrB.gain = gain_gyr;
  accB.dt = dt_acc;  accB.gain = gain_acc;
  magB.dt = dt_mag;  magB.gain = gain_mag;

  // Zero out data values
  ushort i, j;
  for ( i=0; i<3; i++ )  {
    for ( j=0; j<HIST_GYR; j++ )  {  gyrA_data[i][j] = 0.0;  gyrB_data[i][j] = 0.0;  }
    for ( j=0; j<HIST_ACC; j++ )  {  accA_data[i][j] = 0.0;  accB_data[i][j] = 0.0;  }
    for ( j=0; j<HIST_MAG; j++ )  {  magA_data[i][j] = 0.0;  magB_data[i][j] = 0.0;  }
  }

  // Display settings
  if (DEBUG) {
    printf("  |  GYR  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
       HZ_IMU_FAST, dt_gyr, LPF_GYR, tc_gyr, gain_gyr );
    printf("  |  ACC  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
       HZ_IMU_FAST, dt_acc, LPF_ACC, tc_acc, gain_acc );
    printf("  |  MAG  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  TC %5.2f  |  gain %7.4f  |\n", \
       HZ_IMU_SLOW, dt_mag, LPF_MAG, tc_mag, gain_mag );
  }

  return;
}


/**
 *  filter_exit
 *  Terminate the system filters.
 */
void filter_exit ( void )  {
  if(DEBUG)  printf("Close filters \n");
  // Add code as needed...
  return;
}


/**
 *  filter_lpf
 *  Run a signal through a low pass filter.
 */
double filter_lpf ( double val, double gain, double *data, ushort hist )  {

  // Local variables
  ushort i;
  double lpf;

  // Shift stored data
  for ( i=1; i<hist; i++ )  data[i-1] = data[i];

  // Assign newest data value
  data[hist-1] = val;

  // Initialize starting value
  lpf = data[0];

  // Loop through sequence
  for ( i=1; i<hist; i++ )  lpf = lpf + gain * ( data[i] - lpf );

  // Return filtered result
  return lpf;

}



