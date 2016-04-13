

#include "filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sys.h"
#include "timer.h"


/**
 *  filter_init
 *  Initializes the system signal filters.
 */
void filter_init ( void )  {
  if(DEBUG)  printf( "Initializing filters \n" );

  // Calculate time steps
  if(DEBUG)  printf( "  Calculate time steps \n" );
  filter_gyrA.dt = 1.0 / HZ_IMU_FAST;  filter_gyrB.dt = 1.0 / HZ_IMU_FAST;
  filter_accA.dt = 1.0 / HZ_IMU_FAST;  filter_accB.dt = 1.0 / HZ_IMU_FAST;
  filter_magA.dt = 1.0 / HZ_IMU_SLOW;  filter_magB.dt = 1.0 / HZ_IMU_SLOW;

  // Assign array dimension
  if(DEBUG)  printf( "  Assign array dimensions \n" );
  filter_gyrA.dim = 3;  filter_gyrB.dim = 3;
  filter_accA.dim = 3;  filter_accB.dim = 3;
  filter_magA.dim = 3;  filter_magB.dim = 3;

  // Store cutoff frequencies
  if(DEBUG)  printf( "  Store cutoff frequencies \n" );
  filter_freq( &filter_gyrA, LPF_GYR );  filter_freq( &filter_gyrB, LPF_GYR );
  filter_freq( &filter_accA, LPF_ACC );  filter_freq( &filter_accB, LPF_ACC );
  filter_freq( &filter_magA, LPF_MAG );  filter_freq( &filter_magB, LPF_MAG );

  // Generate memory pointer
  //if(DEBUG)  printf( "  Generate memory pointer \n" );
  //filter_gyrA.data = malloc(0);

  // Allocate storage memory
  //if(DEBUG)  printf( "  Allocate storage memory \n" );
  //filter_hist( &filter_gyrA, HIST_GYR );  filter_hist( &filter_gyrB, HIST_GYR );
  //filter_hist( &filter_accA, HIST_ACC );  filter_hist( &filter_accB, HIST_ACC );
  //filter_hist( &filter_magA, HIST_MAG );  filter_hist( &filter_magB, HIST_MAG );

  // Display settings
  if (DEBUG) {
    printf("  |  GYR  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  gain %7.4f  |\n", \
       HZ_IMU_FAST, filter_gyrA.dt, LPF_GYR, filter_gyrA.gain );
    printf("  |  ACC  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  gain %7.4f  |\n", \
       HZ_IMU_FAST, filter_accA.dt, LPF_ACC, filter_accA.gain );
    printf("  |  MAG  |  HZ %4d  |  DT %5.3f  |  LPF %6.2f  |  gain %7.4f  |\n", \
       HZ_IMU_SLOW, filter_magA.dt, LPF_MAG, filter_magA.gain );
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
 *  filter_freq
 *  
 */
void filter_freq ( filter_struct *filter, double freq )  {

  double tc, gain;

  if ( freq != 0.0 )
    tc = 1.0 / ( 2.0 * M_PI * freq );
  else
    tc = 0.0;

  gain = filter->dt / ( tc + filter->dt );

  filter->freq = freq;
  filter->gain = gain;

  return;
}


/**
 *  filter_hist
 *  
 */
void filter_hist ( filter_struct *filter, uint hist )  {

  free(filter->data);
  return;
}


/**
 *  filter_lpf
 *  Run a signal through a low pass filter.
 */
void filter_lpf ( filter_struct *filter, double *input, double *output )  {

  /*
  // Local variables
  ushort i;
  double lpf;

  // Shift stored data
  for ( i=1; i<hist; i++ )  data[i-1] = data[i];

  // Assign newest data value
  data[hist-1] = sample;

  // Initialize starting value
  lpf = data[0];

  // Loop through sequence
  for ( i=1; i<hist; i++ )  lpf = lpf + gain * ( data[i] - lpf );

  // Return filtered result
  return lpf;
  */

  return;
}



