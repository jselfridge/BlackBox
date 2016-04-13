

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

  // Assign array dimensions
  if(DEBUG)  printf( "  Assign array dimensions \n" );
  filter_gyrA.dim = 1;  filter_gyrB.dim = 1;
  filter_accA.dim = 1;  filter_accB.dim = 1;
  filter_magA.dim = 1;  filter_magB.dim = 1;

  // Store cutoff frequencies
  if(DEBUG)  printf( "  Store cutoff frequencies \n" );
  filter_freq( &filter_gyrA, LPF_GYR );  filter_freq( &filter_gyrB, LPF_GYR );
  filter_freq( &filter_accA, LPF_ACC );  filter_freq( &filter_accB, LPF_ACC );
  filter_freq( &filter_magA, LPF_MAG );  filter_freq( &filter_magB, LPF_MAG );

  // Generate memory pointer
  if(DEBUG)  printf( "  Generate memory pointer \n" );
  filter_gyrA.data = malloc(0);  filter_gyrB.data = malloc(0);
  filter_accA.data = malloc(0);  filter_accB.data = malloc(0);
  filter_magA.data = malloc(0);  filter_magB.data = malloc(0);

  // Allocate storage memory
  if(DEBUG)  printf( "  Allocate storage memory \n" );
  filter_hist( &filter_gyrA, HIST_GYR );  filter_hist( &filter_gyrB, HIST_GYR );
  filter_hist( &filter_accA, HIST_ACC );  filter_hist( &filter_accB, HIST_ACC );
  filter_hist( &filter_magA, HIST_MAG );  filter_hist( &filter_magB, HIST_MAG );

  // Display settings
  if (DEBUG) {
    printf( "  Filter settings: \n" );
    printf("  -------------------------------------------------\n" );
    printf("  |       |   HZ  |     DT  |     LPF  |    Gain  |\n" );
    printf("  |  GYR  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_IMU_FAST, filter_gyrA.dt, LPF_GYR, filter_gyrA.gain );
    printf("  |  ACC  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_IMU_FAST, filter_accA.dt, LPF_ACC, filter_accA.gain );
    printf("  |  MAG  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_IMU_SLOW, filter_magA.dt, LPF_MAG, filter_magA.gain );
    printf("  -------------------------------------------------\n" );
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

  // Local variables
  double tc, gain;

  // Determine time constant
  if ( freq != 0.0 )
    tc = 1.0 / ( 2.0 * M_PI * freq );
  else
    tc = 0.0;

  // Calculate gain
  gain = filter->dt / ( tc + filter->dt );

  // Assign values
  filter->freq = freq;
  filter->gain = gain;

  return;
}


/**
 *  filter_hist
 *  Change the data sample size of a filter.
 */
void filter_hist ( filter_struct *filter, uint hist )  {

  // Assign new value
  filter->hist = hist;

  // Free current memory
  free(filter->data);

  // Determine new stroage size
  ushort size = filter->dim * hist;

  // Allocate new storage
  filter->data = malloc( sizeof(double) * size );

  // Zero out values
  ushort i;
  for ( i=0; i<size; i++ )  filter->data[i] = 0.0;

  return;
}


/**
 *  filter_lpf
 *  Run a signal through a low pass filter.
 */
void filter_lpf ( filter_struct *filter, double *input, double *output )  {

  // Local variables
  ushort i, j;
  ushort h     = filter->hist;
  ushort d     = filter->dim;
  double gain  = filter->gain;
  double *data = filter->data;
  double lpf[d];

  //for ( i=0; i<h; i++ )  printf("%f ", data[i] );  printf("\n");  //DEBUG

  // Shift stored data
  for ( i=1; i<h; i++ ) {
    for ( j=0; j<d; j++ ) {
      data[d*i+j-d] = data[d*i+j ];
     //data[i-1] = data[i];
    }
  }
  //for ( i=0; i<h; i++ )  printf("%f ", data[i] );  printf("\n");  //DEBUG

  // Assign newest data value
  for ( j=0; j<d; j++ )  data[(h-1)*d+j] = input[j];
  //data[h-1] = *input;
  //for ( i=0; i<h; i++ )  printf("%f ", data[i] );  printf("\n");  //DEBUG

  // Initialize starting value
  for ( j=0; j<d; j++ )  lpf[j] = data[j];
  //lpf = data[0];
  //printf("%f \n", lpf);  // DEBUG

  // Loop through sequence
  for ( i=1; i<h; i++ ) {
    for ( j=0; j<d; j++ ) {
      lpf[j] = lpf[j] + gain * ( data[i*d+j] - lpf[j] );
    }
  }
  //printf("%f \n", lpf);  // DEBUG

  // Return filtered result
  for ( j=0; j<d; j++ )  output[j] = lpf[j];

  return;
}



