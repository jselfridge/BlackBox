

#include "lpf.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sys.h"
#include "timer.h"


/**
 *  lpf_init
 *  Initializes the low pass filters for the system signals.
 */
void lpf_init ( void )  {
  if(DEBUG)  printf( "Initializing low pass filters \n" );

  // Calculate time steps
  if(DEBUG)  printf( "  Calculate time steps \n" );
  lpf_gyrA.dt = 1.0 / HZ_IMU_FAST;
  lpf_accA.dt = 1.0 / HZ_IMU_FAST;
  lpf_magA.dt = 1.0 / HZ_IMU_SLOW;
  lpf_gyrB.dt = 1.0 / HZ_IMU_FAST;
  lpf_accB.dt = 1.0 / HZ_IMU_FAST;
  lpf_magB.dt = 1.0 / HZ_IMU_SLOW;

  // Assign array dimensions
  if(DEBUG)  printf( "  Assign array dimensions \n" );
  lpf_gyrA.dim = 3;
  lpf_accA.dim = 3;
  lpf_magA.dim = 3;
  lpf_gyrB.dim = 3;
  lpf_accB.dim = 3;
  lpf_magB.dim = 3;

  // Store cutoff frequencies
  if(DEBUG)  printf( "  Store cutoff frequencies \n" );
  lpf_freq( &lpf_gyrA, LPF_FREQ_GYR );
  lpf_freq( &lpf_accA, LPF_FREQ_ACC );
  lpf_freq( &lpf_magA, LPF_FREQ_MAG );
  lpf_freq( &lpf_gyrB, LPF_FREQ_GYR );
  lpf_freq( &lpf_accB, LPF_FREQ_ACC );
  lpf_freq( &lpf_magB, LPF_FREQ_MAG );

  // Generate memory pointer
  if(DEBUG)  printf( "  Generate memory pointer \n" );
  lpf_gyrA.data = malloc(0);
  lpf_accA.data = malloc(0);
  lpf_magA.data = malloc(0);
  lpf_gyrB.data = malloc(0);
  lpf_accB.data = malloc(0);
  lpf_magB.data = malloc(0);

  // Declare sampling history
  if(DEBUG)  printf( "  Allocate storage memory \n" );
  lpf_hist( &lpf_gyrA, LPF_HIST_GYR );
  lpf_hist( &lpf_accA, LPF_HIST_ACC );
  lpf_hist( &lpf_magA, LPF_HIST_MAG );
  lpf_hist( &lpf_gyrB, LPF_HIST_GYR );
  lpf_hist( &lpf_accB, LPF_HIST_ACC );
  lpf_hist( &lpf_magB, LPF_HIST_MAG );

  // Display settings
  if (DEBUG) {
    printf( "  Low pass filter settings: \n" );
    printf("  -------------------------------------------------\n" );
    printf("  |       |   HZ  |     DT  |    LPF  |    Gain  |\n" );
    printf("  |  GYR  |  %3d  |  %5.3f  |  %5.2f  |  %6.4f  |\n", \
       HZ_IMU_FAST, lpf_gyrA.dt, LPF_FREQ_GYR, lpf_gyrA.gain );
    printf("  |  ACC  |  %3d  |  %5.3f  |  %5.2f  |  %6.4f  |\n", \
       HZ_IMU_FAST, lpf_accA.dt, LPF_FREQ_ACC, lpf_accA.gain );
    printf("  |  MAG  |  %3d  |  %5.3f  |  %5.2f  |  %6.4f  |\n", \
       HZ_IMU_SLOW, lpf_magA.dt, LPF_FREQ_MAG, lpf_magA.gain );
    printf("  -------------------------------------------------\n" );
  }

  return;
}


/**
 *  lpf_exit
 *  Terminate the system low pass filters.
 */
void lpf_exit ( void )  {
  if(DEBUG)  printf("Close low pass filters \n");
  // Add code as needed...
  return;
}


/**
 *  lpf_freq
 *  Assign a new cutoff frequency to a filtered signal.
 */
void lpf_freq ( lpf_struct *lpf, double freq )  {

  // Local variables
  double tc, gain;

  // Determine time constant
  if (freq)  tc = 1.0 / ( 2.0 * M_PI * freq );
  else       tc = 0.0;

  // Calculate gain
  gain = lpf->dt / ( tc + lpf->dt );

  // Assign values
  lpf->freq = freq;
  lpf->gain = gain;

  return;
}


/**
 *  lpf_hist
 *  Change the data sample history length of a filtered signal.
 */
void lpf_hist ( lpf_struct *lpf, uint hist )  {

  // Local variables
  ushort i, size;

  // Assign new value
  lpf->hist = hist;

  // Determine new stroage size
  size = lpf->dim * hist;

  // Reallocate memory for storage
  lpf->data = realloc( lpf->data, sizeof(double) * size );

  // Zero out element values
  for ( i=0; i<size; i++ )  lpf->data[i] = 0.0;

  return;
}


/**
 *  lpf_update
 *  Run a signal through a low pass filter.
 */
void lpf_update ( lpf_struct *lpf, double *input, double *output )  {

  // Local variables
  ushort r, c;
  ushort row   = lpf->hist;
  ushort col   = lpf->dim;
  double gain  = lpf->gain;
  double *data = lpf->data;
  double array[col];

  // Loop through dimensions
  for ( c=0; c<col; c++ )  {

    // Shift stored data
    for ( r=1; r<row; r++ )  data[ col * r + c - col ] = data[ col * r + c ];

    // Assign newest data value
    data[ (row-1) * col + c ] = input[c];

    // Initialize starting value
    array[c] = data[c];

    // Loop through sequence
    for ( r=1; r<row; r++ )  array[c] = array[c] + gain * ( data[ r * col + c ] - array[c] );

    // Return filtered result
    output[c] = array[c];

  }

  return;
}



