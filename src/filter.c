

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
  filter_gyrA.dt = 1.0 / HZ_IMU_FAST;
  filter_accA.dt = 1.0 / HZ_IMU_FAST;
  filter_magA.dt = 1.0 / HZ_IMU_SLOW;
  filter_gyrB.dt = 1.0 / HZ_IMU_FAST;
  filter_accB.dt = 1.0 / HZ_IMU_FAST;
  filter_magB.dt = 1.0 / HZ_IMU_SLOW;
  filter_eul.dt  = 1.0 / HZ_AHRS;
  filter_ang.dt  = 1.0 / HZ_AHRS;

  // Assign array dimensions
  if(DEBUG)  printf( "  Assign array dimensions \n" );
  filter_gyrA.dim = 3;
  filter_accA.dim = 3;
  filter_magA.dim = 3;
  filter_gyrB.dim = 3;
  filter_accB.dim = 3;
  filter_magB.dim = 3;
  filter_eul.dim  = 3;
  filter_ang.dim  = 3;

  // Store cutoff frequencies
  if(DEBUG)  printf( "  Store cutoff frequencies \n" );
  filter_freq( &filter_gyrA, LPF_FREQ_GYR );
  filter_freq( &filter_accA, LPF_FREQ_ACC );
  filter_freq( &filter_magA, LPF_FREQ_MAG );
  filter_freq( &filter_gyrB, LPF_FREQ_GYR );
  filter_freq( &filter_accB, LPF_FREQ_ACC );
  filter_freq( &filter_magB, LPF_FREQ_MAG );
  filter_freq( &filter_eul,  LPF_FREQ_EUL );
  filter_freq( &filter_ang,  LPF_FREQ_ANG );

  // Generate memory pointer
  if(DEBUG)  printf( "  Generate memory pointer \n" );
  filter_gyrA.data = malloc(0);
  filter_accA.data = malloc(0);
  filter_magA.data = malloc(0);
  filter_gyrB.data = malloc(0);
  filter_accB.data = malloc(0);
  filter_magB.data = malloc(0);
  filter_eul.data  = malloc(0);
  filter_ang.data  = malloc(0);

  // Allocate storage memory
  if(DEBUG)  printf( "  Allocate storage memory \n" );
  filter_hist( &filter_gyrA, LPF_HIST_GYR );
  filter_hist( &filter_accA, LPF_HIST_ACC );
  filter_hist( &filter_magA, LPF_HIST_MAG );
  filter_hist( &filter_gyrB, LPF_HIST_GYR );
  filter_hist( &filter_accB, LPF_HIST_ACC );
  filter_hist( &filter_magB, LPF_HIST_MAG );
  filter_hist( &filter_eul,  LPF_HIST_EUL );
  filter_hist( &filter_ang,  LPF_HIST_ANG );

  // Display settings
  if (DEBUG) {
    printf( "  Filter settings: \n" );
    printf("  -------------------------------------------------\n" );
    printf("  |       |   HZ  |     DT  |     LPF  |    Gain  |\n" );
    printf("  |  GYR  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_IMU_FAST, filter_gyrA.dt, LPF_FREQ_GYR, filter_gyrA.gain );
    printf("  |  ACC  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_IMU_FAST, filter_accA.dt, LPF_FREQ_ACC, filter_accA.gain );
    printf("  |  MAG  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_IMU_SLOW, filter_magA.dt, LPF_FREQ_MAG, filter_magA.gain );
    printf("  |  EUL  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_AHRS, filter_eul.dt, LPF_FREQ_EUL, filter_eul.gain );
    printf("  |  ANG  |  %3d  |  %5.3f  |  %6.2f  |  %6.4f  |\n", \
       HZ_AHRS, filter_ang.dt, LPF_FREQ_ANG, filter_ang.gain );
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
 *  Assign a new cutoff frequency to a filtered signal.
 */
void filter_freq ( filter_struct *filter, double freq )  {

  // Local variables
  double tc, gain;

  // Determine time constant
  if (freq)  tc = 1.0 / ( 2.0 * M_PI * freq );
  else       tc = 0.0;

  // Calculate gain
  gain = filter->dt / ( tc + filter->dt );

  // Assign values
  filter->freq = freq;
  filter->gain = gain;

  return;
}


/**
 *  filter_hist
 *  Change the data sample history length of a filtered signal.
 */
void filter_hist ( filter_struct *filter, uint hist )  {

  // Local variables
  ushort i, size;

  // Assign new value
  filter->hist = hist;

  // Determine new stroage size
  size = filter->dim * hist;

  // Reallocate memory for storage
  filter->data = realloc( filter->data, sizeof(double) * size );

  // Zero out element values
  for ( i=0; i<size; i++ )  filter->data[i] = 0.0;

  return;
}


/**
 *  filter_lpf
 *  Run a signal through a low pass filter.
 */
void filter_lpf ( filter_struct *filter, double *input, double *output )  {

  // Local variables
  ushort r, c;
  ushort row   = filter->hist;
  ushort col   = filter->dim;
  double gain  = filter->gain;
  double *data = filter->data;
  double lpf[col];

  // Loop through dimensions
  for ( c=0; c<col; c++ )  {

    // Shift stored data
    for ( r=1; r<row; r++ )  data[ col * r + c - col ] = data[ col * r + c ];

    // Assign newest data value
    data[ (row-1) * col + c ] = input[c];

    // Initialize starting value
    lpf[c] = data[c];

    // Loop through sequence
    for ( r=1; r<row; r++ )  lpf[c] = lpf[c] + gain * ( data[ r * col + c ] - lpf[c] );

    // Return filtered result
    output[c] = lpf[c];

  }

  return;
}



