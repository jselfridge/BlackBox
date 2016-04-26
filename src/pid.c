

#include "pid.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//#include "sys.h"
//#include "timer.h"


/**
 *  pid_init
 *  Initializes the PID controller structures.
 */
void pid_init ( void )  {

  /* // Calculate time steps
  if(DEBUG)  printf( "  Calculate time steps \n" );
  filter_gyrA.dt = 1.0 / HZ_IMU_FAST;
  filter_accA.dt = 1.0 / HZ_IMU_FAST;
  filter_magA.dt = 1.0 / HZ_IMU_SLOW;
  filter_gyrB.dt = 1.0 / HZ_IMU_FAST;
  filter_accB.dt = 1.0 / HZ_IMU_FAST;
  filter_magB.dt = 1.0 / HZ_IMU_SLOW;
  filter_eul.dt  = 1.0 / HZ_AHRS;
  filter_ang.dt  = 1.0 / HZ_AHRS; */

  /* // Assign array dimensions
  if(DEBUG)  printf( "  Assign array dimensions \n" );
  filter_gyrA.dim = 3;
  filter_accA.dim = 3;
  filter_magA.dim = 3;
  filter_gyrB.dim = 3;
  filter_accB.dim = 3;
  filter_magB.dim = 3;
  filter_eul.dim  = 3;
  filter_ang.dim  = 3; */

  /* // Store cutoff frequencies
  if(DEBUG)  printf( "  Store cutoff frequencies \n" );
  filter_freq( &filter_gyrA, LPF_FREQ_GYR );
  filter_freq( &filter_accA, LPF_FREQ_ACC );
  filter_freq( &filter_magA, LPF_FREQ_MAG );
  filter_freq( &filter_gyrB, LPF_FREQ_GYR );
  filter_freq( &filter_accB, LPF_FREQ_ACC );
  filter_freq( &filter_magB, LPF_FREQ_MAG );
  filter_freq( &filter_eul,  LPF_FREQ_EUL );
  filter_freq( &filter_ang,  LPF_FREQ_ANG ); */

  /* // Generate memory pointer
  if(DEBUG)  printf( "  Generate memory pointer \n" );
  filter_gyrA.data = malloc(0);
  filter_accA.data = malloc(0);
  filter_magA.data = malloc(0);
  filter_gyrB.data = malloc(0);
  filter_accB.data = malloc(0);
  filter_magB.data = malloc(0);
  filter_eul.data  = malloc(0);
  filter_ang.data  = malloc(0); */

  /* // Allocate storage memory
  if(DEBUG)  printf( "  Allocate storage memory \n" );
  filter_hist( &filter_gyrA, LPF_HIST_GYR );
  filter_hist( &filter_accA, LPF_HIST_ACC );
  filter_hist( &filter_magA, LPF_HIST_MAG );
  filter_hist( &filter_gyrB, LPF_HIST_GYR );
  filter_hist( &filter_accB, LPF_HIST_ACC );
  filter_hist( &filter_magB, LPF_HIST_MAG );
  filter_hist( &filter_eul,  LPF_HIST_EUL );
  filter_hist( &filter_ang,  LPF_HIST_ANG ); */

  /* // Display settings
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
    } */

  return;
}


/**
 *  pid_exit
 *  Terminate the PID entities.
 */
void pid_exit ( void )  {
  // Add code as needed...
  return;
}


/**
 *  pid_setp
 *  Assign a new P gain value.
 */
void pid_setp ( pid_struct *pid, double P )  {
  pthread_mutex_lock(&mutex_pid);
  pid->P = P;
  pthread_mutex_unlock(&mutex_pid);
  return;
}


/**
 *  pid_seti
 *  Assign a new I gain value.
 */
void pid_seti ( pid_struct *pid, double I )  {
  pthread_mutex_lock(&mutex_pid);
  pid->I = I;
  pthread_mutex_unlock(&mutex_pid);
  return;
}


/**
 *  pid_setd
 *  Assign a new D gain value.
 */
void pid_setd ( pid_struct *pid, double D )  {
  pthread_mutex_lock(&mutex_pid);
  pid->D = D;
  pthread_mutex_unlock(&mutex_pid);
  return;
}


/**
 *  pid_update
 *  Calculate the control output for the PID entity.
 */
void pid_update ( pid_struct *pid )  {
  return;
}



