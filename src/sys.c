

#include "sys.h"
#include <malloc.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include "ahrs.h"
//#include "ctrl.h"
#include "ekf.h"
//#include "gps.h"
#include "imu.h"
#include "io.h"
#include "led.h"
#include "log.h"
#include "timer.h"



static void sys_io    ( void );
static void sys_imu   ( void );
static void sys_ahrs  ( void );
static void sys_ekf   ( void );
//static void sys_gps   ( void );
//static void sys_ctrl  ( void );
//static void sys_gcs   ( void );


/**
 *  sys_init
 *  Initializes the system.
 */
void sys_init ( void )  {
  if(DEBUG)  printf("Initializing system \n");

  // Turn off LED indicators
  led_off(LED_SIO);  led_off(LED_IMU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Establish exit condition
  if(DEBUG)  printf("  Set system exit condition \n");
  struct sigaction sys_run;
  memset( &sys_run, 0, sizeof(sys_run) );
  sys_run.sa_handler = &sys_quit;
  if( sigaction( SIGINT, &sys_run, NULL ) == -1 )
    printf( "Error (sys_init): Function 'sigaction' failed. \n" );

  // Establish realtime priority
  if(DEBUG)  printf("  Establish realtime priority \n");
  struct sched_param sp;
  sp.sched_priority = 98;
  if( sched_setscheduler( 0, SCHED_FIFO, &sp ) == -1 )
    printf( "Error (sys_init): Function 'sched_setscheduler' failed. \n" );

  // Prefault memory stack
  if(DEBUG)  printf("  Prefault memory stack \n");
  int i;
  char *buffer;   
  buffer = malloc(SYS_STACK);
  for ( i=0; i<SYS_STACK; i += sysconf(_SC_PAGESIZE) )  {  buffer[i] = 0;  }
  free(buffer);

  // Lock and reserve memory
  if(DEBUG)  printf("  Lock and reserve memory \n");
  if ( mlockall( MCL_CURRENT | MCL_FUTURE ) )
    printf( "Error (sys_init): Failed to lock memory. \n" );
  mallopt( M_TRIM_THRESHOLD, -1 );
  mallopt( M_MMAP_MAX, 0 );

  // Set global boolean conditions
  running = true;
  armed = false;

  return;
}


/**
 *  sys_exit
 *  Code that runs prior to exiting the system.
 */
void sys_exit ( void )  {
  if(DEBUG)  printf("Program complete \n");
  if( sigaction( SIGINT, &sys_signal, NULL ) == -1 )
    printf( "Error (sys_exit): Function 'sigaction' failed. \n" );
  if ( munlockall() )
    printf( "Error (sys_exit): Failed to unlock memory. \n" );
  if(!DEBUG)  system("shutdown -h now");
  kill( 0, SIGINT );
  return;
}


/**
 *  sys_quit
 *  Changes the 'running' status flag to begin closing threads. 
 */
void sys_quit (  )  {
  running = false;
  usleep(200000);
  return;
}


/**
 *  sys_update
 *  Prints system debugging messages to the terminal.
 */
void sys_update ( void )  {

  // Start debugging display
  printf("\r");  fflush(stdout);

  // Datalog status
  if (datalog.enabled)  printf(" Log %s: ", datalog.dir );
  else                  printf(" - - - -  ");
  fflush(stdout);

  // Time values
  float timestamp = (float) ( tmr_debug.start_sec + ( tmr_debug.start_usec / 1000000.0f ) - datalog.offset );
  printf("%6.1f    ", timestamp );  fflush(stdout);

  // Select data for display
  if(SYS_IO)     sys_io();
  if(SYS_IMU)    sys_imu();
  if(SYS_AHRS)   sys_ahrs();
  if(SYS_EKF)    sys_ekf();
  //if(SYS_GPS)    sys_gps();
  //if(SYS_GCS)    sys_gcs();
  //if(SYS_CTRL)   sys_ctrl();

  // Complete debugging display 
  printf("  "); fflush(stdout);

  return;
}


/**
 *  sys_io
 *  Prints input/output values to the terminal.
 */
static void sys_io ( void )  {

  // Loop counter
  ushort i;

  // Input signals
  pthread_mutex_lock(&input.mutex);
  //for ( i=0; i<6; i++ )  printf("%5d ",   input.reg[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<6; i++ )  printf("%4d ",   input.pwm[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<6; i++ )  printf("%5.2f ", input.norm[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&input.mutex);

  // Output signals
  pthread_mutex_lock(&output.mutex);
  //for ( i=0; i<4; i++ )  printf("%5d ",   output.reg[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%4d ",   output.pwm[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<4; i++ )  printf("%5.2f ", output.norm[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&output.mutex);

  // Quadrotor output signals
  //pthread_mutex_lock(&output.mutex);
  //printf("%5.2f ", output.norm[0] );
  //printf("%5.2f ", output.norm[1] );
  //printf("%5.2f ", output.norm[4] );
  //printf("%5.2f ", output.norm[5] );
  //printf("   ");  fflush(stdout);
  //pthread_mutex_unlock(&output.mutex);

  return;
}


/**
 *  sys_imu
 *  Prints IMU debugging messages to the terminal.
 */
static void sys_imu ( void )  {

  // Loop counter
  //ushort i;

  // Check that IMUA is in use
  if (IMUA_ENABLED) {

  // Gyroscope data
  pthread_mutex_lock(&gyrA.mutex);
  //for ( i=0; i<3; i++ )  printf("%6d ",   gyrA.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", gyrA.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", gyrA.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&gyrA.mutex);

  // Accelerometer data
  pthread_mutex_lock(&accA.mutex);
  //for ( i=0; i<3; i++ )  printf("%6d ",   accA.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accA.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accA.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&accA.mutex);

  // Magnetometer data
  pthread_mutex_lock(&magA.mutex);
  //for ( i=0; i<3; i++ )  printf("%4d ",   magA.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magA.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magA.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&magA.mutex);

  // Complimentry filter data
  pthread_mutex_lock(&imuA.mutex);
  double Ra = imuA.roll  * ( 180.0 / PI );
  double Pa = imuA.pitch * ( 180.0 / PI );
  printf("%6.3f %6.3f ", Ra, Pa );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&imuA.mutex);

  }

  // Check that IMUB is in use
  if (IMUB_ENABLED) {

  // Gyroscope data
  pthread_mutex_lock(&gyrB.mutex);
  //for ( i=0; i<3; i++ )  printf("%6d ",   gyrB.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", gyrB.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", gyrB.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&gyrB.mutex);

  // Accelerometer data
  pthread_mutex_lock(&accB.mutex);
  //for ( i=0; i<3; i++ )  printf("%6d ",   accB.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accB.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accB.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&accB.mutex);

  // Magnetometer data
  pthread_mutex_lock(&magB.mutex);
  //for ( i=0; i<3; i++ )  printf("%4d ",   magB.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magB.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magB.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&magB.mutex);

  // Complimentry filter data
  pthread_mutex_lock(&imuB.mutex);
  double Rb = imuB.roll  * ( 180.0 / PI );
  double Pb = imuB.pitch * ( 180.0 / PI );
  printf("%6.3f %6.3f ", Rb, Pb );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&imuB.mutex);

  }

  return;
}


/**
 *  sys_ahrs
 *  Prints AHRS debugging messages to the terminal.
 */
static void sys_ahrs ( void )  {

  // Loop counter
  ushort i;

  // AHRS A data
  if (IMUA_ENABLED)  {
  pthread_mutex_lock(&ahrsA.mutex);
  //for ( i=0; i<4; i++ )  printf("%7.4f ", ahrsA.quat[i]  );              printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%7.4f ", ahrsA.dquat[i] );              printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahrsA.eul[i]  * (180.0/PI) );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahrsA.deul[i] * (180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&ahrsA.mutex);
  }

  // AHRS B data
  if (IMUB_ENABLED)  {
  pthread_mutex_lock(&ahrsB.mutex);
  //for ( i=0; i<4; i++ )  printf("%7.4f ", ahrsB.quat[i]  );              printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%7.4f ", ahrsB.dquat[i] );              printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahrsB.eul[i]  * (180.0/PI) );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahrsB.deul[i] * (180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&ahrsB.mutex);
  }

  return;
}


/**
 *  sys_ekf
 *  Prints EKF debugging messages to the terminal.
 */
static void sys_ekf ( void )  {

  // Loop counter
  //ushort i;

  pthread_mutex_lock(&ekf.mutex);
  printf("%7.2f ", mat_get(ekf.x,1,1) * (180.0/PI) );  printf("   ");  fflush(stdout);
  printf("%7.2f ", mat_get(ekf.x,2,1) * (180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%7.2f ", ekf.x[1] * (180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%7.2f ", ekf.x[2] * (180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%7.2f ", ekf.x[3] * (180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%7.2f ", ekf.x[4] * (180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%7.2f ", ekf.x[5] * (180.0/PI) );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.2f ", ekf.x[i+0] * (180.0/PI) );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.2f ", ekf.x[i+3] * (180.0/PI) );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.2f ", ekf.x[i+6] * (180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&ekf.mutex);

  return;
}


/**
 *  sys_gps
 *  Prints GPS debugging messages to the terminal.
 */
//static void sys_gps ( void )  {
  /*
  pthread_mutex_lock(&mutex_gps);
  printf("GPS msg:    %s ", gps.msg );  printf("   ");  fflush(stdout);
  //printf("lat: %s    ",     gps.lat     );
  //printf("lon: %s    ",     gps.lon     );
  //printf("alt: %s    ",     gps.alt     );
  //printf("heading: %s    ", gps.heading );
  //printf("speed: %s    ",   gps.speed   );
  //printf("numsat: %s    ",  gps.numsat  );
  //fflush(stdout);
  pthread_mutex_unlock(&mutex_gps);
  */
//  return;
//}


/**
 *  sys_ctrl
 *  Prints controller values to the terminal.
 */
/*
static void sys_ctrl ( void )  {

  // Loop counter
  ushort i;

  // Control signals
  pthread_mutex_lock(&ctrl.mutex);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.pgain[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.igain[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.dgain[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.thrl[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%5.2f ", ctrl.range[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.perr[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.ierr[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.derr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%5.2f ", ctrl.cmd[i]  );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.bank    *(180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.climb   *(180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.heading *(180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&ctrl.mutex);

  return;
}
*/

/**
 *  sys_gcs
 *  Prints ground control debugging messages to the terminal.
 */
/*
static void sys_gcs ( void )  {
  // Add code as needed...
  return;
}
*/



