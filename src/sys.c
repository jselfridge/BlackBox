

#include "sys.h"
#include <malloc.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include "ahrs.h"
#include "filter.h"
#include "flag.h"
#include "gps.h"
#include "imu.h"
#include "io.h"
#include "led.h"
#include "log.h"
#include "timer.h"


static void sys_io      ( void );
static void sys_filter  ( void );
static void sys_imuA    ( void );
static void sys_imuB    ( void );
static void sys_ahrs    ( void );
static void sys_gps     ( void );
static void sys_ctrl    ( void );


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
  if(0)  sys_io();
  if(0)  sys_filter();
  if(1)  if(IMUA_ENABLED)  sys_imuA();
  if(1)  if(IMUB_ENABLED)  sys_imuB();
  if(1)  sys_ahrs();
  if(0)  sys_gps();
  if(0)  sys_ctrl();

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
  pthread_mutex_lock(&mutex_input);
  //for ( i=0; i<6; i++ )  printf("%5d ",   input.reg[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<6; i++ )  printf("%4d ",   input.pwm[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<6; i++ )  printf("%5.2f ", input.norm[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_input);

  // Output signals
  pthread_mutex_lock(&mutex_output);
  //for ( i=0; i<4; i++ )  printf("%5d ",   output.reg[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%4d ",   output.pwm[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<6; i++ )  printf("%5.2f ", output.norm[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_output);

  /*
  pthread_mutex_lock(&mutex_output);
  printf("%5.2f ", output.norm[0] );
  printf("%5.2f ", output.norm[1] );
  printf("%5.2f ", output.norm[4] );
  printf("%5.2f ", output.norm[5] );
  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_output);
  */

  return;
}


/**
 *  sys_filter
 *  Prints filter parameter values to the terminal.
 */
static void sys_filter ( void )  {

  // IMUA filters
  if (IMUA_ENABLED) {
    //printf("%6.1f ", filter_gyrA.freq );  printf("%6.1f ", filter_accA.freq );  printf("%6.1f ", filter_magA.freq );  printf("   ");  fflush(stdout);  
    //printf("%5d ",   filter_gyrA.hist );  printf("%5d ",   filter_accA.hist );  printf("%5d ",   filter_magA.hist );  printf("   ");  fflush(stdout);  
  }

  // IMUB filters
  if (IMUB_ENABLED) {
    //printf("%6.1f ", filter_gyrB.freq );  printf("%6.1f ", filter_accB.freq );  printf("%6.1f ", filter_magB.freq );  printf("   ");  fflush(stdout);  
    //printf("%5d ",   filter_gyrB.hist );  printf("%5d ",   filter_accB.hist );  printf("%5d ",   filter_magB.hist );  printf("   ");  fflush(stdout);  
  }

  return;
}


/**
 *  sys_imuA
 *  Prints IMUA debugging messages to the terminal.
 */
static void sys_imuA ( void )  {

  // Check that IMUA is in use
  if (IMUA_ENABLED) {

  // Loop counter
  ushort i;

  // Gyroscope data
  pthread_mutex_lock(&mutex_gyrA);
  //for ( i=0; i<3; i++ )  printf("%6d ",   gyrA.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", gyrA.scaled[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", gyrA.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_gyrA);

  // Accelerometer data
  pthread_mutex_lock(&mutex_accA);
  //for ( i=0; i<3; i++ )  printf("%6d ",   accA.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accA.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accA.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_accA);

  // Magnetometer data
  pthread_mutex_lock(&mutex_magA);
  //for ( i=0; i<3; i++ )  printf("%4d ",   magA.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magA.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magA.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_magA);

  }

  return;
}


/**
 *  sys_imuB
 *  Prints IMUB debugging messages to the terminal.
 */
static void sys_imuB ( void )  {

  // Check that IMUB is in use
  if (IMUB_ENABLED) {

  // Loop counter
  ushort i;

  // Gyroscope data
  pthread_mutex_lock(&mutex_gyrB);
  //for ( i=0; i<3; i++ )  printf("%6d ",   gyrB.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", gyrB.scaled[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", gyrB.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_gyrB);

  // Accelerometer data
  pthread_mutex_lock(&mutex_accB);
  //for ( i=0; i<3; i++ )  printf("%6d ",   accB.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accB.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", accB.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_accB);

  // Magnetometer data
  pthread_mutex_lock(&mutex_magB);
  //for ( i=0; i<3; i++ )  printf("%4d ",   magB.raw[i]    );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magB.scaled[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%6.3f ", magB.filter[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_magB);

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

  // Averaged IMU data
  pthread_mutex_lock(&mutex_ahrs);
  for ( i=0; i<3; i++ )  printf("%7.4f ", ahrs.gyr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.4f ", ahrs.acc[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.4f ", ahrs.mag[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_ahrs);

  // Quaternion data
  pthread_mutex_lock(&mutex_quat);
  //for ( i=0; i<4; i++ )  printf("%7.4f ", ahrs.quat[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%7.4f ", ahrs.dquat[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_quat);

  // Euler data
  pthread_mutex_lock(&mutex_eul);
  //for ( i=0; i<3; i++ )  printf("%7.2f ", ahrs.eul[i]  * (180.0/PI) );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.2f ", ahrs.deul[i] * (180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_eul); 

  return;
}


/**
 *  sys_gps
 *  Prints GPS debugging messages to the terminal.
 */
static void sys_gps ( void )  {

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

  return;
}


/**
 *  sys_ctrl
 *  Prints controller values to the terminal.
 */
static void sys_ctrl ( void )  {

  // Loop counter
  //ushort i;

  // Control signals
  pthread_mutex_lock(&mutex_ctrl);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.pgain[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.igain[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.dgain[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.thrl[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%5.2f ", ctrl.scale[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.perr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.ierr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.derr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%5.2f ", ctrl.cmd[i]  );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.bank    *(180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.climb   *(180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.heading *(180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_ctrl);

  return;
}



