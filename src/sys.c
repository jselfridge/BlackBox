
//============================================================
//  sys.c
//  Justin M Selfridge
//============================================================
#include "sys.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_init
//  Initializes the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_init ( void )  {
  if(DEBUG)  printf("Initializing system \n");

  // Establish exit condition
  if(DEBUG)  printf("  Set system exit condition \n");
  struct sigaction sys_run;
  running = true;
  memset( &sys_run, 0, sizeof(sys_run) );
  sys_run.sa_handler = &sys_exit;
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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_update
//  Prints system debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
  //sys_sio();
  //sys_imuA();
  //sys_imuB();
  //sys_ahrs();
  sys_uart1();
  //sys_uart2();
  //sys_uart4();
  //sys_uart5();
  //sys_ctrl();

  // Complete debugging display 
  printf("  "); fflush(stdout);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_sio
//  Prints system input/output values to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_sio ( void )  {

  // Loop counter
  ushort i;

  // Input signals
  pthread_mutex_lock(&mutex_input);
  //for ( i=0; i<4; i++ )  printf("%5d ",   input.reg[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<6; i++ )  printf("%4d ",   input.pwm[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<6; i++ )  printf("%5.2f ", input.norm[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_input);

  // Output signals
  pthread_mutex_lock(&mutex_output);
  //for ( i=0; i<4; i++ )  printf("%5d ",   output.reg[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<6; i++ )  printf("%4d ",   output.pwm[i]  );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%5.2f ", output.norm[i] );  printf("   ");  fflush(stdout);
  printf("%5.2f ", output.norm[0] );  printf("   ");  fflush(stdout);
  printf("%5.2f ", output.norm[1] );  printf("   ");  fflush(stdout);
  printf("%5.2f ", output.norm[4] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_output);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_imuA
//  Prints IMUA debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_imuA ( void )  {

  // Check that IMUA is in use
  if (USE_IMUA) {

  // Loop counter
  ushort i;

  // Gyroscope data
  pthread_mutex_lock(&mutex_gyrA);
  //for ( i=0; i<3; i++ )  printf("%6d ",   gyrA.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%9.2f ", gyrA.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", gyrA.cal[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_gyrA);

  // Accelerometer data
  pthread_mutex_lock(&mutex_accA);
  //for ( i=0; i<3; i++ )  printf("%6d ",   accA.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%9.2f ", accA.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", accA.cal[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_accA);

  // Magnetometer data
  pthread_mutex_lock(&mutex_magA);
  //for ( i=0; i<3; i++ )  printf("%4d ",   magA.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.2f ", magA.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", magA.cal[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_magA);

  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_imuB
//  Prints IMUB debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_imuB ( void )  {

  // Check that IMUB is in use
  if (USE_IMUB) {

  // Loop counter
  ushort i;

  // Gyroscope data
  pthread_mutex_lock(&mutex_gyrB);
  //for ( i=0; i<3; i++ )  printf("%6d ",   gyrB.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%9.2f ", gyrB.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", gyrB.cal[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_gyrB);

  // Accelerometer data
  pthread_mutex_lock(&mutex_accB);
  //for ( i=0; i<3; i++ )  printf("%6d ",   accB.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%9.2f ", accB.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", accB.cal[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_accB);

  // Magnetometer data
  pthread_mutex_lock(&mutex_magB);
  //for ( i=0; i<3; i++ )  printf("%4d ",   magB.raw[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%7.2f ", magB.avg[i] );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%6.3f ", magB.cal[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_magB);

  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_ahrs
//  Prints AHRS debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_ahrs ( void )  {

  // Loop counter
  ushort i;

  /*// Quaternion data
  pthread_mutex_lock(&mutex_quat);
  for ( i=0; i<4; i++ )  printf("%7.4f ", ahrs.quat[i]  );  printf("   ");  fflush(stdout);
  for ( i=0; i<4; i++ )  printf("%7.4f ", ahrs.dquat[i] );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_quat);*/

  // Euler data
  pthread_mutex_lock(&mutex_eul);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahrs.eul[i]  * (180.0/PI) );  printf("   ");  fflush(stdout);
  for ( i=0; i<3; i++ )  printf("%7.2f ", ahrs.deul[i] * (180.0/PI) );  printf("   ");  fflush(stdout);
  pthread_mutex_unlock(&mutex_eul); 

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_uart1
//  Prints UART1 debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_uart1 ( void )  {
  if (UART1_ENABLED)  {
  printf( "TX1:  %s    ", uart1.txdata );  fflush(stdout);
  printf( "RX1:  %s    ", uart1.rxdata );  fflush(stdout);
  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_uart2
//  Prints UART2 debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_uart2 ( void )  {
  if (UART2_ENABLED)  {
  printf( "TX2: %s    ", uart2.txdata );  fflush(stdout);
  printf( "RX2: %s    ", uart2.rxdata );  fflush(stdout);
  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_uart4
//  Prints UART4 debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_uart4 ( void )  {
  if (UART4_ENABLED)  {
  printf( "TX4:  %s    ", uart4.txdata );  fflush(stdout);
  printf( "RX4:  %s    ", uart4.rxdata );  fflush(stdout);
  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_uart5
//  Prints UART5 debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_uart5 ( void )  {
  if (UART5_ENABLED)  {
  printf( "TX5: %s    ", uart5.txdata );  fflush(stdout);
  printf( "RX5: %s    ", uart5.rxdata );  fflush(stdout);
  }
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_ctrl
//  Prints controller values to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_ctrl ( void )  {

  // Loop counter
  //ushort i;

  // Control signals
  //pthread_mutex_lock(&mutex_ctrl);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.perr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.ierr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<3; i++ )  printf("%5.2f ", ctrl.derr[i] );  printf("   ");  fflush(stdout);
  //for ( i=0; i<4; i++ )  printf("%5.2f ", ctrl.cmd[i]  );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.bank    *(180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.climb   *(180.0/PI) );  printf("   ");  fflush(stdout);
  //printf("%5.2f ", ctrl.heading *(180.0/PI) );  printf("   ");  fflush(stdout);
  //pthread_mutex_unlock(&mutex_ctrl);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_exit
//  Code that runs prior to exiting the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_exit (  )  {

  // Change exit status
  running = false;
  usleep(200000);

  // Exit subsystems
  if(DEBUG)  printf("\n\n--- Exit BlackBox program --- \n");
  tmr_exit();
  //--  DEBUGGING  --//
  //datalog.enabled = false;
  //log_close();
  //-----------------//
  //log_exit();
  //ctl_exit();
  gps_exit();
  uart_exit();
  ahrs_exit();
  imu_exit();
  flag_exit();
  sio_exit();

  // Shut everything down
  if(DEBUG)  printf("Program complete \n");
  if( sigaction( SIGINT, &sys_signal, NULL ) == -1 )
    printf( "Error (sys_exit): Function 'sigaction' failed. \n" );
  if(!DEBUG)  system("shutdown -h now");
  kill( 0, SIGINT );

  return;
}



