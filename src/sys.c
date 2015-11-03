
//============================================================
//  sys.c
//  Justin M Selfridge
//============================================================
#include "sys.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_err
//  If error condition is true, prints a warning and exits.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_err ( bool cond, char* msg )  {
  if (cond) {  fprintf( stderr, "%s\n\n", msg );  exit(1);  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_init
//  Initializes the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_init ( void )  {
  if(DEBUG)  printf("Initializing system \n");

  // Initialize struct values
  sys.running = true;
  sys.ret = 0;

  // Establish exit condition
  struct sigaction sys_run;
  if(DEBUG)  printf("  Setting system exit condition \n");
  memset( &sys_run, 0, sizeof(sys_run) );
  sys_run.sa_handler = &sys_exit;
  sys.ret = sigaction( SIGINT, &sys_run, NULL );
  sys_err( sys.ret == -1, "Error (sys_init): Function 'sigaction' failed." );

  // Lock and reserve memory
  if(DEBUG)  printf("  Locking and reserving memory \n");
  sys.ret = mlockall( MCL_CURRENT | MCL_FUTURE );
  sys_err( sys.ret, "Error (sys_init): Failed to lock memory." );
  mallopt( M_TRIM_THRESHOLD, -1 );
  mallopt( M_MMAP_MAX, 0 );
  sys_memory(SYS_STACK);

  // Establish realtime priority
  //if(!DEBUG) {
  //struct sched_param sys_priority;
  //memset( &sys_priority, 0, sizeof(sys_priority) );
  //sys_priority.sched_priority = 99;
  //ret = sched_setscheduler( 0, SCHED_FIFO, &sys_priority );
  //sys_err( ret == -1, "Error (sys_init): Function 'sched_setscheduler' failed." );
  // }

  return;
}

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_loop
//  Executes instructions at timed intervals.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_loop ( void )  {

  //ushort i;
  timer_start();
  //for ( i=0; i<6; i++ )  sys.input[i] = pru_read_pulse(i);
  //imu_sample(&imu1);
  //ctrl_law();
  //for ( i=0; i<4; i++ )  pru_send_pulse( i, 1000 );
  //for ( i=0; i<4; i++ )  pru_send_pulse( i, sys.output[i] );
  timer_finish();
  log_write();
  if (DEBUG)  sys_debug();

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_debug
//  Prints system debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_debug (  )  {

  // Loop counter
  //ushort i;

  // Time values
  printf("\r");  fflush(stdout);
  printf("%6.3f  ", t.runtime );  fflush(stdout);
  printf("%09ld  ", t.start_nano );  fflush(stdout);
  printf("%09ld  ", t.dur );  fflush(stdout);
  printf("%4.2f  ", t.percent );  fflush(stdout);
  if (t.percent<1.0) {  printf("_    ");  fflush(stdout);  }
  else               {  printf("X    ");  fflush(stdout);  }

  // Input/Output values
  //for ( i=0; i<6; i++ )  printf("%06.1f ", sys.input[i]  );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%06.1f ", sys.output[i] );  printf("   ");

  // IMU1 heading status
  //printf("%012ld ", imu1.rawQuat[0] );  printf("   ");
  //printf("%6.1f ", heading *(180.0f/PI) );  printf("   ");

  // IMU2 heading status
  //printf("%012ld ", imu2.rawQuat[0] );  printf("   ");
  //printf("%6.1f ", heading *(180.0f/PI) );  printf("   ");

  // Raw sensor values - IMU1
  //for ( i=0; i<3; i++ )  printf("%04d ",   imu1.rawMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   imu1.rawAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   imu1.rawGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%012ld ", imu1.rawQuat[i] );  printf("   ");

  // Raw sensor values - IMU2
  //for ( i=0; i<3; i++ )  printf("%04d ",   imu2.rawMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   imu2.rawAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   imu2.rawGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%012ld ", imu2.rawQuat[i] );  printf("   ");

  // Moving average sensor values - IMU1
  //for ( i=0; i<3; i++ )  printf("%07.2f ",   imu1.avgMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ",   imu1.avgAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ",   imu1.avgGyro[i] );  printf("   ");

  // Moving average sensor values - IMU2
  //for ( i=0; i<3; i++ )  printf("%07.2f ",   imu2.avgMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ",   imu2.avgAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%09.2f ",   imu2.avgGyro[i] );  printf("   ");

  // Normalized sensor values - IMU1
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu1.normMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu1.normAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu1.normGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%7.4f ", imu1.normQuat[i] );  printf("   ");

  // Normalized sensor values - IMU2
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu2.normMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu2.normAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", imu2.normGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%7.4f ", imu2.normQuat[i] );  printf("   ");

  // Data fusion values - IMU1
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu1.Quat[i]              );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu1.dQuat[i]             );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu1.Eul[i]  *(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu1.dEul[i] *(180.0f/PI) );  printf("   ");

  // Data fusion values - IMU2
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu2.Quat[i]              );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%6.3f ", imu2.dQuat[i]             );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu2.Eul[i]  *(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", imu2.dEul[i] *(180.0f/PI) );  printf("   ");

  // Control values
  //printf( "%6.3f      ", ctrl.heading*(180.0/PI) );
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[X][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[Y][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%05.2f ", ctrl.err[Z][i]*(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf( "%07.2f ", ctrl.input[i] *(180.0f/PI) );  printf("   ");

  // Finish print loop
  printf("    ");
  fflush(stdout);

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_exit
//  Code that runs prior to exiting the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_exit (  )  {
  if(DEBUG)  printf("\n\nExiting program \n");
  thread_exit();
  //timer_exit();
  //usleep(200000);
  //imu_exit();
  //pru_exit();
  led_off(LED_MPU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);
  if(DEBUG)  printf("Program complete \n\n");
  sys.ret = sigaction( SIGINT, &sys_signal, NULL );
  sys_err( sys.ret == -1, "Error (sys_exit): Function 'sigaction' failed." );
  kill( 0, SIGINT );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_memory
//  Reserves a block of memory exclusively for the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_memory ( int size )  {
  int i;
  char *buffer;   
  buffer = malloc(size);
  for ( i=0; i<size; i += sysconf(_SC_PAGESIZE) )  {  buffer[i] = 0;  }
  free(buffer);
  //unsigned char temp[size];
  //memset( temp, 0, size );
  return;
}



