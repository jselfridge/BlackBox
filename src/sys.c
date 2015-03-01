
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

  // Establish exit condition
  struct sigaction sys_run;
  if(DEBUG)  printf("  Setting system exit condition... ");
  memset( &sys_run, 0, sizeof(sys_run) );
  sys_run.sa_handler = &sys_exit;
  ret = sigaction( SIGINT, &sys_run, NULL );
  sys_err( ret == -1, "Error (sys_init): Function 'sigaction' failed." );
  if(DEBUG)  printf("complete \n");

  // Establish realtime priority
  if(!DEBUG) {
  struct sched_param sys_priority;
  memset( &sys_priority, 0, sizeof(sys_priority) );
  sys_priority.sched_priority = 90;
  ret = sched_setscheduler( 0, SCHED_FIFO, &sys_priority );
  sys_err( ret == -1, "Error (sys_init): Function 'sched_setscheduler' failed." );
  }

  // Lock and reserve memory
  if(DEBUG)  printf("  Locking and reserving memory... ");
  ret = mlockall( MCL_CURRENT | MCL_FUTURE );
  sys_err( ret == -1, "Error (sys_init): Failed to lock memory." );
  sys_memory();
  if(DEBUG)  printf("complete \n");

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_loop
//  Executes instructions at timed intervals.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_loop ( void )  {

  // Loop counter
  ushort i;

  // Start timing loop
  timer_start();

  // Get new radio inputs
  for ( i=0; i<10; i++ )   sys.input[i] = pru_read_pulse(i);

  // Get new MPU data
  mpu_sample(&mpu1);
  //mpu_sample(&mpu2);  // Save data fussion for later
  mpu_raw(&mpu2);
  mpu_norm(&mpu2);

  // Run control law
  ctrl_law();

  // Assign motor values
  for ( i=0; i<10; i++ )   pru_send_pulse( i, sys.output[i] );
  //for ( i=0; i<10; i++ )   pru_send_pulse( i, 1000 );    //  Manually turn off motors for debugging
  //for ( i=0; i<10; i++ )   pru_send_pulse( i, sys.input[i] );

  // Finish timing loop 
  timer_finish();

  // Write to log file
  log_write();

  // Debugging print statement
  if (DEBUG)  sys_debug();

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_debug
//  Prints system debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_debug (  )  {

  // Loop counter
  ushort i;

  // Time values
  printf("\r");
  printf("%6.3f  ", t.runtime );
  //printf("%09ld  ", t.start_nano );
  //printf("%09ld  ", t.dur );
  printf("%4.2f  ", t.percent );
  if (t.percent<1.0) printf("_    ");
  else               printf("X    ");

  // Input/Output values
  for ( i=0; i<4; i++ )  printf("%04d ", sys.input[i]  );  printf("   ");
  for ( i=0; i<4; i++ )  printf("%04d ", sys.output[i] );  printf("   ");

  // MPU1 heading status
  //printf("%012ld ", mpu1.rawQuat[0] );  printf("   ");
  //printf("%6.1f ", heading *(180.0f/PI) );  printf("   ");

  // MPU2 heading status
  //printf("%012ld ", mpu2.rawQuat[0] );  printf("   ");
  //printf("%6.1f ", heading *(180.0f/PI) );  printf("   ");

  // Raw sensor values - MPU1
  //for ( i=0; i<3; i++ )  printf("%04d ",   mpu1.rawMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   mpu1.rawAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   mpu1.rawGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%012ld ", mpu1.rawQuat[i] );  printf("   ");

  // Raw sensor values - MPU2
  //for ( i=0; i<3; i++ )  printf("%04d ",   mpu2.rawMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   mpu2.rawAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%06d ",   mpu2.rawGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%012ld ", mpu2.rawQuat[i] );  printf("   ");

  // Calibrated sensor values - MPU1
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu1.normMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu1.normAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu1.normGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%7.4f ", mpu1.normQuat[i] );  printf("   ");

  // Calibrated sensor values - MPU2
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu2.calMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu2.calAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu2.calGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%7.4f ", mpu2.calQuat[i] );  printf("   ");

  // Data fusion values - MPU1
  //for ( i=0; i<4; i++ )  printf("%6.3f ", mpu1.Quat[i]              );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%6.3f ", mpu1.dQuat[i]             );  printf("   ");
  for ( i=0; i<3; i++ )  printf("%6.1f ", mpu1.Eul[i]  *(180.0f/PI) );  printf("   ");
  for ( i=0; i<3; i++ )  printf("%6.1f ", mpu1.dEul[i] *(180.0f/PI) );  printf("   ");

  // Data fusion values - MPU2
  //for ( i=0; i<4; i++ )  printf("%6.3f ", mpu2.Quat[i]              );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%6.3f ", mpu2.dQuat[i]             );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", mpu2.Eul[i]  *(180.0f/PI) );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%6.1f ", mpu2.dEul[i] *(180.0f/PI) );  printf("   ");

  // Finish print loop
  printf("    ");
  fflush(stdout);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_exit
//  Code that runs prior to exiting the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_exit (  )  {
  if(DEBUG)  printf("\n\nExiting program \n");
  timer_exit();
  usleep(200000);
  mpu_exit();
  pru_exit();
  led_off(LED_MPU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);
  if(DEBUG)  printf("Program complete \n");
  ret = sigaction( SIGINT, &sys_signal, NULL );
  sys_err( ret == -1, "Error (sys_exit): Function 'sigaction' failed." );
  kill( 0, SIGINT );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  sys_memory
//  Reserves a block of memory exclusively for the system.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void sys_memory ( void )  {
  int size = 1024*1024;
  unsigned char temp[size];
  memset( temp, 0, size );
  return;
}



