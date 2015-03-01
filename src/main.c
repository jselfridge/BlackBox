
//============================================================
//  main.c
//  Justin M Selfridge
//============================================================
#include "main.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  main
//  Primary code that runs the UAV avionics.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main ( void )  {
  if(DEBUG)  printf("Begin BlackBox program \n");
  //uav_init();
  //timer_init();
  //pru_init();
  //mpu1.bus = 1;
  //mpu2.bus = 2;
  //mpu_init(&mpu1);
  //mpu_init(&mpu2);
  //ctrl_init();
  //timer_begin();
  //while(uav.running) {}
  //uav_exit();
  return 0;
}


/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uav_err
//  If error condition is true, prints a warning and exits.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uav_err ( bool cond, char* msg )  {
  if (cond) {  fprintf( stderr, "%s\n\n", msg );  exit(1);  }
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uav_init
//  Initializes the UAV program.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uav_init ( void )  {
  if(DEBUG)  printf("Initializing UAV \n");

  // Establish exit condition
  struct sigaction uav_run;
  if(DEBUG)  printf("  Setting exit condition... ");
  memset( &uav_run, 0, sizeof(uav_run) );
  uav_run.sa_handler = &uav_exit;
  ret = sigaction( SIGINT, &uav_run, NULL );
  uav_err( ret == -1, "Error (main): Function'sigaction' failed." );
  if(DEBUG)  printf("complete \n");

  // Establish realtime priority
  if(!DEBUG) {
  struct sched_param uav_priority;
  memset( &uav_priority, 0, sizeof(uav_priority) );
  uav_priority.sched_priority = 90;
  ret = sched_setscheduler( 0, SCHED_FIFO, &uav_priority );
  uav_err( ret == -1, "Error (main): Function 'sched_setscheduler' failed." );
  }

  // Lock and reserve memory
  if(DEBUG)  printf("  Locking and reserving memory... ");
  ret = mlockall( MCL_CURRENT | MCL_FUTURE );
  uav_err( ret == -1, "Error (uav_init): Failed to lock memory." );
  uav_memory();
  if(DEBUG)  printf("complete \n");

  // Set LEDs
  led_off(LED_MPU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);

  // Log file status
  uav.fileopen = false;
  uav.logdata  = false;
  uav.running  = true;

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uav_loop
//  Executes instructions at timed intervals.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uav_loop ( void )  {

  // Loop counter
  ushort i;

  // Start timing loop
  timer_start();

  // Get new radio inputs
  for ( i=0; i<8; i++ )   uav.radio[i] = pru_read_pulse(i);

  // Get new MPU data
  mpu_sample(&mpu1);
  //mpu_sample(&mpu2);

  // Run control law
  ctrl_law();

  // Assign motor values
  //for ( i=0; i<8; i++ )   pru_send_pulse( i, 1000 );    //  Manually turn off motors for debugging
  for ( i=0; i<8; i++ )   pru_send_pulse( i, uav.servo[i] );

  // Finish timing loop 
  timer_finish();

  // Write to log file
  log_write();

  // Debugging print statement
  if (DEBUG)  uav_debug();

  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uav_debug
//  Prints debugging messages to the terminal.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uav_debug (  )  {

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

  // Input output values
  for ( i=0; i<4; i++ )  printf("%04d ", uav.radio[i] );  printf("   ");
  for ( i=0; i<4; i++ )  printf("%04d ", uav.servo[i] );  printf("   ");

  // MPU1 status
  //printf("%012ld ", mpu1.rawQuat[0] );  printf("   ");
  //printf("%6.1f ", heading *(180.0f/PI) );  printf("   ");

  // MPU2 status
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
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu1.calMag[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu1.calAcc[i]  );  printf("   ");
  //for ( i=0; i<3; i++ )  printf("%7.4f ", mpu1.calGyro[i] );  printf("   ");
  //for ( i=0; i<4; i++ )  printf("%7.4f ", mpu1.calQuat[i] );  printf("   ");

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
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uav_exit
//  Code that runs prior to exiting the program.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uav_exit (  )  {
  if(DEBUG)  printf("\n\nExiting program \n");
  timer_exit();
  usleep(200000);
  mpu_exit();
  pru_exit();
  led_off(LED_MPU);  led_off(LED_PRU);  led_off(LED_LOG);  led_off(LED_MOT);
  if(DEBUG)  printf("Program complete \n");
  ret = sigaction( SIGINT, &exit_signal, NULL );
  uav_err( ret == -1, "Error (uav_exit): Function 'sigaction' failed." );
  kill( 0, SIGINT );
  return;
}
*/
/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  uav_memory
//  Reserves a block of memory exclusively for UAV program.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void uav_memory ( void )  {
  int size = 1024*1024;
  unsigned char temp[size];
  memset( temp, 0, size );
  return;
}
*/


