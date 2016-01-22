
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

  // Begin the program
  if(DEBUG)  printf("\n--- Begin BlackBox program ---\n");
  led_on(LED_IMU);  led_on(LED_PRU);  led_on(LED_LOG);  led_on(LED_MOT);

  // Initialize subsystems
  sys_init();
  imu_init();
  log_init();
  tmr_init();

  // Under development
  //pru_init();
  //ctrl_init();

  // Wait for exit condition
  while(running)  usleep(100000);


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Develop datalog file here first
  if(DEBUG)  printf("DATALOG DEBUG...\n");

  // Variables
  char *dir  = malloc(16);
  char *path = malloc(32);
  char *file = malloc(64);
  FILE *gyrf;
  
  // Find next available log directory
  ushort i;
  while (true) {
    i++;
    if      ( i<10   )  sprintf( dir, "Log_00%d", i );
    else if ( i<100  )  sprintf( dir, "Log_0%d",  i );
    else if ( i<1000 )  sprintf( dir, "Log_%d",   i );
    else    printf( "Error (log_XX): Exceeded maximum number of log directories. \n" );
    sprintf( file, "./log/%s/gyr.txt", dir );
    if ( access( file , F_OK ) == -1 )  break;
  }

  // Create new directory
  sprintf( path, "./log/%s/", dir );
  mkdir( path, 222 );

  // Create gyroscope datalog file
  sprintf( file, "%sgyr.txt", path );
  gyrf = fopen( file, "w" );
  if( gyrf == NULL )
    printf( "Error (log_XX): Cannot open 'gyr' file. \n" );
  fprintf( gyrf, 
    " Gtime        Gdur  \
    Grx     Gry     Grz    \
    Gax        Gay        Gaz       \
    Gcx      Gcy      Gcz     ");

  // Loop through data
  ulong j;
  ushort k;
  for ( j=0; j<log_gyr.count; j++ ) {
    fprintf( gyrf, "\n %011.6f  %06d    ", log_gyr.time[j], log_gyr.dur[j] );
    for ( k=0; k<3; k++ )  fprintf( gyrf, "%06d  ",   log_gyr.raw[j*3+k] );   fprintf( gyrf, "   " );
    for ( k=0; k<3; k++ )  fprintf( gyrf, "%09.2f  ", log_gyr.avg[j*3+k] );   fprintf( gyrf, "   " );
    for ( k=0; k<3; k++ )  fprintf( gyrf, "%07.4f  ", log_gyr.cal[j*3+k] );   fprintf( gyrf, "   " );
  }

  fclose(gyrf);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  // Run exit functions
  sys_exit();

  return 0;
}



