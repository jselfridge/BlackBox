
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
  FILE *gyrf, *accf, *magf;
  ushort i;
  ulong j;
  ushort k;

  // Find next available log directory
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

  // Create accelerometer datalog file
  sprintf( file, "%sacc.txt", path );
  accf = fopen( file, "w" );
  if( accf == NULL )
    printf( "Error (log_XX): Cannot open 'acc' file. \n" );
  fprintf( accf, 
    " Atime        Adur  \
    Arx     Ary     Arz    \
    Aax        Aay        Aaz       \
    Acx      Acy      Acz     ");

  // Create magnetometer datalog file
  sprintf( file, "%smag.txt", path );
  magf = fopen( file, "w" );
  if( magf == NULL )
    printf( "Error (log_XX): Cannot open 'mag' file. \n" );
  fprintf( magf, 
    " Mtime        Mdur  \
    Mrx     Mry     Mrz    \
    Max        May        Maz       \
    Mcx      Mcy      Mcz     ");

  // Loop through gyroscope data
  for ( j=0; j<log_gyr.count; j++ ) {
    fprintf( gyrf, "\n %011.6f  %06d    ", log_gyr.time[j], log_gyr.dur[j] );
    for ( k=0; k<3; k++ )  fprintf( gyrf, "%06d  ",   log_gyr.raw[j*3+k] );   fprintf( gyrf, "   " );
    for ( k=0; k<3; k++ )  fprintf( gyrf, "%09.2f  ", log_gyr.avg[j*3+k] );   fprintf( gyrf, "   " );
    for ( k=0; k<3; k++ )  fprintf( gyrf, "%07.4f  ", log_gyr.cal[j*3+k] );   fprintf( gyrf, "   " );
  }

  // Loop through accelerometer data
  for ( j=0; j<log_acc.count; j++ ) {
    fprintf( accf, "\n %011.6f  %06d    ", log_acc.time[j], log_acc.dur[j] );
    for ( k=0; k<3; k++ )  fprintf( accf, "%06d  ",   log_acc.raw[j*3+k] );   fprintf( accf, "   " );
    for ( k=0; k<3; k++ )  fprintf( accf, "%09.2f  ", log_acc.avg[j*3+k] );   fprintf( accf, "   " );
    for ( k=0; k<3; k++ )  fprintf( accf, "%07.4f  ", log_acc.cal[j*3+k] );   fprintf( accf, "   " );
  }

  // Loop through magnetometer data
  for ( j=0; j<log_mag.count; j++ ) {
    fprintf( magf, "\n %011.6f  %06d    ", log_mag.time[j], log_mag.dur[j] );
    for ( k=0; k<3; k++ )  fprintf( magf, "%06d  ",   log_mag.raw[j*3+k] );   fprintf( magf, "   " );
    for ( k=0; k<3; k++ )  fprintf( magf, "%09.2f  ", log_mag.avg[j*3+k] );   fprintf( magf, "   " );
    for ( k=0; k<3; k++ )  fprintf( magf, "%07.4f  ", log_mag.cal[j*3+k] );   fprintf( magf, "   " );
  }

  // Close files
  fclose(gyrf);
  fclose(accf);
  fclose(magf);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  // Run exit functions
  sys_exit();

  return 0;
}



