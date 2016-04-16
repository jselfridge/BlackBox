

#include "log.h"
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include "ahrs.h"
#include "gcs.h"
#include "gps.h"
#include "imu.h"
#include "io.h"
#include "led.h"
#include "sys.h"
#include "timer.h"


static void  log_save  ( void );
static void  log_free  ( void );
static void  log_close ( void );


/**
 *  log_init
 *  Runs on start up to initalize the datalog attributes.
 */
void log_init ( void )  {
  if(DEBUG)  printf("Initializing data logging \n");

  // Parameter value setup
  log_param.limit   =  LOG_MAX_PARAM;  
  log_param.time    =  malloc( sizeof(float) * log_param.limit               );
  log_param.values  =  malloc( sizeof(float) * log_param.limit * param_count );

  // Input signal setup
  log_input.limit   =  LOG_MAX_DUR * HZ_IO;
  log_input.time    =  malloc( sizeof(float)  * log_input.limit         );
  log_input.reg     =  malloc( sizeof(ushort) * log_input.limit * IN_CH );
  log_input.pwm     =  malloc( sizeof(ushort) * log_input.limit * IN_CH );
  log_input.norm    =  malloc( sizeof(float)  * log_input.limit * IN_CH );

  // Output signal setup
  log_output.limit  =  LOG_MAX_DUR * HZ_IO;
  log_output.time   =  malloc( sizeof(float)  * log_output.limit          );
  log_output.reg    =  malloc( sizeof(ushort) * log_output.limit * OUT_CH );
  log_output.pwm    =  malloc( sizeof(ushort) * log_output.limit * OUT_CH );
  log_output.norm   =  malloc( sizeof(float)  * log_output.limit * OUT_CH );

  // Gyroscope A setup
  log_gyrA.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_gyrA.time     =  malloc( sizeof(float) * log_gyrA.limit     );
  log_gyrA.dur      =  malloc( sizeof(ulong) * log_gyrA.limit     );
  log_gyrA.raw      =  malloc( sizeof(short) * log_gyrA.limit * 3 );
  log_gyrA.scaled   =  malloc( sizeof(float) * log_gyrA.limit * 3 );
  log_gyrA.filter   =  malloc( sizeof(float) * log_gyrA.limit * 3 );

  // Accelerometer A setup
  log_accA.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_accA.time     =  malloc( sizeof(float) * log_accA.limit     );
  log_accA.dur      =  malloc( sizeof(ulong) * log_accA.limit     );
  log_accA.raw      =  malloc( sizeof(short) * log_accA.limit * 3 );
  log_accA.scaled   =  malloc( sizeof(float) * log_accA.limit * 3 );
  log_accA.filter   =  malloc( sizeof(float) * log_accA.limit * 3 );

  // Magnetometer A setup
  log_magA.limit    =  LOG_MAX_DUR * HZ_IMU_SLOW;
  log_magA.time     =  malloc( sizeof(float) * log_magA.limit     );
  log_magA.dur      =  malloc( sizeof(ulong) * log_magA.limit     );
  log_magA.raw      =  malloc( sizeof(short) * log_magA.limit * 3 );
  log_magA.scaled   =  malloc( sizeof(float) * log_magA.limit * 3 );
  log_magA.filter   =  malloc( sizeof(float) * log_magA.limit * 3 );

  // Gyroscope B setup
  log_gyrB.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_gyrB.time     =  malloc( sizeof(float) * log_gyrB.limit     );
  log_gyrB.dur      =  malloc( sizeof(ulong) * log_gyrB.limit     );
  log_gyrB.raw      =  malloc( sizeof(short) * log_gyrB.limit * 3 );
  log_gyrB.scaled   =  malloc( sizeof(float) * log_gyrB.limit * 3 );
  log_gyrB.filter   =  malloc( sizeof(float) * log_gyrB.limit * 3 );

  // Accelerometer B setup
  log_accB.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_accB.time     =  malloc( sizeof(float) * log_accB.limit     );
  log_accB.dur      =  malloc( sizeof(ulong) * log_accB.limit     );
  log_accB.raw      =  malloc( sizeof(short) * log_accB.limit * 3 );
  log_accB.scaled   =  malloc( sizeof(float) * log_accB.limit * 3 );
  log_accB.filter   =  malloc( sizeof(float) * log_accB.limit * 3 );

  // Magnetometer B setup
  log_magB.limit    =  LOG_MAX_DUR * HZ_IMU_SLOW;
  log_magB.time     =  malloc( sizeof(float) * log_magB.limit     );
  log_magB.dur      =  malloc( sizeof(ulong) * log_magB.limit     );
  log_magB.raw      =  malloc( sizeof(short) * log_magB.limit * 3 );
  log_magB.scaled   =  malloc( sizeof(float) * log_magB.limit * 3 );
  log_magB.filter   =  malloc( sizeof(float) * log_magB.limit * 3 );

  // Attitude and Heading Reference System setup
  log_ahrs.limit    =  LOG_MAX_DUR * HZ_AHRS;
  log_ahrs.time     =  malloc( sizeof(float) * log_ahrs.limit     );
  log_ahrs.dur      =  malloc( sizeof(ulong) * log_ahrs.limit     );
  log_ahrs.gyr      =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.acc      =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.mag      =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.quat     =  malloc( sizeof(float) * log_ahrs.limit * 4 );
  log_ahrs.dquat    =  malloc( sizeof(float) * log_ahrs.limit * 4 );
  log_ahrs.eul      =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.deul     =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.bias     =  malloc( sizeof(float) * log_ahrs.limit * 3 );
  log_ahrs.fx       =  malloc( sizeof(float) * log_ahrs.limit     );
  log_ahrs.fz       =  malloc( sizeof(float) * log_ahrs.limit     );

  // Global Positioning System setup
  log_gps.limit     =  LOG_MAX_DUR * HZ_GPS;
  log_gps.time      =  malloc( sizeof(float) * log_gps.limit );
  log_gps.dur       =  malloc( sizeof(ulong) * log_gps.limit );
  log_gps.msg       =  malloc( sizeof(char)  * log_gps.limit * 96 );

  /* // Controller parameter setup
  log_ctrl.limit    =  LOG_MAX_DUR * HZ_CTRL;
  log_ctrl.time     =  malloc( sizeof(float) * log_ctrl.limit     );
  log_ctrl.dur      =  malloc( sizeof(ulong) * log_ctrl.limit     );
  log_ctrl.perr     =  malloc( sizeof(float) * log_ctrl.limit * 3 );
  log_ctrl.ierr     =  malloc( sizeof(float) * log_ctrl.limit * 3 );
  log_ctrl.derr     =  malloc( sizeof(float) * log_ctrl.limit * 3 );
  log_ctrl.cmd      =  malloc( sizeof(float) * log_ctrl.limit * 4 );
  */

  return;
}


/**
 *  log_exit
 *  Closes the data log files.
 */
void log_exit ( void )  {
  if(DEBUG)  printf("Close logs \n");
  // Add code as needed...
  return;
}


/**
 *  log_start
 *  Prepares the system for the next datalog sequence.
 */
void log_start ( void )  {

  // Clear counters for the new session
  log_param.count  = 0;
  log_input.count  = 0;
  log_output.count = 0;
  log_gyrA.count   = 0;
  log_accA.count   = 0;
  log_magA.count   = 0;
  log_gyrB.count   = 0;
  log_accB.count   = 0;
  log_magB.count   = 0;
  log_ahrs.count   = 0;
  log_gps.count    = 0;
  //log_ctrl.count   = 0;

  // Allocate dir/path/file memory
  datalog.dir  = malloc(16);
  datalog.path = malloc(32);
  char *file   = malloc(64);

  // Find next available log directory
  ushort i = 0;
  while (true) {
    i++;
    if      ( i<10   )  sprintf( datalog.dir, "00%d", i );
    else if ( i<100  )  sprintf( datalog.dir, "0%d",  i );
    else if ( i<1000 )  sprintf( datalog.dir, "%d",   i );
    else    printf( "Error (log_start): Exceeded maximum number of log directories. \n" );
    sprintf( file, "../Log/%s/notes.txt", datalog.dir );
    if ( access( file , F_OK ) == -1 )  break;
  }

  // Create new directory
  sprintf( datalog.path, "../Log/%s/", datalog.dir );
  mkdir( datalog.path, 222 );

  // Parameter datalog file
  sprintf( file, "%sparam.txt", datalog.path );
  datalog.param = fopen( file, "w" );
  if( datalog.param == NULL )  printf( "Error (log_start): Cannot generate 'param' file. \n" );
  fprintf( datalog.param, "   ParamTime " );

  // Input datalog file
  sprintf( file, "%sinput.txt", datalog.path );
  datalog.in = fopen( file, "w" );
  if( datalog.in == NULL )  printf( "Error (log_start): Cannot generate 'input' file. \n" );
  fprintf( datalog.in,  "      InTime       In01     In02     In03     In04     In05     In06     In07     In08     In09     In10" );

  // Output datalog file
  sprintf( file, "%soutput.txt", datalog.path );
  datalog.out = fopen( file, "w" );
  if( datalog.out == NULL )  printf( "Error (log_start): Cannot generate 'output' file. \n" );
  fprintf( datalog.out, "     OutTime      Out01    Out02    Out03    Out04    Out05    Out06    Out07    Out08    Out09    Out10" );

  // IMUA datalogs
  if (IMUA_ENABLED)  {

  // Gyroscope A datalog file
  sprintf( file, "%sgyrA.txt", datalog.path );
  datalog.gyrA = fopen( file, "w" );
  if( datalog.gyrA == NULL )  printf( "Error (log_start): Cannot generate 'gyrA' file. \n" );
  fprintf( datalog.gyrA,
    "     GyrTime  GyrDur   \
    Grx     Gry     Grz     \
    Gsx      Gsy      Gsz     \
    Gfx      Gfy      Gfz");

  // Accelerometer A datalog file
  sprintf( file, "%saccA.txt", datalog.path );
  datalog.accA = fopen( file, "w" );
  if( datalog.accA == NULL )  printf( "Error (log_start): Cannot generate 'accA' file. \n" );
  fprintf( datalog.accA, 
    "     AccTime  AccDur   \
    Arx     Ary     Arz     \
    Asx      Asy      Asz     \
    Afx      Afy      Afz");

  // Magnetometer A datalog file
  sprintf( file, "%smagA.txt", datalog.path );
  datalog.magA = fopen( file, "w" );
  if( datalog.magA == NULL )  printf( "Error (log_start): Cannot generate 'magA' file. \n" );
  fprintf( datalog.magA,
    "     MagTime  MagDur   \
    Mrx     Mry     Mrz     \
    Msx      Msy      Msz     \
    Mfx      Mfy      Mfz");

  }

  // IMUB datalogs
  if (IMUB_ENABLED )  {

  // Gyroscope B datalog file
  sprintf( file, "%sgyrB.txt", datalog.path );
  datalog.gyrB = fopen( file, "w" );
  if( datalog.gyrB == NULL )  printf( "Error (log_start): Cannot generate 'gyrB' file. \n" );
  fprintf( datalog.gyrB,
    "       Gtime    Gdur   \
    Grx     Gry     Grz     \
    Gsx      Gsy      Gsz     \
    Gfx      Gfy      Gfz");

  // Accelerometer B datalog file
  sprintf( file, "%saccB.txt", datalog.path );
  datalog.accB = fopen( file, "w" );
  if( datalog.accB == NULL )  printf( "Error (log_start): Cannot generate 'accB' file. \n" );
  fprintf( datalog.accB, 
    "       Atime    Adur   \
    Arx     Ary     Arz     \
    Asx      Asy      Asz     \
    Afx      Afy      Afz");

  // Magnetometer B datalog file
  sprintf( file, "%smagB.txt", datalog.path );
  datalog.magB = fopen( file, "w" );
  if( datalog.magB == NULL )  printf( "Error (log_start): Cannot generate 'magB' file. \n" );
  fprintf( datalog.magB,
    "       Mtime    Mdur   \
    Mrx     Mry     Mrz     \
    Msx      Msy      Msz     \
    Mfx      Mfy      Mfz");

  }

  // Attitude and heading reference system datalog file
  sprintf( file, "%sahrs.txt", datalog.path );
  datalog.ahrs = fopen( file, "w" );
  if( datalog.ahrs == NULL )  printf( "Error (log_start): Cannot generate 'ahrs' file. \n" );
  fprintf( datalog.ahrs,
    "       Rtime    Rdur     \
    Gx       Gy       Gz      \
    Ax       Ay       Az      \
    Mx       My       Mz      \
    Qw       Qx       Qy       Qz     \
    dQw      dQx      dQy      dQz      \
    Ex       Ey       Ez     \
    dEx      dEy      dEz      \
    bx       by       bz      \
    fx       fz");

  // GPS datalog file
  sprintf( file, "%sgps.txt", datalog.path );
  datalog.gps = fopen( file, "w" );
  if( datalog.gps == NULL )  printf( "Error (log_start): Cannot generate 'gps' file. \n" );
  fprintf( datalog.gps, "       Gtime    Gdur    Data    ");

  /* // Controller datalog file
  sprintf( file, "%sctrl.txt", datalog.path );
  datalog.ctrl = fopen( file, "w" );
  if( datalog.ctrl == NULL )  printf( "Error (log_start): Cannot generate 'ctrl' file. \n" );
  fprintf( datalog.ctrl,
    "       Ctime    Cdur    \
    EPX      EPY      EPZ     \
    EIX      EIY      EIZ     \
    EDX      EDY      EDZ      \
    CX       CY       CZ       CT" );*/

  // Determine start second
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  datalog.offset = timeval.tv_sec;
  datalog.setup = true;

  return;
}


/**
 *  log_finish
 *  Completes a datalog session by writing out collected data.
 */
void log_finish ( void )  {

  // Inidcate the download is in progress
  datalog.saving = true;
  led_blink( LED_LOG, 200, 200 );
  usleep(200000);

  // Transfer data from memory to file
  log_save();
  log_free();
  log_close();

  // Switch datalog setup flag
  datalog.setup = false;
  datalog.saving = false;
  led_off( LED_LOG );

  return;
}


/**
 *  log_save
 *  Saves each datalog to file.
 */
static void log_save ( void )  {

  // Local variables
  ushort i;
  ulong row;

  // Parameter data
  for ( row = 0; row < log_param.count; row++ ) {
    fprintf( datalog.param, "\n %011.6f    ", log_param.time[row] );
    //for ( i=0; i<param_count; i++ )  fprintf( datalog.param, "%data  ", log_param.value [ row* param_count +i ] );   fprintf( datalog.param, "   " );
  }

  // Input data
  for ( row = 0; row < log_input.count; row++ ) {
    fprintf( datalog.in, "\n %011.6f    ", log_input.time[row] );
    for ( i=0; i<10; i++ )  fprintf( datalog.in, "%7.4f  ", log_input.norm [ row*10 +i ] );   fprintf( datalog.in, "   " );
  }

  // Output data
  for ( row = 0; row < log_output.count; row++ ) {
    fprintf( datalog.out, "\n %011.6f    ", log_output.time[row] );
    for ( i=0; i<10; i++ )  fprintf( datalog.out, "%7.4f  ", log_output.norm [ row*10 +i ] );   fprintf( datalog.out, "   " );
  }

  // IMU A datalogs
  if(IMUA_ENABLED)  {

  // Gyroscope A data
  for ( row = 0; row < log_gyrA.count; row++ ) {
    fprintf( datalog.gyrA, "\n %011.6f  %06ld    ", log_gyrA.time[row], log_gyrA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%06d  ",   log_gyrA.raw    [ row*3 +i ] );   fprintf( datalog.gyrA, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%07.4f  ", log_gyrA.scaled [ row*3 +i ] );   fprintf( datalog.gyrA, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%07.4f  ", log_gyrA.filter [ row*3 +i ] );   fprintf( datalog.gyrA, "   " );
  }

  // Accelerometer A data
  for ( row = 0; row < log_accA.count; row++ ) {
    fprintf( datalog.accA, "\n %011.6f  %06ld    ", log_accA.time[row], log_accA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%06d  ",   log_accA.raw    [ row*3 +i ] );   fprintf( datalog.accA, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%07.4f  ", log_accA.scaled [ row*3 +i ] );   fprintf( datalog.accA, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%07.4f  ", log_accA.filter [ row*3 +i ] );   fprintf( datalog.accA, "   " );
  }

  // Magnetometer A data
  for ( row = 0; row < log_magA.count; row++ ) {
    fprintf( datalog.magA, "\n %011.6f  %06ld    ", log_magA.time[row], log_magA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%06d  ",   log_magA.raw    [ row*3 +i ] );   fprintf( datalog.magA, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%07.4f  ", log_magA.scaled [ row*3 +i ] );   fprintf( datalog.magA, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%07.4f  ", log_magA.filter [ row*3 +i ] );   fprintf( datalog.magA, "   " );
  }

  }

  // IMU B datalogs
  if (IMUB_ENABLED)  {

  // Gyroscope B data
  for ( row = 0; row < log_gyrB.count; row++ ) {
    fprintf( datalog.gyrB, "\n %011.6f  %06ld    ", log_gyrB.time[row], log_gyrB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%06d  ",   log_gyrB.raw    [ row*3 +i ] );   fprintf( datalog.gyrB, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%07.4f  ", log_gyrB.scaled [ row*3 +i ] );   fprintf( datalog.gyrB, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%07.4f  ", log_gyrB.filter [ row*3 +i ] );   fprintf( datalog.gyrB, "   " );
  }

  // Accelerometer B data
  for ( row = 0; row < log_accB.count; row++ ) {
    fprintf( datalog.accB, "\n %011.6f  %06ld    ", log_accB.time[row], log_accB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%06d  ",   log_accB.raw    [ row*3 +i ] );   fprintf( datalog.accB, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%07.4f  ", log_accB.scaled [ row*3 +i ] );   fprintf( datalog.accB, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%07.4f  ", log_accB.filter [ row*3 +i ] );   fprintf( datalog.accB, "   " );
  }

  // Magnetometer B data
  for ( row = 0; row < log_magB.count; row++ ) {
    fprintf( datalog.magB, "\n %011.6f  %06ld    ", log_magB.time[row], log_magB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%06d  ",   log_magB.raw    [ row*3 +i ] );   fprintf( datalog.magB, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%07.4f  ", log_magB.scaled [ row*3 +i ] );   fprintf( datalog.magB, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%07.4f  ", log_magB.filter [ row*3 +i ] );   fprintf( datalog.magB, "   " );
  }

  }

  // Attitude/Heading Reference System data
  for ( row = 0; row < log_ahrs.count; row++ ) {
    fprintf( datalog.ahrs, "\n %011.6f  %06ld    ", log_ahrs.time[row], log_ahrs.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.gyr   [ row*3 +i ] );  fprintf( datalog.ahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.acc   [ row*3 +i ] );  fprintf( datalog.ahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.mag   [ row*3 +i ] );  fprintf( datalog.ahrs, "   " );
    for ( i=0; i<4; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.quat  [ row*4 +i ] );  fprintf( datalog.ahrs, "   " );
    for ( i=0; i<4; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.dquat [ row*4 +i ] );  fprintf( datalog.ahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.eul   [ row*3 +i ] );  fprintf( datalog.ahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.deul  [ row*3 +i ] );  fprintf( datalog.ahrs, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.bias  [ row*3 +i ] );  fprintf( datalog.ahrs, "   " );
    fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.fx[ row +i ] );
    fprintf( datalog.ahrs, "%07.4f  ", log_ahrs.fz[ row +i ] );
    fprintf( datalog.ahrs, "   " );
  }

  // GPS data
  for ( row = 0; row < log_gps.count; row++ ) {
    fprintf( datalog.gps, "\n %011.6f  %06ld    ", log_gps.time[row], log_gps.dur[row] );
    fprintf( datalog.gps, "%s ", &log_gps.msg[row*96] );//  fprintf( fgps, "   " );
    fprintf( datalog.gps, "   " );
  }
  /*
  // Controller data
  for ( row = 0; row < log_ctrl.count; row++ ) {
    fprintf( datalog.ctrl, "\n %011.6f  %06ld    ", log_ctrl.time[row], log_ctrl.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.ctrl, "%07.4f  ",  log_ctrl.perr[ row*3 +i ] );   fprintf( datalog.ctrl, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ctrl, "%07.4f  ",  log_ctrl.ierr[ row*3 +i ] );   fprintf( datalog.ctrl, "   " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ctrl, "%07.4f  ",  log_ctrl.derr[ row*3 +i ] );   fprintf( datalog.ctrl, "   " );
    for ( i=0; i<4; i++ )  fprintf( datalog.ctrl, "%07.4f  ",  log_ctrl.cmd [ row*4 +i ] );   fprintf( datalog.ctrl, "   " );
  }
  */

  return;
}


/**
 *  log_free
 *  Frees the allocated memory reserved for the datalogs.
 */
static void log_free ( void )  {

  // Input memory
  free(log_input.time);
  free(log_input.reg);
  free(log_input.pwm);
  free(log_input.norm);

  // Output memory
  free(log_output.time);
  free(log_output.reg);
  free(log_output.pwm);
  free(log_output.norm);

  // Gyroscope A memory
  free(log_gyrA.time);
  free(log_gyrA.dur);
  free(log_gyrA.raw);
  free(log_gyrA.scaled);
  free(log_gyrA.filter);

  // Accelerometer A memory
  free(log_accA.time);
  free(log_accA.dur);
  free(log_accA.raw);
  free(log_accA.scaled);
  free(log_accA.filter);

  // Magnetometer A memory
  free(log_magA.time);
  free(log_magA.dur);
  free(log_magA.raw);
  free(log_magA.scaled);
  free(log_magA.filter);

  // Gyroscope B memory
  free(log_gyrB.time);
  free(log_gyrB.dur);
  free(log_gyrB.raw);
  free(log_gyrB.scaled);
  free(log_gyrB.filter);

  // Accelerometer B memory
  free(log_accB.time);
  free(log_accB.dur);
  free(log_accB.raw);
  free(log_accB.scaled);
  free(log_accB.filter);

  // Magnetometer B memory
  free(log_magB.time);
  free(log_magB.dur);
  free(log_magB.raw);
  free(log_magB.scaled);
  free(log_magB.filter);

  // Attitude/Heading memory
  free(log_ahrs.time);
  free(log_ahrs.dur);
  free(log_ahrs.gyr);
  free(log_ahrs.acc);
  free(log_ahrs.mag);
  free(log_ahrs.quat);
  free(log_ahrs.dquat);
  free(log_ahrs.eul);
  free(log_ahrs.deul);
  free(log_ahrs.bias);
  free(log_ahrs.fx);
  free(log_ahrs.fz);

  // GPS memory
  free(log_gps.time);
  free(log_gps.dur);
  free(log_gps.msg);
  /*
  // Controller memory
  free(log_ctrl.time);
  free(log_ctrl.dur);
  free(log_ctrl.perr);
  free(log_ctrl.ierr);
  free(log_ctrl.derr);
  free(log_ctrl.cmd);
  */

  return;
}


/**
 *  log_close
 *  Closes the datalog file descriptors.
 */
static void log_close ( void )  {

  fclose(datalog.param);
  fclose(datalog.in);
  fclose(datalog.out);

  if (IMUA_ENABLED)  {
    fclose(datalog.gyrA);
    fclose(datalog.accA);
    fclose(datalog.magA);
  }

  if (IMUB_ENABLED)  {
    fclose(datalog.gyrB);
    fclose(datalog.accB);
    fclose(datalog.magB);
  }

  fclose(datalog.ahrs);
  fclose(datalog.gps);
  //fclose(datalog.ctrl);

  return;
}


/**
 *  log_record
 *  Records the data to the log file.
 */
void log_record ( enum log_index index )  {

  // Local variables
  ushort i;
  ulong  row;
  float  timestamp;

  // Jump to appropriate log 
  switch(index) {


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record system input/output data
  case LOG_IO :

    timestamp = (float) ( tmr_io.start_sec + ( tmr_io.start_usec / 1000000.0f ) ) - datalog.offset;

    // Input data
    pthread_mutex_lock(&mutex_input);
    if ( log_input.count < log_input.limit ) {
      row = log_input.count;
      log_input.time[row] = timestamp;
      for ( i=0; i<10; i++ )  log_input.norm [ row*10 +i ] = input.norm[i];
      log_input.count++;
    }
    pthread_mutex_unlock(&mutex_input);

    // Output data
    pthread_mutex_lock(&mutex_output);
    if ( log_output.count < log_output.limit ) {
      row = log_output.count;
      log_output.time[row] = timestamp;
      for ( i=0; i<10; i++ )  log_output.norm [ row*10 +i ] = output.norm[i];
      log_output.count++;
    }
    pthread_mutex_unlock(&mutex_output);

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record IMUA data
  case LOG_IMUA :

    if (IMUB_ENABLED)  {  timestamp = (float) ( tmr_imu.start_sec  + ( tmr_imu.start_usec  / 1000000.0f ) ) - datalog.offset;  }
    else               {  timestamp = (float) ( tmr_imuA.start_sec + ( tmr_imuA.start_usec / 1000000.0f ) ) - datalog.offset;  }

    // Gyroscope A data
    pthread_mutex_lock(&mutex_gyrA);
    if ( log_gyrA.count < log_gyrA.limit ) {
      row = log_gyrA.count;
      log_gyrA.time[row] = timestamp;
      if (IMUB_ENABLED)  log_gyrA.dur[row]  = tmr_imu.dur;
      else               log_gyrA.dur[row]  = tmr_imuA.dur;
      for ( i=0; i<3; i++ )  log_gyrA.raw    [ row*3 +i ] = gyrA.raw[i];
      for ( i=0; i<3; i++ )  log_gyrA.scaled [ row*3 +i ] = gyrA.scaled[i];
      for ( i=0; i<3; i++ )  log_gyrA.filter [ row*3 +i ] = gyrA.filter[i];
      log_gyrA.count++;
    }
    pthread_mutex_unlock(&mutex_gyrA);

    // Accelerometer A data
    pthread_mutex_lock(&mutex_accA);
    if ( log_accA.count < log_accA.limit ) {
      row = log_accA.count;
      log_accA.time[row] = timestamp;
      if (IMUB_ENABLED)  log_accA.dur[row]  = tmr_imu.dur;
      else               log_accA.dur[row]  = tmr_imuA.dur;
      for ( i=0; i<3; i++ )  log_accA.raw    [ row*3 +i ] = accA.raw[i];
      for ( i=0; i<3; i++ )  log_accA.scaled [ row*3 +i ] = accA.scaled[i];
      for ( i=0; i<3; i++ )  log_accA.filter [ row*3 +i ] = accA.filter[i];
      log_accA.count++;
    }
    pthread_mutex_unlock(&mutex_accA);

    // Magnetometer A data
    pthread_mutex_lock(&mutex_magA);
    if( imuA.getmag && ( log_magA.count < log_magA.limit) ) {
      row = log_magA.count;
      log_magA.time[row] = timestamp;
      if (IMUB_ENABLED)  log_magA.dur[row]  = tmr_imu.dur;
      else               log_magA.dur[row]  = tmr_imuA.dur;
      for ( i=0; i<3; i++ )  log_magA.raw    [ row*3 +i ] = magA.raw[i];
      for ( i=0; i<3; i++ )  log_magA.scaled [ row*3 +i ] = magA.scaled[i];
      for ( i=0; i<3; i++ )  log_magA.filter [ row*3 +i ] = magA.filter[i];
      log_magA.count++;
    }
    pthread_mutex_unlock(&mutex_magA);

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record IMUB data
  case LOG_IMUB :

    if (IMUA_ENABLED)  {  timestamp = (float) ( tmr_imu.start_sec  + ( tmr_imu.start_usec  / 1000000.0f ) ) - datalog.offset;  }
    else               {  timestamp = (float) ( tmr_imuB.start_sec + ( tmr_imuB.start_usec / 1000000.0f ) ) - datalog.offset;  }

    // Gyroscope B data
    pthread_mutex_lock(&mutex_gyrB);
    if ( log_gyrB.count < log_gyrB.limit ) {
      row = log_gyrB.count;
      log_gyrB.time[row] = timestamp;
      if (IMUA_ENABLED)  log_gyrB.dur[row]  = tmr_imu.dur;
      else               log_gyrB.dur[row]  = tmr_imuB.dur;
      for ( i=0; i<3; i++ )  log_gyrB.raw    [ row*3 +i ] = gyrB.raw[i];
      for ( i=0; i<3; i++ )  log_gyrB.scaled [ row*3 +i ] = gyrB.scaled[i];
      for ( i=0; i<3; i++ )  log_gyrB.filter [ row*3 +i ] = gyrB.filter[i];
      log_gyrB.count++;
    }
    pthread_mutex_unlock(&mutex_gyrB);

    // Accelerometer B data
    pthread_mutex_lock(&mutex_accB);
    if ( log_accB.count < log_accB.limit ) {
      row = log_accB.count;
      log_accB.time[row] = timestamp;
      if (IMUA_ENABLED)  log_accB.dur[row]  = tmr_imu.dur;
      else               log_accB.dur[row]  = tmr_imuB.dur;
      for ( i=0; i<3; i++ )  log_accB.raw    [ row*3 +i ] = accB.raw[i];
      for ( i=0; i<3; i++ )  log_accB.scaled [ row*3 +i ] = accB.scaled[i];
      for ( i=0; i<3; i++ )  log_accB.filter [ row*3 +i ] = accB.filter[i];
      log_accB.count++;
    }
    pthread_mutex_unlock(&mutex_accB);

    // Magnetometer B data
    pthread_mutex_lock(&mutex_magB);
    if( imuB.getmag && ( log_magB.count < log_magB.limit) ) {
      row = log_magB.count;
      log_magB.time[row] = timestamp;
      if (IMUA_ENABLED)  log_magB.dur[row]  = tmr_imu.dur;
      else               log_magB.dur[row]  = tmr_imuB.dur;
      for ( i=0; i<3; i++ )  log_magB.raw    [ row*3 +i ] = magB.raw[i];
      for ( i=0; i<3; i++ )  log_magB.scaled [ row*3 +i ] = magB.scaled[i];
      for ( i=0; i<3; i++ )  log_magB.filter [ row*3 +i ] = magB.filter[i];
      log_magB.count++;
    }
    pthread_mutex_unlock(&mutex_magB);

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record AHRS data
  case LOG_AHRS :

    timestamp = (float) ( tmr_ahrs.start_sec + ( tmr_ahrs.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ahrs.count < log_ahrs.limit ) {
      row = log_ahrs.count;
      log_ahrs.time[row] = timestamp;
      log_ahrs.dur[row]  = tmr_ahrs.dur;

      pthread_mutex_lock(&mutex_ahrs);
      for ( i=0; i<3; i++ )  log_ahrs.gyr [ row*3 +i ] = ahrs.gyr [i];
      for ( i=0; i<3; i++ )  log_ahrs.acc [ row*3 +i ] = ahrs.acc [i];
      for ( i=0; i<3; i++ )  log_ahrs.mag [ row*3 +i ] = ahrs.mag [i];
      pthread_mutex_unlock(&mutex_ahrs);

      pthread_mutex_lock(&mutex_quat);
      for ( i=0; i<4; i++ )  log_ahrs.quat  [ row*4 +i ] = ahrs.quat  [i];
      for ( i=0; i<4; i++ )  log_ahrs.dquat [ row*4 +i ] = ahrs.dquat [i];
      pthread_mutex_unlock(&mutex_quat);

      pthread_mutex_lock(&mutex_eul);
      for ( i=0; i<3; i++ )  log_ahrs.eul   [ row*3 +i ] = ahrs.eul   [i];
      for ( i=0; i<3; i++ )  log_ahrs.deul  [ row*3 +i ] = ahrs.deul  [i];
      pthread_mutex_unlock(&mutex_eul);

      pthread_mutex_lock(&mutex_ahrs);
      for ( i=0; i<3; i++ )  log_ahrs.bias  [ row*3 +i ] = ahrs.bias  [i];
                             log_ahrs.fx    [ row   +i ] = ahrs.fx;
                             log_ahrs.fz    [ row   +i ] = ahrs.fz;
      pthread_mutex_unlock(&mutex_ahrs);

      log_ahrs.count++;
    }

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record GPS data
  case LOG_GPS :

    timestamp = (float) ( tmr_gps.start_sec + ( tmr_gps.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_gps.count < log_gps.limit ) {
      row = log_gps.count;
      log_gps.time[row] = timestamp;
      log_gps.dur[row]  = tmr_gps.dur;

      pthread_mutex_lock(&mutex_gps);
      sprintf( &log_gps.msg[row*96], gps.msg );
      //printf("\nGPS.MSG: %s \n", &log_gps.msg[row*96] );
      pthread_mutex_unlock(&mutex_gps);

      log_gps.count++;
    }

    return;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Record CTRL data
    /*
  case LOG_CTRL :

    timestamp = (float) ( tmr_ctrl.start_sec + ( tmr_ctrl.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ctrl.count < log_ctrl.limit ) {
      row = log_ctrl.count;
      log_ctrl.time[row] = timestamp;
      log_ctrl.dur[row]  = tmr_ctrl.dur;
      for ( i=0; i<3; i++ )  log_ctrl.perr[ row*3 +i ] = ctrl.perr[i];
      for ( i=0; i<3; i++ )  log_ctrl.ierr[ row*3 +i ] = ctrl.ierr[i];
      for ( i=0; i<3; i++ )  log_ctrl.derr[ row*3 +i ] = ctrl.derr[i];
      for ( i=0; i<4; i++ )  log_ctrl.cmd [ row*4 +i ] = ctrl.cmd[i];
      log_ctrl.count++;
    }

    return;
    */

  default :
    return;
  
  }
}



