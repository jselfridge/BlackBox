

#include "log.h"
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include "ahrs.h"
//#include "ekf.h"
#include "gcs.h"
//#include "gps.h"
#include "imu.h"
#include "io.h"
#include "led.h"
#include "stab.h"
#include "sys.h"
#include "timer.h"


static void  log_save  ( void );
static void  log_close ( void );


/**
 *  log_init
 *  Runs on start up to initalize the datalog attributes.
 */
void log_init ( void )  {
  if(DEBUG)  printf("Initializing data logging \n");

  // Set boolean values... move to "log.c"
  if(DEBUG)  printf("  Set boolean conditions \n");
  datalog.enabled  = false;
  datalog.setup    = false;
  datalog.saving   = false;

  // Assign datalog limits
  log_param.limit   =  LOG_MAX_PARAM;  
  log_input.limit   =  LOG_MAX_DUR * HZ_IO;
  log_output.limit  =  LOG_MAX_DUR * HZ_IO;
  log_gyrA.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_accA.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_magA.limit    =  LOG_MAX_DUR * HZ_IMU_SLOW;
  log_compA.limit   =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_ahrsA.limit   =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_gyrB.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_accB.limit    =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_magB.limit    =  LOG_MAX_DUR * HZ_IMU_SLOW;
  log_compB.limit   =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_ahrsB.limit   =  LOG_MAX_DUR * HZ_IMU_FAST;
  log_stab.limit    =  LOG_MAX_DUR * HZ_STAB;
  //log_ekf.limit     =  LOG_MAX_DUR * HZ_EKF;
  //log_gps.limit     =  LOG_MAX_DUR * HZ_GPS;

  // Parameter value setup
  log_param.time    =  malloc( sizeof(float)  * log_param.limit               );
  log_param.values  =  malloc( sizeof(float)  * log_param.limit * param_count );

  // Input signal setup
  log_input.time    =  malloc( sizeof(float)  * log_input.limit         );
  log_input.data    =  malloc( sizeof(float)  * log_input.limit * IN_CH );

  // Output signal setup
  log_output.time   =  malloc( sizeof(float)  * log_output.limit          );
  log_output.data   =  malloc( sizeof(float)  * log_output.limit * OUT_CH );

  // IMU A setup
  if (IMUA_ENABLED) {

  // Gyroscope A setup
  log_gyrA.time     =  malloc( sizeof(float)  * log_gyrA.limit     );
  log_gyrA.dur      =  malloc( sizeof(ulong)  * log_gyrA.limit     );
  log_gyrA.raw      =  malloc( sizeof(short)  * log_gyrA.limit * 3 );
  log_gyrA.scaled   =  malloc( sizeof(float)  * log_gyrA.limit * 3 );
  log_gyrA.filter   =  malloc( sizeof(float)  * log_gyrA.limit * 3 );

  // Accelerometer A setup
  log_accA.time     =  malloc( sizeof(float)  * log_accA.limit     );
  log_accA.dur      =  malloc( sizeof(ulong)  * log_accA.limit     );
  log_accA.raw      =  malloc( sizeof(short)  * log_accA.limit * 3 );
  log_accA.scaled   =  malloc( sizeof(float)  * log_accA.limit * 3 );
  log_accA.filter   =  malloc( sizeof(float)  * log_accA.limit * 3 );

  // Magnetometer A setup
  log_magA.time     =  malloc( sizeof(float)  * log_magA.limit     );
  log_magA.dur      =  malloc( sizeof(ulong)  * log_magA.limit     );
  log_magA.raw      =  malloc( sizeof(short)  * log_magA.limit * 3 );
  log_magA.scaled   =  malloc( sizeof(float)  * log_magA.limit * 3 );
  log_magA.filter   =  malloc( sizeof(float)  * log_magA.limit * 3 );

  // Complimentary filter A setup
  log_compA.time    =  malloc( sizeof(float)  * log_compA.limit );
  log_compA.dur     =  malloc( sizeof(ulong)  * log_compA.limit );
  log_compA.roll    =  malloc( sizeof(float)  * log_compA.limit );
  log_compA.pitch   =  malloc( sizeof(float)  * log_compA.limit );

  // Attitude and Heading Reference System A setup
  log_ahrsA.time    =  malloc( sizeof(float)  * log_ahrsA.limit     );
  log_ahrsA.dur     =  malloc( sizeof(ulong)  * log_ahrsA.limit     );
  log_ahrsA.quat    =  malloc( sizeof(float)  * log_ahrsA.limit * 4 );
  log_ahrsA.dquat   =  malloc( sizeof(float)  * log_ahrsA.limit * 4 );
  log_ahrsA.eul     =  malloc( sizeof(float)  * log_ahrsA.limit * 3 );
  log_ahrsA.deul    =  malloc( sizeof(float)  * log_ahrsA.limit * 3 );
  log_ahrsA.bias    =  malloc( sizeof(float)  * log_ahrsA.limit * 3 );
  log_ahrsA.fx      =  malloc( sizeof(float)  * log_ahrsA.limit     );
  log_ahrsA.fz      =  malloc( sizeof(float)  * log_ahrsA.limit     );

  }

  // IMU B setup
  if (IMUB_ENABLED) {

  // Gyroscope B setup
  log_gyrB.time     =  malloc( sizeof(float)  * log_gyrB.limit     );
  log_gyrB.dur      =  malloc( sizeof(ulong)  * log_gyrB.limit     );
  log_gyrB.raw      =  malloc( sizeof(short)  * log_gyrB.limit * 3 );
  log_gyrB.scaled   =  malloc( sizeof(float)  * log_gyrB.limit * 3 );
  log_gyrB.filter   =  malloc( sizeof(float)  * log_gyrB.limit * 3 );

  // Accelerometer B setup
  log_accB.time     =  malloc( sizeof(float)  * log_accB.limit     );
  log_accB.dur      =  malloc( sizeof(ulong)  * log_accB.limit     );
  log_accB.raw      =  malloc( sizeof(short)  * log_accB.limit * 3 );
  log_accB.scaled   =  malloc( sizeof(float)  * log_accB.limit * 3 );
  log_accB.filter   =  malloc( sizeof(float)  * log_accB.limit * 3 );

  // Magnetometer B setup
  log_magB.time     =  malloc( sizeof(float)  * log_magB.limit     );
  log_magB.dur      =  malloc( sizeof(ulong)  * log_magB.limit     );
  log_magB.raw      =  malloc( sizeof(short)  * log_magB.limit * 3 );
  log_magB.scaled   =  malloc( sizeof(float)  * log_magB.limit * 3 );
  log_magB.filter   =  malloc( sizeof(float)  * log_magB.limit * 3 );

  // Complimentary filter B setup
  log_compB.time    =  malloc( sizeof(float)  * log_compB.limit );
  log_compB.dur     =  malloc( sizeof(ulong)  * log_compB.limit );
  log_compB.roll    =  malloc( sizeof(float)  * log_compB.limit );
  log_compB.pitch   =  malloc( sizeof(float)  * log_compB.limit );

  // Attitude and Heading Reference System B setup
  log_ahrsB.time    =  malloc( sizeof(float)  * log_ahrsB.limit     );
  log_ahrsB.dur     =  malloc( sizeof(ulong)  * log_ahrsB.limit     );
  log_ahrsB.quat    =  malloc( sizeof(float)  * log_ahrsB.limit * 4 );
  log_ahrsB.dquat   =  malloc( sizeof(float)  * log_ahrsB.limit * 4 );
  log_ahrsB.eul     =  malloc( sizeof(float)  * log_ahrsB.limit * 3 );
  log_ahrsB.deul    =  malloc( sizeof(float)  * log_ahrsB.limit * 3 );
  log_ahrsB.bias    =  malloc( sizeof(float)  * log_ahrsB.limit * 3 );
  log_ahrsB.fx      =  malloc( sizeof(float)  * log_ahrsB.limit     );
  log_ahrsB.fz      =  malloc( sizeof(float)  * log_ahrsB.limit     );

  }

  // Stabilization setup
  log_stab.time     =  malloc( sizeof(float)  * log_stab.limit     );
  log_stab.dur      =  malloc( sizeof(ulong)  * log_stab.limit     );
  log_stab.att      =  malloc( sizeof(float)  * log_stab.limit * 3 );
  log_stab.ang      =  malloc( sizeof(float)  * log_stab.limit * 3 );
  log_stab.cmd      =  malloc( sizeof(float)  * log_stab.limit * 4 );

  // PID stabilization
  log_pidX.perr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidX.ierr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidX.derr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidY.perr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidY.ierr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidY.derr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidZ.perr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidZ.ierr     =  malloc( sizeof(float)  * log_stab.limit );
  log_pidZ.derr     =  malloc( sizeof(float)  * log_stab.limit );

  // Extended Kalman Filter setup
  //uint n = EKF_N;
  //uint m = EKF_M;
  //log_ekf.time      =  malloc( sizeof(float)  * log_ekf.limit         );
  //log_ekf.dur       =  malloc( sizeof(ulong)  * log_ekf.limit         );
  //log_ekf.x         =  malloc( sizeof(float)  * log_ekf.limit * n     );
  //log_ekf.z         =  malloc( sizeof(float)  * log_ekf.limit * m     );
  //log_ekf.f         =  malloc( sizeof(float)  * log_ekf.limit * n     );
  //log_ekf.h         =  malloc( sizeof(float)  * log_ekf.limit * m     );
  //log_ekf.P         =  malloc( sizeof(float)  * log_ekf.limit * n * n );
  //log_ekf.S         =  malloc( sizeof(float)  * log_ekf.limit * m * m );
  //log_ekf.K         =  malloc( sizeof(float)  * log_ekf.limit * n * m );

  /*
  // Global Positioning System setup
  log_gps.time      =  malloc( sizeof(float)  * log_gps.limit      );
  log_gps.dur       =  malloc( sizeof(ulong)  * log_gps.limit      );
  log_gps.msg       =  malloc( sizeof(char)   * log_gps.limit * 96 );
  */

  return;
}


/**
 *  log_exit
 *  Closes the data log files.
 */
void log_exit ( void )  {
  if(DEBUG)  printf("Close logs \n");

  // Parameter memory
  free(log_param.time);
  free(log_param.values);

  // Input memory
  free(log_input.time);
  free(log_input.data);

  // Output memory
  free(log_output.time);
  free(log_output.data);

  // IMU A memory
  if (IMUA_ENABLED) {

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

  // Comp filter A memory
  free(log_compA.time);
  free(log_compA.dur);
  free(log_compA.roll);
  free(log_compA.pitch);

  // Attitude/Heading A memory
  free(log_ahrsA.time);
  free(log_ahrsA.dur);
  free(log_ahrsA.quat);
  free(log_ahrsA.dquat);
  free(log_ahrsA.eul);
  free(log_ahrsA.deul);
  free(log_ahrsA.bias);
  free(log_ahrsA.fx);
  free(log_ahrsA.fz);

  }

  // IMU B memory
  if (IMUB_ENABLED) {

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

  // Comp filter B memory
  free(log_compB.time);
  free(log_compB.dur);
  free(log_compB.roll);
  free(log_compB.pitch);

  // Attitude/Heading B memory
  free(log_ahrsB.time);
  free(log_ahrsB.dur);
  free(log_ahrsB.quat);
  free(log_ahrsB.dquat);
  free(log_ahrsB.eul);
  free(log_ahrsB.deul);
  free(log_ahrsB.bias);
  free(log_ahrsB.fx);
  free(log_ahrsB.fz);

  }

  // Stabilization memory
  free(log_stab.time);
  free(log_stab.dur);
  free(log_stab.att);
  free(log_stab.ang);
  free(log_stab.cmd);

  // PID stabilization memory
  free(log_pidX.perr);
  free(log_pidX.ierr);
  free(log_pidX.derr);
  free(log_pidY.perr);
  free(log_pidY.ierr);
  free(log_pidY.derr);
  free(log_pidZ.perr);
  free(log_pidZ.ierr);
  free(log_pidZ.derr);

  // EKF memory
  //free(log_ekf.time);
  //free(log_ekf.dur);
  //free(log_ekf.x);
  //free(log_ekf.z);
  //free(log_ekf.f);
  //free(log_ekf.h);
  //free(log_ekf.P);
  //free(log_ekf.S);
  //free(log_ekf.K);

  /*
  // GPS memory
  free(log_gps.time);
  free(log_gps.dur);
  free(log_gps.msg);
  */

  return;
}


/**
 *  log_start
 *  Starts the next datalog sequence.
 */
void log_start ( void )  {

  // Reset counters
  log_param.count  = 0;
  log_input.count  = 0;
  log_output.count = 0;
  log_gyrA.count   = 0;
  log_accA.count   = 0;
  log_magA.count   = 0;
  log_compA.count  = 0;
  log_ahrsA.count  = 0;
  log_gyrB.count   = 0;
  log_accB.count   = 0;
  log_magB.count   = 0;
  log_compB.count  = 0;
  log_ahrsB.count  = 0;
  log_stab.count   = 0;
  //log_ekf.count    = 0;
  //log_gps.count    = 0;

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
    else    printf( "Error (log_init): Exceeded maximum number of log directories. \n" );
    sprintf( file, "../Log/%s/param.txt", datalog.dir );
    if ( access( file , F_OK ) == -1 )  break;
  }

  // Create new directory
  sprintf( datalog.path, "../Log/%s/", datalog.dir );
  mkdir( datalog.path, 222 );

  // Parameter datalog file
  sprintf( file, "%sparam.txt", datalog.path );
  datalog.param = fopen( file, "w" );
  if( datalog.param == NULL )  printf( "Error (log_init): Cannot generate 'param' file. \n" );
  fprintf( datalog.param, "   ParamTime   " );
  for ( i=1; i <= param_count; i++ )  fprintf( datalog.param, "   Param_%02d", i );

  // Input datalog file
  sprintf( file, "%sinput.txt", datalog.path );
  datalog.in = fopen( file, "w" );
  if( datalog.in == NULL )  printf( "Error (log_init): Cannot generate 'input' file. \n" );
  fprintf( datalog.in,
    "      InTime   \
    In01     In02     In03     In04     In05 \
    In06     In07     In08     In09     In10" );

  // Output datalog file
  sprintf( file, "%soutput.txt", datalog.path );
  datalog.out = fopen( file, "w" );
  if( datalog.out == NULL )  printf( "Error (log_init): Cannot generate 'output' file. \n" );
  fprintf( datalog.out, 
    "     OutTime  \
    Out01    Out02    Out03    Out04    Out05\
    Out06    Out07    Out08    Out09    Out10" );

  // IMUA datalogs
  if (IMUA_ENABLED)  {

  // Gyroscope A datalog file
  sprintf( file, "%sgyrA.txt", datalog.path );
  datalog.gyrA = fopen( file, "w" );
  if( datalog.gyrA == NULL )  printf( "Error (log_init): Cannot generate 'gyrA' file. \n" );
  fprintf( datalog.gyrA,
    "    GyrAtime  GyrAdur  \
    GyrArx  GyrAry  GyrArz   \
    GyrAsx   GyrAsy   GyrAsz   \
    GyrAfx   GyrAfy   GyrAfz");

  // Accelerometer A datalog file
  sprintf( file, "%saccA.txt", datalog.path );
  datalog.accA = fopen( file, "w" );
  if( datalog.accA == NULL )  printf( "Error (log_init): Cannot generate 'accA' file. \n" );
  fprintf( datalog.accA, 
    "    AccAtime  AccAdur  \
    AccArx  AccAry  AccArz   \
    AccAsx   AccAsy   AccAsz   \
    AccAfx   AccAfy   AccAfz");

  // Magnetometer A datalog file
  sprintf( file, "%smagA.txt", datalog.path );
  datalog.magA = fopen( file, "w" );
  if( datalog.magA == NULL )  printf( "Error (log_init): Cannot generate 'magA' file. \n" );
  fprintf( datalog.magA,
    "    MagAtime  MagAdur  \
    MagArx  MagAry  MagArz   \
    MagAsx   MagAsy   MagAsz   \
    MagAfx   MagAfy   MagAfz");

  // Comp filter A datalog file
  sprintf( file, "%scompA.txt", datalog.path );
  datalog.compA = fopen( file, "w" );
  if( datalog.compA == NULL )  printf( "Error (log_init): Cannot generate 'compA' file. \n" );
  fprintf( datalog.compA, "   CompAtime CompAdur    CompAroll CompApitch ");  

  // Attitude and heading reference system A datalog file
  sprintf( file, "%sahrsA.txt", datalog.path );
  datalog.ahrsA = fopen( file, "w" );
  if( datalog.ahrsA == NULL )  printf( "Error (log_init): Cannot generate 'ahrsA' file. \n" );
  fprintf( datalog.ahrsA,
    "   ahrsAtime ahrsAdur   \
    QuatAw   QuatAx   QuatAy   QuatAz  \
    dQuatAw  dQuatAx  dQuatAy  dQuatAz    \
    EulAx    EulAy    EulAz   \
    dEulAx   dEulAy   dEulAz   \
    biasAx   biasAy   biasAz   \
    fluxAx   fluxAz");

  }

  // IMUB datalogs
  if (IMUB_ENABLED )  {

  // Gyroscope B datalog file
  sprintf( file, "%sgyrB.txt", datalog.path );
  datalog.gyrB = fopen( file, "w" );
  if( datalog.gyrB == NULL )  printf( "Error (log_init): Cannot generate 'gyrB' file. \n" );
  fprintf( datalog.gyrB,
    "    GyrBtime  GyrBdur  \
    GyrBrx  GyrBry  GyrBrz   \
    GyrBsx   GyrBsy   GyrBsz   \
    GyrBfx   GyrBfy   GyrBfz");

  // Accelerometer B datalog file
  sprintf( file, "%saccB.txt", datalog.path );
  datalog.accB = fopen( file, "w" );
  if( datalog.accB == NULL )  printf( "Error (log_init): Cannot generate 'accB' file. \n" );
  fprintf( datalog.accB, 
    "    AccBtime  AccBdur  \
    AccBrx  AccBry  AccBrz   \
    AccBsx   AccBsy   AccBsz   \
    AccBfx   AccBfy   AccBfz");

  // Magnetometer B datalog file
  sprintf( file, "%smagB.txt", datalog.path );
  datalog.magB = fopen( file, "w" );
  if( datalog.magB == NULL )  printf( "Error (log_init): Cannot generate 'magB' file. \n" );
  fprintf( datalog.magB,
    "    MagBtime  MagBdur  \
    MagBrx  MagBry  MagBrz   \
    MagBsx   MagBsy   MagBsz   \
    MagBfx   MagBfy   MagBfz");

  // Comp filter B datalog file
  sprintf( file, "%scompB.txt", datalog.path );
  datalog.compB = fopen( file, "w" );
  if( datalog.compB == NULL )  printf( "Error (log_init): Cannot generate 'compB' file. \n" );
  fprintf( datalog.compB, "   CompBtime CompBdur    CompBroll CompBpitch ");  

  // Attitude and heading reference system B datalog file
  sprintf( file, "%sahrsB.txt", datalog.path );
  datalog.ahrsB = fopen( file, "w" );
  if( datalog.ahrsB == NULL )  printf( "Error (log_init): Cannot generate 'ahrsB' file. \n" );
  fprintf( datalog.ahrsB,
    "   ahrsBtime ahrsBdur   \
    QuatBw   QuatBx   QuatBy   QuatBz  \
    dQuatBw  dQuatBx  dQuatBy  dQuatBz    \
    EulBx    EulBy    EulBz   \
    dEulBx   dEulBy   dEulBz   \
    biasBx   biasBy   biasBz   \
    fluxBx   fluxBz");

  }

  // Stabilization datalog file
  sprintf( file, "%sstab.txt", datalog.path );
  datalog.stab = fopen( file, "w" );
  if( datalog.stab == NULL )  printf( "Error (log_init): Cannot generate 'stab' file. \n" );
  fprintf( datalog.stab, 
    "    stabtime  stabdur     \
    attX     attY     attZ         angX     angY     angZ     \
    cmdX     cmdY     cmdZ     cmdT        " );
  fprintf( datalog.stab, "Xperr    Xierr    Xzerr        Yperr    Yierr    Yderr        Zperr    Zierr    Zderr " );  

  // Extended Kalman Filter datalog file
  //uint n = EKF_N;
  //uint m = EKF_M;
  //sprintf( file, "%sekf.txt", datalog.path );
  //datalog.ekf = fopen( file, "w" );
  //if( datalog.ekf == NULL )  printf( "Error (log_init): Cannot generate 'ekf' file. \n" );
  //fprintf( datalog.ekf, "     ekftime   ekfdur          " );
  //for ( i=1; i<=n;  i++ )  fprintf( datalog.ekf, "x%02d      ", i );  fprintf( datalog.ekf, "    " );
  //for ( i=1; i<=m;  i++ )  fprintf( datalog.ekf, "z%02d      ", i );  fprintf( datalog.ekf, "    " );
  //for ( i=1; i<=n;  i++ )  fprintf( datalog.ekf, "f%02d      ", i );  fprintf( datalog.ekf, "    " );
  //for ( i=1; i<=m;  i++ )  fprintf( datalog.ekf, "h%02d      ", i );  fprintf( datalog.ekf, " " );
  //for ( i=0; i<n*n; i++ )  fprintf( datalog.ekf, "P_%02d%02d    ", (i/n)+1, (i%n)+1 );  fprintf( datalog.ekf, "   " );
  //for ( i=0; i<m*m; i++ )  fprintf( datalog.ekf, "S_%02d%02d    ", (i/m)+1, (i%m)+1 );  fprintf( datalog.ekf, "   " );
  //for ( i=0; i<n*m; i++ )  fprintf( datalog.ekf, "K_%02d%02d    ", (i/n)+1, (i%n)+1 );  fprintf( datalog.ekf, "   " );

  /*
  // GPS datalog file
  sprintf( file, "%sgps.txt", datalog.path );
  datalog.gps = fopen( file, "w" );
  if( datalog.gps == NULL )  printf( "Error (log_init): Cannot generate 'gps' file. \n" );
  fprintf( datalog.gps, "       Gtime    Gdur    Data    ");
  */

  // Determine start second
  struct timespec timeval;
  clock_gettime( CLOCK_MONOTONIC, &timeval );
  datalog.offset = timeval.tv_sec;

  // Record initial parameter values
  log_record(LOG_PARAM);

  // Update status flag
  datalog.setup = true;

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
  struct timespec t;

  // Jump to appropriate log 
  switch(index) {


  // Record parameter values
  case LOG_PARAM :

    // Determine timestamp
    clock_gettime( CLOCK_MONOTONIC, &t );
    timestamp = (float) ( t.tv_sec + ( t.tv_nsec / 1000000000.0f ) ) - datalog.offset;

    pthread_mutex_lock(&gcs.mutex);
    if ( log_param.count < log_param.limit )  {
      row = log_param.count;
      log_param.time[row] = timestamp;
      for ( i=0; i < param_count; i++ )  log_param.values [ row * param_count + i ] = param.val[i];
      log_param.count++;
    }
    pthread_mutex_unlock(&gcs.mutex);

    return;


  // Record system input/output data
  case LOG_IO :

    timestamp = (float) ( tmr_io.start_sec + ( tmr_io.start_usec / 1000000.0f ) ) - datalog.offset;

    // Input data
    pthread_mutex_lock(&input.mutex);
    if ( log_input.count < log_input.limit ) {
      row = log_input.count;
      log_input.time[row] = timestamp;
      for ( i=0; i<10; i++ )  log_input.data [ row*10 +i ] = input.norm[i];
      log_input.count++;
    }
    pthread_mutex_unlock(&input.mutex);

    // Output data
    pthread_mutex_lock(&output.mutex);
    if ( log_output.count < log_output.limit ) {
      row = log_output.count;
      log_output.time[row] = timestamp;
      for ( i=0; i<10; i++ )  log_output.data [ row*10 +i ] = output.norm[i];
      log_output.count++;
    }
    pthread_mutex_unlock(&output.mutex);

    return;


  // Record IMUA data
  case LOG_IMUA :

    timestamp = (float) ( tmr_imu.start_sec  + ( tmr_imu.start_usec  / 1000000.0f ) ) - datalog.offset;

    // Gyroscope A data
    pthread_mutex_lock(&gyrA.mutex);
    if ( log_gyrA.count < log_gyrA.limit ) {
      row = log_gyrA.count;
      log_gyrA.time[row] = timestamp;
      log_gyrA.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_gyrA.raw    [ row*3 +i ] = gyrA.raw[i];
      for ( i=0; i<3; i++ )  log_gyrA.scaled [ row*3 +i ] = gyrA.scaled[i];
      for ( i=0; i<3; i++ )  log_gyrA.filter [ row*3 +i ] = gyrA.filter[i];
      log_gyrA.count++;
    }
    pthread_mutex_unlock(&gyrA.mutex);

    // Accelerometer A data
    pthread_mutex_lock(&accA.mutex);
    if ( log_accA.count < log_accA.limit ) {
      row = log_accA.count;
      log_accA.time[row] = timestamp;
      log_accA.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_accA.raw    [ row*3 +i ] = accA.raw[i];
      for ( i=0; i<3; i++ )  log_accA.scaled [ row*3 +i ] = accA.scaled[i];
      for ( i=0; i<3; i++ )  log_accA.filter [ row*3 +i ] = accA.filter[i];
      log_accA.count++;
    }
    pthread_mutex_unlock(&accA.mutex);

    // Magnetometer A data
    pthread_mutex_lock(&magA.mutex);
    if ( imuA.getmag && ( log_magA.count < log_magA.limit ) ) {
      row = log_magA.count;
      log_magA.time[row] = timestamp;
      log_magA.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_magA.raw    [ row*3 +i ] = magA.raw[i];
      for ( i=0; i<3; i++ )  log_magA.scaled [ row*3 +i ] = magA.scaled[i];
      for ( i=0; i<3; i++ )  log_magA.filter [ row*3 +i ] = magA.filter[i];
      log_magA.count++;
    }
    pthread_mutex_unlock(&magA.mutex);

    // Comp filter A data
    pthread_mutex_lock(&imuA.mutex);
    if ( log_compA.count < log_compA.limit ) {
      row = log_compA.count;
      log_compA.time[row]  = timestamp;
      log_compA.dur[row]   = tmr_imu.dur;
      log_compA.roll[row]  = imuA.roll;
      log_compA.pitch[row] = imuA.pitch;
      log_compA.count++;
    }
    pthread_mutex_unlock(&imuA.mutex);

    return;


  // Record IMUB data
  case LOG_IMUB :

    timestamp = (float) ( tmr_imu.start_sec  + ( tmr_imu.start_usec  / 1000000.0f ) ) - datalog.offset;

    // Gyroscope B data
    pthread_mutex_lock(&gyrB.mutex);
    if ( log_gyrB.count < log_gyrB.limit ) {
      row = log_gyrB.count;
      log_gyrB.time[row] = timestamp;
      log_gyrB.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_gyrB.raw    [ row*3 +i ] = gyrB.raw[i];
      for ( i=0; i<3; i++ )  log_gyrB.scaled [ row*3 +i ] = gyrB.scaled[i];
      for ( i=0; i<3; i++ )  log_gyrB.filter [ row*3 +i ] = gyrB.filter[i];
      log_gyrB.count++;
    }
    pthread_mutex_unlock(&gyrB.mutex);

    // Accelerometer B data
    pthread_mutex_lock(&accB.mutex);
    if ( log_accB.count < log_accB.limit ) {
      row = log_accB.count;
      log_accB.time[row] = timestamp;
      log_accB.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_accB.raw    [ row*3 +i ] = accB.raw[i];
      for ( i=0; i<3; i++ )  log_accB.scaled [ row*3 +i ] = accB.scaled[i];
      for ( i=0; i<3; i++ )  log_accB.filter [ row*3 +i ] = accB.filter[i];
      log_accB.count++;
    }
    pthread_mutex_unlock(&accB.mutex);

    // Magnetometer B data
    pthread_mutex_lock(&magB.mutex);
    if( imuB.getmag && ( log_magB.count < log_magB.limit) ) {
      row = log_magB.count;
      log_magB.time[row] = timestamp;
      log_magB.dur[row]  = tmr_imu.dur;
      for ( i=0; i<3; i++ )  log_magB.raw    [ row*3 +i ] = magB.raw[i];
      for ( i=0; i<3; i++ )  log_magB.scaled [ row*3 +i ] = magB.scaled[i];
      for ( i=0; i<3; i++ )  log_magB.filter [ row*3 +i ] = magB.filter[i];
      log_magB.count++;
    }
    pthread_mutex_unlock(&magB.mutex);

    // Comp filter B data
    pthread_mutex_lock(&imuB.mutex);
    if ( log_compB.count < log_compB.limit ) {
      row = log_compB.count;
      log_compB.time[row]  = timestamp;
      log_compB.dur[row]   = tmr_imu.dur;
      log_compB.roll[row]  = imuB.roll;
      log_compB.pitch[row] = imuB.pitch;
      log_compB.count++;
    }
    pthread_mutex_unlock(&imuB.mutex);

    return;


  // Record AHRSA data
  case LOG_AHRSA :

    timestamp = (float) ( tmr_imu.start_sec + ( tmr_imu.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ahrsA.count < log_ahrsA.limit ) {
      row = log_ahrsA.count;
      log_ahrsA.time[row] = timestamp;
      log_ahrsA.dur[row]  = tmr_imu.dur;
      pthread_mutex_lock(&ahrsA.mutex);
      for ( i=0; i<4; i++ )  log_ahrsA.quat  [ row*4 +i ] = ahrsA.quat  [i];
      for ( i=0; i<4; i++ )  log_ahrsA.dquat [ row*4 +i ] = ahrsA.dquat [i];
      for ( i=0; i<3; i++ )  log_ahrsA.eul   [ row*3 +i ] = ahrsA.eul   [i];
      for ( i=0; i<3; i++ )  log_ahrsA.deul  [ row*3 +i ] = ahrsA.deul  [i];
      for ( i=0; i<3; i++ )  log_ahrsA.bias  [ row*3 +i ] = ahrsA.bias  [i];
                             log_ahrsA.fx    [ row      ] = ahrsA.fx;
                             log_ahrsA.fz    [ row      ] = ahrsA.fz;
      pthread_mutex_unlock(&ahrsA.mutex);
      log_ahrsA.count++;
    }

    return;


  // Record AHRSB data
  case LOG_AHRSB :

    timestamp = (float) ( tmr_imu.start_sec + ( tmr_imu.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ahrsB.count < log_ahrsB.limit ) {
      row = log_ahrsB.count;
      log_ahrsB.time[row] = timestamp;
      log_ahrsB.dur[row]  = tmr_imu.dur;
      pthread_mutex_lock(&ahrsB.mutex);
      for ( i=0; i<4; i++ )  log_ahrsB.quat  [ row*4 +i ] = ahrsB.quat  [i];
      for ( i=0; i<4; i++ )  log_ahrsB.dquat [ row*4 +i ] = ahrsB.dquat [i];
      for ( i=0; i<3; i++ )  log_ahrsB.eul   [ row*3 +i ] = ahrsB.eul   [i];
      for ( i=0; i<3; i++ )  log_ahrsB.deul  [ row*3 +i ] = ahrsB.deul  [i];
      for ( i=0; i<3; i++ )  log_ahrsB.bias  [ row*3 +i ] = ahrsB.bias  [i];
                             log_ahrsB.fx    [ row      ] = ahrsB.fx;
                             log_ahrsB.fz    [ row      ] = ahrsB.fz;
      pthread_mutex_unlock(&ahrsB.mutex);
      log_ahrsB.count++;
    }

    return;


  // Record STAB data
  case LOG_STAB :

    timestamp = (float) ( tmr_stab.start_sec + ( tmr_stab.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_stab.count < log_stab.limit ) {
      row = log_stab.count;
      log_stab.time[row] = timestamp;
      log_stab.dur[row]  = tmr_stab.dur;

      // Rotational state values
      pthread_mutex_lock(&rot.mutex);
      for ( i=0; i<3; i++ )  log_stab.att [ row*3 +i ] = rot.att[i];
      for ( i=0; i<3; i++ )  log_stab.ang [ row*3 +i ] = rot.ang[i];
      pthread_mutex_unlock(&rot.mutex);

      // Stabilization values
      pthread_mutex_lock(&stab.mutex);
      for ( i=0; i<4; i++ )  log_stab.cmd   [ row*4 +i ] = stab.cmd[i];
      pthread_mutex_unlock(&stab.mutex);

      // Roll PID values
      pthread_mutex_lock(&pidX.mutex);
      log_pidX.perr [row] = pidX.perr;
      log_pidX.ierr [row] = pidX.ierr;
      log_pidX.derr [row] = pidX.derr;
      pthread_mutex_unlock(&pidX.mutex);

      // Pitch PID values
      pthread_mutex_lock(&pidY.mutex);
      log_pidY.perr [row] = pidY.perr;
      log_pidY.ierr [row] = pidY.ierr;
      log_pidY.derr [row] = pidY.derr;
      pthread_mutex_unlock(&pidY.mutex);

      // Yaw PID values
      pthread_mutex_lock(&pidZ.mutex);
      log_pidZ.perr [row] = pidZ.perr;
      log_pidZ.ierr [row] = pidZ.ierr;
      log_pidZ.derr [row] = pidZ.derr;
      pthread_mutex_unlock(&pidZ.mutex);

      log_stab.count++;
    }

    return;

    /*
  // Record EKF data
  case LOG_EKF :

    timestamp = (float) ( tmr_ekf.start_sec + ( tmr_ekf.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ekf.count < log_ekf.limit ) {

      row = log_ekf.count;
      log_ekf.time[row] = timestamp;
      log_ekf.dur[row]  = tmr_ekf.dur;

      uint n = EKF_N;
      uint m = EKF_M;

      pthread_mutex_lock(&ekf.mutex);
      for ( i=0; i<n;   i++ )  log_ekf.x [ row*n   +i ] = mat_get( ekf.x, i+1, 1 );
      for ( i=0; i<m;   i++ )  log_ekf.z [ row*m   +i ] = mat_get( ekf.z, i+1, 1 );
      for ( i=0; i<n;   i++ )  log_ekf.f [ row*n   +i ] = mat_get( ekf.f, i+1, 1 );
      for ( i=0; i<m;   i++ )  log_ekf.h [ row*m   +i ] = mat_get( ekf.h, i+1, 1 );
      //for ( i=0; i<n*n; i++ )  log_ekf.P [ row*n*n +i ] = mat_get( ekf.P, (i/n)+1, (i%n)+1 );
      //for ( i=0; i<m*m; i++ )  log_ekf.S [ row*m*m +i ] = mat_get( ekf.S, (i/m)+1, (i%m)+1 );
      //for ( i=0; i<n*m; i++ )  log_ekf.K [ row*n*m +i ] = mat_get( ekf.K, (i/n)+1, (i%n)+1 );
      pthread_mutex_unlock(&ekf.mutex);

      log_ekf.count++;
    }

    return;
    */

    /*
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
    */

  default :
    return;

  }
}


/**
 *  log_finish
 *  Finishes a datalog session by writing out collected data.
 */
void log_finish ( void )  {

  // Inidcate the download is in progress
  datalog.saving = true;
  led_blink( LED_LOG, 200, 200 );
  usleep(200000);

  // Transfer data from memory to file
  log_save();
  log_close();

  // Switch datalog setup flags
  datalog.setup = false;
  datalog.saving = false;
  led_off( LED_LOG );

  return;
}


/**
 *  log_save
 *  Saves each datalog from memory to file.
 */
static void log_save ( void )  {

  // Local variables
  ushort i;
  ulong  row;

  // Parameter data
  for ( row = 0; row < log_param.count; row++ )  {
    fprintf( datalog.param, "\n %011.6f     ", log_param.time[row] );
    for ( i=0; i < param_count; i++ )  fprintf( datalog.param, "%09.4f  ", log_param.values [ row * param_count + i ] );
  }

  // Input data
  for ( row = 0; row < log_input.count; row++ ) {
    fprintf( datalog.in, "\n %011.6f    ", log_input.time[row] );
    for ( i=0; i<10; i++ )  fprintf( datalog.in, "%7.4f  ", log_input.data [ row*10 +i ] );   fprintf( datalog.in, "    " );
  }

  // Output data
  for ( row = 0; row < log_output.count; row++ ) {
    fprintf( datalog.out, "\n %011.6f    ", log_output.time[row] );
    for ( i=0; i<10; i++ )  fprintf( datalog.out, "%7.4f  ", log_output.data [ row*10 +i ] );   fprintf( datalog.out, "    " );
  }

  // IMU A datalogs
  if(IMUA_ENABLED)  {

  // Gyroscope A data
  for ( row = 0; row < log_gyrA.count; row++ ) {
    fprintf( datalog.gyrA, "\n %011.6f   %06ld      ", log_gyrA.time[row], log_gyrA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%06d  ",   log_gyrA.raw    [ row*3 +i ] );   fprintf( datalog.gyrA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%07.4f  ", log_gyrA.scaled [ row*3 +i ] );   fprintf( datalog.gyrA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%07.4f  ", log_gyrA.filter [ row*3 +i ] );   fprintf( datalog.gyrA, "    " );
  }

  // Accelerometer A data
  for ( row = 0; row < log_accA.count; row++ ) {
    fprintf( datalog.accA, "\n %011.6f   %06ld      ", log_accA.time[row], log_accA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%06d  ",   log_accA.raw    [ row*3 +i ] );   fprintf( datalog.accA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%07.4f  ", log_accA.scaled [ row*3 +i ] );   fprintf( datalog.accA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%07.4f  ", log_accA.filter [ row*3 +i ] );   fprintf( datalog.accA, "    " );
  }

  // Magnetometer A data
  for ( row = 0; row < log_magA.count; row++ ) {
    fprintf( datalog.magA, "\n %011.6f   %06ld      ", log_magA.time[row], log_magA.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%06d  ",   log_magA.raw    [ row*3 +i ] );   fprintf( datalog.magA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%07.4f  ", log_magA.scaled [ row*3 +i ] );   fprintf( datalog.magA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%07.4f  ", log_magA.filter [ row*3 +i ] );   fprintf( datalog.magA, "    " );
  }

  // Complimentary filter A data
  for ( row = 0; row < log_compA.count; row++ ) {
    fprintf( datalog.compA, "\n %011.6f   %06ld      ", log_compA.time[row], log_compA.dur[row] );
    fprintf( datalog.compA, "%07.4f    ", log_compA.roll[row]  );
    fprintf( datalog.compA, "%07.4f    ", log_compA.pitch[row] );
  }

  // Attitude/Heading Reference System A data
  for ( row = 0; row < log_ahrsA.count; row++ ) {
    fprintf( datalog.ahrsA, "\n %011.6f   %06ld      ", log_ahrsA.time[row], log_ahrsA.dur[row] );
    for ( i=0; i<4; i++ )  fprintf( datalog.ahrsA, "%07.4f  ", log_ahrsA.quat   [ row*4 +i ] );  fprintf( datalog.ahrsA, "    " );
    for ( i=0; i<4; i++ )  fprintf( datalog.ahrsA, "%07.4f  ", log_ahrsA.dquat  [ row*4 +i ] );  fprintf( datalog.ahrsA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrsA, "%07.4f  ", log_ahrsA.eul    [ row*3 +i ] );  fprintf( datalog.ahrsA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrsA, "%07.4f  ", log_ahrsA.deul   [ row*3 +i ] );  fprintf( datalog.ahrsA, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrsA, "%07.4f  ", log_ahrsA.bias   [ row*3 +i ] );  fprintf( datalog.ahrsA, "    " );
    fprintf( datalog.ahrsA, "%07.4f  ", log_ahrsA.fx[row] );
    fprintf( datalog.ahrsA, "%07.4f  ", log_ahrsA.fz[row] );
    fprintf( datalog.ahrsA, "   " );
  }

  }

  // IMU B datalogs
  if (IMUB_ENABLED)  {

  // Gyroscope B data
  for ( row = 0; row < log_gyrB.count; row++ ) {
    fprintf( datalog.gyrB, "\n %011.6f   %06ld      ", log_gyrB.time[row], log_gyrB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%06d  ",   log_gyrB.raw    [ row*3 +i ] );   fprintf( datalog.gyrB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%07.4f  ", log_gyrB.scaled [ row*3 +i ] );   fprintf( datalog.gyrB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%07.4f  ", log_gyrB.filter [ row*3 +i ] );   fprintf( datalog.gyrB, "    " );
  }

  // Accelerometer B data
  for ( row = 0; row < log_accB.count; row++ ) {
    fprintf( datalog.accB, "\n %011.6f   %06ld      ", log_accB.time[row], log_accB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%06d  ",   log_accB.raw    [ row*3 +i ] );   fprintf( datalog.accB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%07.4f  ", log_accB.scaled [ row*3 +i ] );   fprintf( datalog.accB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%07.4f  ", log_accB.filter [ row*3 +i ] );   fprintf( datalog.accB, "    " );
  }

  // Magnetometer B data
  for ( row = 0; row < log_magB.count; row++ ) {
    fprintf( datalog.magB, "\n %011.6f   %06ld      ", log_magB.time[row], log_magB.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%06d  ",   log_magB.raw    [ row*3 +i ] );   fprintf( datalog.magB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%07.4f  ", log_magB.scaled [ row*3 +i ] );   fprintf( datalog.magB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%07.4f  ", log_magB.filter [ row*3 +i ] );   fprintf( datalog.magB, "    " );
  }

  // Complimentary filter B data
  for ( row = 0; row < log_compB.count; row++ ) {
    fprintf( datalog.compB, "\n %011.6f   %06ld      ", log_compB.time[row], log_compB.dur[row] );
    fprintf( datalog.compB, "%07.4f    ", log_compB.roll[row]  );
    fprintf( datalog.compB, "%07.4f    ", log_compB.pitch[row] );
  }

  // Attitude/Heading Reference System B data
  for ( row = 0; row < log_ahrsB.count; row++ ) {
    fprintf( datalog.ahrsB, "\n %011.6f   %06ld      ", log_ahrsB.time[row], log_ahrsB.dur[row] );
    for ( i=0; i<4; i++ )  fprintf( datalog.ahrsB, "%07.4f  ", log_ahrsB.quat   [ row*4 +i ] );  fprintf( datalog.ahrsB, "    " );
    for ( i=0; i<4; i++ )  fprintf( datalog.ahrsB, "%07.4f  ", log_ahrsB.dquat  [ row*4 +i ] );  fprintf( datalog.ahrsB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrsB, "%07.4f  ", log_ahrsB.eul    [ row*3 +i ] );  fprintf( datalog.ahrsB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrsB, "%07.4f  ", log_ahrsB.deul   [ row*3 +i ] );  fprintf( datalog.ahrsB, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.ahrsB, "%07.4f  ", log_ahrsB.bias   [ row*3 +i ] );  fprintf( datalog.ahrsB, "    " );
    fprintf( datalog.ahrsB, "%07.4f  ", log_ahrsB.fx[row] );
    fprintf( datalog.ahrsB, "%07.4f  ", log_ahrsB.fz[row] );
    fprintf( datalog.ahrsB, "   " );
  }

  }

  // Stabilization data
  for ( row = 0; row < log_stab.count; row++ )  {
    fprintf( datalog.stab, "\n %011.6f   %06ld      ", log_stab.time[row], log_stab.dur[row] );
    for ( i=0; i<3; i++ )  fprintf( datalog.stab, "%07.4f  ",  log_stab.att [ row*3 +i ] );   fprintf( datalog.stab, "    " );
    for ( i=0; i<3; i++ )  fprintf( datalog.stab, "%07.4f  ",  log_stab.ang [ row*3 +i ] );   fprintf( datalog.stab, "    " );
    for ( i=0; i<4; i++ )  fprintf( datalog.stab, "%07.4f  ",  log_stab.cmd [ row*4 +i ] );   fprintf( datalog.stab, "    " );
    fprintf( datalog.stab, "%07.4f  ",  log_pidX.perr[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_pidX.ierr[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_pidX.derr[row] );
    fprintf( datalog.stab, "    " );
    fprintf( datalog.stab, "%07.4f  ",  log_pidY.perr[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_pidY.ierr[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_pidY.derr[row] );
    fprintf( datalog.stab, "    " );
    fprintf( datalog.stab, "%07.4f  ",  log_pidZ.perr[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_pidZ.ierr[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_pidZ.derr[row] );
    fprintf( datalog.stab, "    " );
  }

  // Extended Kalman Filter data
  //uint n = EKF_N;
  //uint m = EKF_M;
  //for ( row = 0; row < log_ekf.count; row++ ) {
    //fprintf( datalog.ekf, "\n %011.6f   %06ld      ", log_ekf.time[row], log_ekf.dur[row] );
    //for ( i=0; i<n;   i++ )  fprintf( datalog.ekf, "%07.4f  ", log_ekf.x [ row*n   +i ] );  fprintf( datalog.ekf, "    " );
    //for ( i=0; i<m;   i++ )  fprintf( datalog.ekf, "%07.4f  ", log_ekf.z [ row*m   +i ] );  fprintf( datalog.ekf, "    " );
    //for ( i=0; i<n;   i++ )  fprintf( datalog.ekf, "%07.4f  ", log_ekf.f [ row*n   +i ] );  fprintf( datalog.ekf, "    " );
    //for ( i=0; i<m;   i++ )  fprintf( datalog.ekf, "%07.4f  ", log_ekf.h [ row*m   +i ] );  fprintf( datalog.ekf, "    " );
    //for ( i=0; i<n*n; i++ )  fprintf( datalog.ekf, "%07.4f  ", log_ekf.P [ row*n*n +i ] );  fprintf( datalog.ekf, "    " );
    //for ( i=0; i<m*m; i++ )  fprintf( datalog.ekf, "%07.4f  ", log_ekf.S [ row*m*m +i ] );  fprintf( datalog.ekf, "    " );
    //for ( i=0; i<n*m; i++ )  fprintf( datalog.ekf, "%07.4f  ", log_ekf.K [ row*n*m +i ] );  fprintf( datalog.ekf, "    " );
  //}

  /*
  // GPS data
  for ( row = 0; row < log_gps.count; row++ ) {
    fprintf( datalog.gps, "\n %011.6f  %06ld    ", log_gps.time[row], log_gps.dur[row] );
    fprintf( datalog.gps, "%s ", &log_gps.msg[row*96] );//  fprintf( fgps, "    " );
    fprintf( datalog.gps, "   " );
  }
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
    fclose(datalog.compA);
    fclose(datalog.ahrsA);
  }

  if (IMUB_ENABLED)  {
    fclose(datalog.gyrB);
    fclose(datalog.accB);
    fclose(datalog.magB);
    fclose(datalog.compB);
    fclose(datalog.ahrsB);
  }

  fclose(datalog.stab);
  //fclose(datalog.ekf);
  //fclose(datalog.gps);

  return;
}



