

#include "log.h"
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include "gcs.h"
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

  // Set boolean values
  if(DEBUG)  printf("  Set boolean conditions \n");
  datalog.enabled  = false;
  datalog.setup    = false;
  datalog.saving   = false;

  // Assign datalog limits
  log_param.limit   =  LOG_MAX_PARAM;  
  log_input.limit   =  LOG_MAX_DUR * HZ_IO;
  log_output.limit  =  LOG_MAX_DUR * HZ_IO;
  log_imu.limit     =  LOG_MAX_DUR * HZ_IMU;
  //log_stab.limit    =  LOG_MAX_DUR * HZ_STAB;
  //log_ins.limit     =  LOG_MAX_DUR * HZ_INS;
  //log_nav.limit     =  LOG_MAX_DUR * HZ_NAV;

  // Parameter value setup
  log_param.time    =  malloc( sizeof(float)  * log_param.limit               );
  log_param.values  =  malloc( sizeof(float)  * log_param.limit * param_count );

  // Input signal setup
  log_input.time    =  malloc( sizeof(float)  * log_input.limit         );
  log_input.data    =  malloc( sizeof(float)  * log_input.limit * IN_CH );

  // Output signal setup
  log_output.time   =  malloc( sizeof(float)  * log_output.limit          );
  log_output.data   =  malloc( sizeof(float)  * log_output.limit * OUT_CH );

  // Timestamp setup
  log_imu.time      = malloc( sizeof(float)  * log_imu.limit  );
  //log_stab.time     = malloc( sizeof(float)  * log_stab.limit );
  //log_ins.time      = malloc( sizeof(float)  * log_ins.limit  );
  //log_nav.time      = malloc( sizeof(float)  * log_nav.limit  );

  // Timing loop duration setup
  log_imu.dur       = malloc( sizeof(ulong)  * log_imu.limit  );
  //log_stab.dur      = malloc( sizeof(ulong)  * log_stab.limit );
  //log_ins.dur       = malloc( sizeof(ulong)  * log_ins.limit  );
  //log_nav.dur       = malloc( sizeof(ulong)  * log_nag.limit  );

  // IMU A setup
  if (IMUA_ENABLED) {

  // Gyroscope A setup
  log_gyrA.raw      =  malloc( sizeof(short)  * log_imu.limit * 3 );
  log_gyrA.scaled   =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_gyrA.filter   =  malloc( sizeof(float)  * log_imu.limit * 3 );

  // Accelerometer A setup
  log_accA.raw      =  malloc( sizeof(short)  * log_imu.limit * 3 );
  log_accA.scaled   =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_accA.filter   =  malloc( sizeof(float)  * log_imu.limit * 3 );

  // Magnetometer A setup
  log_magA.raw      =  malloc( sizeof(short)  * log_imu.limit * 3 );
  log_magA.scaled   =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_magA.filter   =  malloc( sizeof(float)  * log_imu.limit * 3 );

  // Complimentary filter A setup
  log_compA.roll    =  malloc( sizeof(float)  * log_imu.limit );
  log_compA.pitch   =  malloc( sizeof(float)  * log_imu.limit );

  // Attitude and Heading Reference System A setup
  log_ahrsA.quat    =  malloc( sizeof(float)  * log_imu.limit * 4 );
  log_ahrsA.dquat   =  malloc( sizeof(float)  * log_imu.limit * 4 );
  log_ahrsA.eul     =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_ahrsA.deul    =  malloc( sizeof(float)  * log_imu.limit * 3 );

  }

  // IMU B setup
  if (IMUB_ENABLED) {

  // Gyroscope B setup
  log_gyrB.raw      =  malloc( sizeof(short)  * log_imu.limit * 3 );
  log_gyrB.scaled   =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_gyrB.filter   =  malloc( sizeof(float)  * log_imu.limit * 3 );

  // Accelerometer B setup
  log_accB.raw      =  malloc( sizeof(short)  * log_imu.limit * 3 );
  log_accB.scaled   =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_accB.filter   =  malloc( sizeof(float)  * log_imu.limit * 3 );

  // Magnetometer B setup
  log_magB.raw      =  malloc( sizeof(short)  * log_imu.limit * 3 );
  log_magB.scaled   =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_magB.filter   =  malloc( sizeof(float)  * log_imu.limit * 3 );

  // Complimentary filter B setup
  log_compB.roll    =  malloc( sizeof(float)  * log_imu.limit );
  log_compB.pitch   =  malloc( sizeof(float)  * log_imu.limit );

  // Attitude and Heading Reference System B setup
  log_ahrsB.quat    =  malloc( sizeof(float)  * log_imu.limit * 4 );
  log_ahrsB.dquat   =  malloc( sizeof(float)  * log_imu.limit * 4 );
  log_ahrsB.eul     =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_ahrsB.deul    =  malloc( sizeof(float)  * log_imu.limit * 3 );

  }

  // Rotational states setup
  log_rot.att       =  malloc( sizeof(float)  * log_imu.limit * 3 );
  log_rot.ang       =  malloc( sizeof(float)  * log_imu.limit * 3 );


  // Stabilization setup
  //log_stab.time     =  malloc( sizeof(float)  * log_stab.limit     );
  //log_stab.dur      =  malloc( sizeof(ulong)  * log_stab.limit     );

  // SF roll stabilization
  //log_sfX.r         =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfX.xp        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfX.xd        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfX.u         =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfX.kp        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfX.kd        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfX.ku        =  malloc( sizeof(float)  * log_stab.limit );

  // SF pitch stabilization
  //log_sfY.r         =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfY.xp        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfY.xd        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfY.u         =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfY.kp        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfY.kd        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfY.ku        =  malloc( sizeof(float)  * log_stab.limit );

  // SF yaw stabilization
  //log_sfZ.r         =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfZ.xp        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfZ.xd        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfZ.u         =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfZ.kp        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfZ.kd        =  malloc( sizeof(float)  * log_stab.limit );
  //log_sfZ.ku        =  malloc( sizeof(float)  * log_stab.limit );

  // SysID roll axis
  //log_sysidX.z1     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidX.z2     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidX.p1     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidX.p2     =  malloc( sizeof(float)  * log_stab.limit );

  // SysID pitch axis
  //log_sysidY.z1     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidY.z2     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidY.p1     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidY.p2     =  malloc( sizeof(float)  * log_stab.limit );

  // SysID roll axis
  //log_sysidZ.z1     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidZ.z2     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidZ.p1     =  malloc( sizeof(float)  * log_stab.limit );
  //log_sysidZ.p2     =  malloc( sizeof(float)  * log_stab.limit );

  /*
  // EKF setup
  ushort n = EKF_N, m = EKF_M;
  log_ekf.time      =  malloc( sizeof(float)  * log_ekf.limit       );
  log_ekf.dur       =  malloc( sizeof(ulong)  * log_ekf.limit       );
  log_ekf.x         =  malloc( sizeof(float)  * log_ekf.limit *  n  );
  log_ekf.z         =  malloc( sizeof(float)  * log_ekf.limit *  m  );
  log_ekf.f         =  malloc( sizeof(float)  * log_ekf.limit *  n  );
  log_ekf.h         =  malloc( sizeof(float)  * log_ekf.limit *  m  );
  log_ekf.F         =  malloc( sizeof(float)  * log_ekf.limit * n*n );
  log_ekf.P         =  malloc( sizeof(float)  * log_ekf.limit * n*n );
  log_ekf.T         =  malloc( sizeof(float)  * log_ekf.limit * n*n );
  log_ekf.S         =  malloc( sizeof(float)  * log_ekf.limit * m*m );
  */

  /*
  // INS setup
  log_ins.time      =  malloc( sizeof(float)  * log_ins.limit       );
  log_ins.dur       =  malloc( sizeof(ulong)  * log_ins.limit       );
  log_ins.K         =  malloc( sizeof(float)  * log_ins.limit * n*m );
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


  // Timestamp memory
  free(log_imu.time);
  //free(log_stab.time);
  //free(log_ins.time);
  //free(log_nav.time);

  // Timing loop duration memory
  free(log_imu.dur);
  //free(log_stab.dur);
  //free(log_ins.dur);
  //free(log_nav.dur);

  // IMU A memory
  if (IMUA_ENABLED) {

  // Gyroscope A memory
  free(log_gyrA.raw);
  free(log_gyrA.scaled);
  free(log_gyrA.filter);

  // Accelerometer A memory
  free(log_accA.raw);
  free(log_accA.scaled);
  free(log_accA.filter);

  // Magnetometer A memory
  free(log_magA.raw);
  free(log_magA.scaled);
  free(log_magA.filter);

  // Comp filter A memory
  free(log_compA.roll);
  free(log_compA.pitch);

  // Attitude/Heading A memory
  free(log_ahrsA.quat);
  free(log_ahrsA.dquat);
  free(log_ahrsA.eul);
  free(log_ahrsA.deul);

  }

  // IMU B memory
  if (IMUB_ENABLED) {

  // Gyroscope B memory
  free(log_gyrB.raw);
  free(log_gyrB.scaled);
  free(log_gyrB.filter);

  // Accelerometer B memory
  free(log_accB.raw);
  free(log_accB.scaled);
  free(log_accB.filter);

  // Magnetometer B memory
  free(log_magB.raw);
  free(log_magB.scaled);
  free(log_magB.filter);

  // Comp filter B memory
  free(log_compB.roll);
  free(log_compB.pitch);

  // Attitude/Heading B memory
  free(log_ahrsB.quat);
  free(log_ahrsB.dquat);
  free(log_ahrsB.eul);
  free(log_ahrsB.deul);

  }

  // Rotational states memory
  free(log_rot.att);
  free(log_rot.ang);

  // Stabilization memory
  //free(log_stab.time);
  //free(log_stab.dur);

  // SF roll stab memory
  //free(log_sfX.r);
  //free(log_sfX.xp);
  //free(log_sfX.xd);
  //free(log_sfX.u);
  //free(log_sfX.kp);
  //free(log_sfX.kd);
  //free(log_sfX.ku);

  // SF pitch stab memory
  //free(log_sfY.r);
  //free(log_sfY.xp);
  //free(log_sfY.xd);
  //free(log_sfY.u);
  //free(log_sfY.kp);
  //free(log_sfY.kd);
  //free(log_sfY.ku);

  // SF yaw stab memory
  //free(log_sfZ.r);
  //free(log_sfZ.xp);
  //free(log_sfZ.xd);
  //free(log_sfZ.u);
  //free(log_sfZ.kp);
  //free(log_sfZ.kd);
  //free(log_sfZ.ku);

  // SysID roll memory
  //free(log_sysidX.z1);
  //free(log_sysidX.z2);
  //free(log_sysidX.p1);
  //free(log_sysidX.p2);

  // SysID pitch memory
  //free(log_sysidY.z1);
  //free(log_sysidY.z2);
  //free(log_sysidY.p1);
  //free(log_sysidY.p2);

  // SysID yaw memory
  //free(log_sysidZ.z1);
  //free(log_sysidZ.z2);
  //free(log_sysidZ.p1);
  //free(log_sysidZ.p2);

  /*
  // EKF memory
  free(log_ekf.time);
  free(log_ekf.dur);
  free(log_ekf.x);
  free(log_ekf.z);
  free(log_ekf.f);
  free(log_ekf.h);
  free(log_ekf.F);
  free(log_ekf.P);
  free(log_ekf.T);
  free(log_ekf.S);
  */

  /*
  // INS memory
  free(log_ins.time);
  free(log_ins.dur);
  free(log_ins.K);
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
  log_imu.count    = 0;
  //log_stab.count   = 0;
  //log_ins.count    = 0;
  //log_nav.count    = 0;

  // Allocate dir/path/file memory
  datalog.dir  = malloc(16);
  datalog.path = malloc(32);
  char *file   = malloc(64);

  // Find next available log directory
  ushort i = 0; // r, c;
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
    "         in_time   \
    in_01     in_02     in_03     in_04     in_05 \
    in_06     in_07     in_08     in_09     in_10   " );

  // Output datalog file
  sprintf( file, "%soutput.txt", datalog.path );
  datalog.out = fopen( file, "w" );
  if( datalog.out == NULL )  printf( "Error (log_init): Cannot generate 'output' file. \n" );
  fprintf( datalog.out, 
    "        out_time  \
    out_01    out_02    out_03    out_04    out_05\
    out_06    out_07    out_08    out_09    out_10   " );

  // IMU timing thread datalog file
  sprintf( file, "%simu.txt", datalog.path );
  datalog.imu = fopen( file, "w" );
  if( datalog.imu == NULL )  printf( "Error (log_init): Cannot generate 'imu' file. \n" );
  fprintf( datalog.imu, "        imu_time    imu_dur   ");

  // IMUA datalogs
  if (IMUA_ENABLED)  {

  // Gyroscope A datalog file
  sprintf( file, "%sgyra.txt", datalog.path );
  datalog.gyrA = fopen( file, "w" );
  if( datalog.gyrA == NULL )  printf( "Error (log_init): Cannot generate 'gyra' file. \n" );
  fprintf( datalog.gyrA, "\
    gyr_arx  gyr_ary  gyr_arz   \
    gyr_asx   gyr_asy   gyr_asz   \
    gyr_afx   gyr_afy   gyr_afz");

  // Accelerometer A datalog file
  sprintf( file, "%sacca.txt", datalog.path );
  datalog.accA = fopen( file, "w" );
  if( datalog.accA == NULL )  printf( "Error (log_init): Cannot generate 'acca' file. \n" );
  fprintf( datalog.accA, "\
    acc_arx  acc_ary  acc_arz   \
    acc_asx   acc_asy   acc_asz   \
    acc_afx   acc_afy   acc_afz");

  // Magnetometer A datalog file
  sprintf( file, "%smaga.txt", datalog.path );
  datalog.magA = fopen( file, "w" );
  if( datalog.magA == NULL )  printf( "Error (log_init): Cannot generate 'maga' file. \n" );
  fprintf( datalog.magA, "\
    mag_arx  mag_ary  mag_arz   \
    mag_asx   mag_asy   mag_asz   \
    mag_afx   mag_afy   mag_afz");

  // Comp filter A datalog file
  sprintf( file, "%scompa.txt", datalog.path );
  datalog.compA = fopen( file, "w" );
  if( datalog.compA == NULL )  printf( "Error (log_init): Cannot generate 'compa' file. \n" );
  fprintf( datalog.compA, "  comp_ar comp_ap ");  

  // Attitude and heading reference system A datalog file
  sprintf( file, "%sahrsa.txt", datalog.path );
  datalog.ahrsA = fopen( file, "w" );
  if( datalog.ahrsA == NULL )  printf( "Error (log_init): Cannot generate 'ahrsa' file. \n" );
  fprintf( datalog.ahrsA, "\
    quat_aw   quat_ax   quat_ay   quat_az  \
    dquat_aw  dquat_ax  dquat_ay  dquat_az    \
    eul_ax    eul_ay    eul_az   \
    deul_ax   deul_ay   deul_az");

  }

  // IMUB datalogs
  if (IMUB_ENABLED )  {

  // Gyroscope B datalog file
  sprintf( file, "%sgyrb.txt", datalog.path );
  datalog.gyrB = fopen( file, "w" );
  if( datalog.gyrB == NULL )  printf( "Error (log_init): Cannot generate 'gyrb' file. \n" );
  fprintf( datalog.gyrB, "\
    gyr_brx  gyr_bry  gyr_brz   \
    gyr_bsx   gyr_bsy   gyr_bsz   \
    gyr_bfx   gyr_bfy   gyr_bfz");

  // Accelerometer B datalog file
  sprintf( file, "%saccb.txt", datalog.path );
  datalog.accB = fopen( file, "w" );
  if( datalog.accB == NULL )  printf( "Error (log_init): Cannot generate 'accb' file. \n" );
  fprintf( datalog.accB, "\
    acc_brx  acc_bry  acc_brz   \
    acc_bsx   acc_bsy   acc_bsz   \
    acc_bfx   acc_bfy   acc_bfz");

  // Magnetometer B datalog file
  sprintf( file, "%smagb.txt", datalog.path );
  datalog.magB = fopen( file, "w" );
  if( datalog.magB == NULL )  printf( "Error (log_init): Cannot generate 'magb' file. \n" );
  fprintf( datalog.magB, "\
    mag_brx  mag_bry  mag_brz   \
    mag_bsx   mag_bsy   mag_bsz   \
    mag_bfx   mag_bfy   mag_bfz");

  // Comp filter B datalog file
  sprintf( file, "%scompb.txt", datalog.path );
  datalog.compB = fopen( file, "w" );
  if( datalog.compB == NULL )  printf( "Error (log_init): Cannot generate 'compb' file. \n" );
  fprintf( datalog.compB, "    comp_br comp_bp ");

  // Attitude and heading reference system B datalog file
  sprintf( file, "%sahrsb.txt", datalog.path );
  datalog.ahrsB = fopen( file, "w" );
  if( datalog.ahrsB == NULL )  printf( "Error (log_init): Cannot generate 'ahrsb' file. \n" );
  fprintf( datalog.ahrsB, "\
    quat_bw   quat_bx   quat_by   quat_bz  \
    dquat_bw  dquat_bx  dquat_by  dquat_bz    \
    eul_bx    eul_by    eul_bz   \
    deul_bx   deul_by   deul_bz");

  }

  // Rotational states datalog file
  sprintf( file, "%srot.txt", datalog.path );
  datalog.rot = fopen( file, "w" );
  if( datalog.rot == NULL )  printf( "Error (log_init): Cannot generate 'rot' file. \n" );
  fprintf( datalog.rot, "   \
    att_x     att_y     att_z      \
    ang_x     ang_y     ang_z        ");


  /*
  // Stabilization datalog file
  sprintf( file, "%sstab.txt", datalog.path );
  datalog.stab = fopen( file, "w" );
  if( datalog.stab == NULL )  printf( "Error (log_init): Cannot generate 'stab' file. \n" );
  fprintf( datalog.stab, 
    "    stabtime  stabdur      \
    X_r     X_xp     X_xd      X_u     X_kp     X_kd     X_ku      \
    Y_r     Y_xp     Y_xd      Y_u     Y_kp     Y_kd     Y_ku      \
    Z_r     Z_xp     Z_xd      Z_u     Z_kp     Z_kd     Z_ku   " );

  // System identification datalog file
  sprintf( file, "%ssysid.txt", datalog.path );
  datalog.sysid = fopen( file, "w" );
  if( datalog.sysid == NULL )  printf( "Error (log_init): Cannot generate 'sysid' file. \n" );
  fprintf( datalog.sysid, 
    "   sysidtime sysiddur     \
    X_z1     X_z2     X_p1     X_p2     \
    Y_z1     Y_z2     Y_p1     Y_p2     \
    Z_z1     Z_z2     Z_p1     Z_p2    " );
  */

  /*
  // EKF datalog file
  ushort n = EKF_N, m = EKF_M;
  sprintf( file, "%sekf.txt", datalog.path );
  datalog.ekf = fopen( file, "w" );
  if( datalog.ekf == NULL )  printf( "Error (log_init): Cannot generate 'ekf' file. \n" );
  fprintf( datalog.ekf, "     ekftime   ekfdur    " );
  for ( r=1; r<=n; r++ )  fprintf( datalog.ekf, "      x%02d", r );  fprintf( datalog.ekf, "    " );
  for ( r=1; r<=m; r++ )  fprintf( datalog.ekf, "      z%02d", r );  fprintf( datalog.ekf, "    " );
  for ( r=1; r<=n; r++ )  fprintf( datalog.ekf, "      f%02d", r );  fprintf( datalog.ekf, "    " );
  for ( r=1; r<=m; r++ )  fprintf( datalog.ekf, "      h%02d", r );  fprintf( datalog.ekf, "    " );
  for ( r=1; r<=n; r++ )  for ( c=1; c<=n; c++ )  fprintf( datalog.ekf, "    F%02d%02d", r, c );  fprintf( datalog.ekf, "    " );
  //for ( r=1; r<=n; r++ )  for ( c=1; c<=n; c++ )  fprintf( datalog.ekf, "    P%02d%02d", r, c );  fprintf( datalog.ekf, "    " );
  //for ( r=1; r<=n; r++ )  for ( c=1; c<=n; c++ )  fprintf( datalog.ekf, "    T%02d%02d", r, c );  fprintf( datalog.ekf, "    " );
  //for ( r=1; r<=m; r++ )  for ( c=1; c<=m; c++ )  fprintf( datalog.ekf, "    S%02d%02d", r, c );  fprintf( datalog.ekf, "    " );
  */

  /*
  // INS datalog file
  sprintf( file, "%sins.txt", datalog.path );
  datalog.ins = fopen( file, "w" );
  if( datalog.ins == NULL )  printf( "Error (log_init): Cannot generate 'ins' file. \n" );
  fprintf( datalog.ins, "     instime   insdur    " );
  for ( r=1; r<=n; r++ )  for ( c=1; c<=m; c++ )  fprintf( datalog.ins, "    K%02d%02d", r, c );  fprintf( datalog.ins, "    " );
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
  //struct timespec t;

  // Jump to appropriate log 
  switch(index) {


  // Record parameter values
  case LOG_PARAM :
    /*
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
    */
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


  // Record IMU data
  case LOG_IMU :

    // Check memory limit
    if ( log_imu.count < log_imu.limit )  {

      // IMU timing data
      row = log_imu.count;
      timestamp = (float) ( tmr_imu.start_sec  + ( tmr_imu.start_usec  / 1000000.0f ) ) - datalog.offset;
      log_imu.time[row] = timestamp;
      log_imu.dur[row]  = tmr_imu.dur;

      // Record IMUA data
      if (IMUA_ENABLED)  {

        // Gyroscope A data
        pthread_mutex_lock(&gyrA.mutex);
        for ( i=0; i<3; i++ )  log_gyrA.raw    [ row*3 +i ] = gyrA.raw[i];
        for ( i=0; i<3; i++ )  log_gyrA.scaled [ row*3 +i ] = gyrA.scaled[i];
        for ( i=0; i<3; i++ )  log_gyrA.filter [ row*3 +i ] = gyrA.filter[i];
        pthread_mutex_unlock(&gyrA.mutex);

        // Accelerometer A data
        pthread_mutex_lock(&accA.mutex);
        for ( i=0; i<3; i++ )  log_accA.raw    [ row*3 +i ] = accA.raw[i];
        for ( i=0; i<3; i++ )  log_accA.scaled [ row*3 +i ] = accA.scaled[i];
        for ( i=0; i<3; i++ )  log_accA.filter [ row*3 +i ] = accA.filter[i];
        pthread_mutex_unlock(&accA.mutex);

        // Magnetometer A data
        pthread_mutex_lock(&magA.mutex);
        for ( i=0; i<3; i++ )  log_magA.raw    [ row*3 +i ] = magA.raw[i];
        for ( i=0; i<3; i++ )  log_magA.scaled [ row*3 +i ] = magA.scaled[i];
        for ( i=0; i<3; i++ )  log_magA.filter [ row*3 +i ] = magA.filter[i];
        pthread_mutex_unlock(&magA.mutex);

        // Comp filter A data
        pthread_mutex_lock(&compA.mutex);
        log_compA.roll[row]  = compA.roll;
        log_compA.pitch[row] = compA.pitch;
        pthread_mutex_unlock(&compA.mutex);

        // AHRS A data
        pthread_mutex_lock(&ahrsA.mutex);
        for ( i=0; i<4; i++ )  log_ahrsA.quat  [ row*4 +i ] = ahrsA.quat  [i];
        for ( i=0; i<4; i++ )  log_ahrsA.dquat [ row*4 +i ] = ahrsA.dquat [i];
        for ( i=0; i<3; i++ )  log_ahrsA.eul   [ row*3 +i ] = ahrsA.eul   [i];
        for ( i=0; i<3; i++ )  log_ahrsA.deul  [ row*3 +i ] = ahrsA.deul  [i];
        pthread_mutex_unlock(&ahrsA.mutex);

      }

      // Record IMUB data
      if (IMUB_ENABLED)  {

        // Gyroscope B data
        pthread_mutex_lock(&gyrB.mutex);
        for ( i=0; i<3; i++ )  log_gyrB.raw    [ row*3 +i ] = gyrB.raw[i];
        for ( i=0; i<3; i++ )  log_gyrB.scaled [ row*3 +i ] = gyrB.scaled[i];
        for ( i=0; i<3; i++ )  log_gyrB.filter [ row*3 +i ] = gyrB.filter[i];
        pthread_mutex_unlock(&gyrB.mutex);

        // Accelerometer B data
        pthread_mutex_lock(&accB.mutex);
        for ( i=0; i<3; i++ )  log_accB.raw    [ row*3 +i ] = accB.raw[i];
        for ( i=0; i<3; i++ )  log_accB.scaled [ row*3 +i ] = accB.scaled[i];
        for ( i=0; i<3; i++ )  log_accB.filter [ row*3 +i ] = accB.filter[i];
        pthread_mutex_unlock(&accB.mutex);

        // Magnetometer B data
        pthread_mutex_lock(&magB.mutex);
        for ( i=0; i<3; i++ )  log_magB.raw    [ row*3 +i ] = magB.raw[i];
        for ( i=0; i<3; i++ )  log_magB.scaled [ row*3 +i ] = magB.scaled[i];
        for ( i=0; i<3; i++ )  log_magB.filter [ row*3 +i ] = magB.filter[i];
        pthread_mutex_unlock(&magB.mutex);

        // Comp filter B data
        pthread_mutex_lock(&compB.mutex);
        log_compB.roll[row]  = compB.roll;
        log_compB.pitch[row] = compB.pitch;
        pthread_mutex_unlock(&compB.mutex);

        // AHRS B data
        pthread_mutex_lock(&ahrsB.mutex);
        for ( i=0; i<4; i++ )  log_ahrsB.quat  [ row*4 +i ] = ahrsB.quat  [i];
        for ( i=0; i<4; i++ )  log_ahrsB.dquat [ row*4 +i ] = ahrsB.dquat [i];
        for ( i=0; i<3; i++ )  log_ahrsB.eul   [ row*3 +i ] = ahrsB.eul   [i];
        for ( i=0; i<3; i++ )  log_ahrsB.deul  [ row*3 +i ] = ahrsB.deul  [i];
        pthread_mutex_unlock(&ahrsB.mutex);

      }

      // Rotational state data
      pthread_mutex_lock(&rot.mutex);
      for ( i=0; i<3; i++ )  log_rot.att [ row*3 +i ] = rot.att[i];
      for ( i=0; i<3; i++ )  log_rot.ang [ row*3 +i ] = rot.ang[i];
      pthread_mutex_unlock(&rot.mutex);

      // Increment log counter
      log_imu.count++;

    }
    return;


  // Record STAB data
  //case LOG_STAB :

    //timestamp = (float) ( tmr_stab.start_sec + ( tmr_stab.start_usec / 1000000.0f ) ) - datalog.offset;

    //if ( log_stab.count < log_stab.limit ) {
      //row = log_stab.count;
      //log_stab.time[row] = timestamp;
      //log_stab.dur[row]  = tmr_stab.dur;

      /*
      // Roll SF values
      pthread_mutex_lock(&sfX.mutex);
      log_sfX.r  [row] = sfX.r;
      log_sfX.xp [row] = sfX.xp;
      log_sfX.xd [row] = sfX.xd;
      log_sfX.u  [row] = sfX.u;
      log_sfX.kp [row] = sfX.kp;
      log_sfX.kd [row] = sfX.kd;
      log_sfX.ku [row] = sfX.ku;
      pthread_mutex_unlock(&sfX.mutex);

      // Pitch SF values
      pthread_mutex_lock(&sfY.mutex);
      log_sfY.r  [row] = sfY.r;
      log_sfY.xp [row] = sfY.xp;
      log_sfY.xd [row] = sfY.xd;
      log_sfY.u  [row] = sfY.u;
      log_sfY.kp [row] = sfY.kp;
      log_sfY.kd [row] = sfY.kd;
      log_sfY.ku [row] = sfY.ku;
      pthread_mutex_unlock(&sfY.mutex);

      // Yaw SF values
      pthread_mutex_lock(&sfZ.mutex);
      log_sfZ.r  [row] = sfZ.r;
      log_sfZ.xp [row] = sfZ.xp;
      log_sfZ.xd [row] = sfZ.xd;
      log_sfZ.u  [row] = sfZ.u;
      log_sfZ.kp [row] = sfZ.kp;
      log_sfZ.kd [row] = sfZ.kd;
      log_sfZ.ku [row] = sfZ.ku;
      pthread_mutex_unlock(&sfZ.mutex);

      // Roll SysID values
      pthread_mutex_lock(&sysidX.mutex);
      log_sysidX.z1 [row] = sysidX.z1;
      log_sysidX.z2 [row] = sysidX.z2;
      log_sysidX.p1 [row] = sysidX.p1;
      log_sysidX.p2 [row] = sysidX.p2;
      pthread_mutex_unlock(&sysidX.mutex);

      // Pitch SysID values
      pthread_mutex_lock(&sysidY.mutex);
      log_sysidY.z1 [row] = sysidY.z1;
      log_sysidY.z2 [row] = sysidY.z2;
      log_sysidY.p1 [row] = sysidY.p1;
      log_sysidY.p2 [row] = sysidY.p2;
      pthread_mutex_unlock(&sysidY.mutex);

      // Yaw SysID values
      pthread_mutex_lock(&sysidZ.mutex);
      log_sysidZ.z1 [row] = sysidZ.z1;
      log_sysidZ.z2 [row] = sysidZ.z2;
      log_sysidZ.p1 [row] = sysidZ.p1;
      log_sysidZ.p2 [row] = sysidZ.p2;
      pthread_mutex_unlock(&sysidZ.mutex);
      */
      /*
      // EKF values
      ushort r, c;
      ushort n = EKF_N, m = EKF_M;
      pthread_mutex_lock(&ekf.mutex);
      for ( r=0; r<n; r++ )                         log_ekf.x [ row*n        +r ] = mat_get( ekf.x, r+1,   1 );
      for ( r=0; r<m; r++ )                         log_ekf.z [ row*m        +r ] = mat_get( ekf.z, r+1,   1 );
      for ( r=0; r<n; r++ )                         log_ekf.f [ row*n        +r ] = mat_get( ekf.f, r+1,   1 );
      for ( r=0; r<m; r++ )                         log_ekf.h [ row*m        +r ] = mat_get( ekf.h, r+1,   1 );
      for ( r=0; r<n; r++ )  for ( c=0; c<n; c++ )  log_ekf.F [ row*n*n +r*n +c ] = mat_get( ekf.F, r+1, c+1 );
      //for ( r=0; r<n; r++ )  for ( c=0; c<n; c++ )  log_ekf.P [ row*n*n +r*n +c ] = mat_get( ekf.P, r+1, c+1 );
      //for ( r=0; r<n; r++ )  for ( c=0; c<n; c++ )  log_ekf.T [ row*n*n +r*n +c ] = mat_get( ekf.T, r+1, c+1 );
      //for ( r=0; r<m; r++ )  for ( c=0; c<m; c++ )  log_ekf.S [ row*m*m +r*m +c ] = mat_get( ekf.S, r+1, c+1 );
      pthread_mutex_unlock(&ekf.mutex);
      */

      //log_stab.count++;
    //}
    //return;


  /*
  // Record INS data
  case LOG_INS :

    timestamp = (float) ( tmr_ins.start_sec + ( tmr_ins.start_usec / 1000000.0f ) ) - datalog.offset;

    if ( log_ins.count < log_ins.limit ) {
      row = log_ins.count;
      log_ins.time[row] = timestamp;
      log_ins.dur[row]  = tmr_ins.dur;

      ushort r, c;
      ushort n = EKF_N, m = EKF_M;
      pthread_mutex_lock(&ekf.mutex);
      for ( r=0; r<n; r++ )  for ( c=0; c<m; c++ )  log_ins.K [ row*n*m +r*m +c ] = mat_get( ekf.K, r+1, c+1 );
      pthread_mutex_unlock(&ekf.mutex);

      log_ins.count++;
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
    fprintf( datalog.param, "\n     %011.6f     ", log_param.time[row] );
    for ( i=0; i < param_count; i++ )  fprintf( datalog.param, "%09.4f   ", log_param.values [ row * param_count + i ] );
  }

  // Input data
  for ( row = 0; row < log_input.count; row++ ) {
    fprintf( datalog.in, "\n     %011.6f     ", log_input.time[row] );
    for ( i=0; i<10; i++ )  fprintf( datalog.in, "%7.4f   ", log_input.data [ row*10 +i ] );
  }

  // Output data
  for ( row = 0; row < log_output.count; row++ ) {
    fprintf( datalog.out, "\n     %011.6f     ", log_output.time[row] );
    for ( i=0; i<10; i++ )  fprintf( datalog.out, "%7.4f   ", log_output.data [ row*10 +i ] );
  }

  // IMU timing thread
  for ( row = 0; row < log_imu.count; row++ ) {

    // IMU timing data
    fprintf( datalog.imu, "\n     %011.6f     %06ld   ", log_imu.time[row], log_imu.dur[row] );

    // IMU A datalogs
    if(IMUA_ENABLED)  {

      // Gyroscope A data
      fprintf( datalog.gyrA, "\n     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%06d   ",   log_gyrA.raw    [ row*3 +i ] );   fprintf( datalog.gyrA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%07.4f   ", log_gyrA.scaled [ row*3 +i ] );   fprintf( datalog.gyrA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.gyrA, "%07.4f   ", log_gyrA.filter [ row*3 +i ] );   fprintf( datalog.gyrA, "     " );

      // Accelerometer A data
      fprintf( datalog.accA, "\n     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%06d   ",   log_accA.raw    [ row*3 +i ] );   fprintf( datalog.accA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%07.4f   ", log_accA.scaled [ row*3 +i ] );   fprintf( datalog.accA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.accA, "%07.4f   ", log_accA.filter [ row*3 +i ] );   fprintf( datalog.accA, "     " );

      // Magnetometer A data
      fprintf( datalog.magA, "\n     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%06d   ",   log_magA.raw    [ row*3 +i ] );   fprintf( datalog.magA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%07.4f   ", log_magA.scaled [ row*3 +i ] );   fprintf( datalog.magA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.magA, "%07.4f   ", log_magA.filter [ row*3 +i ] );   fprintf( datalog.magA, "     " );

      // Complimentary filter A data
      fprintf( datalog.compA, "\n     " );
      fprintf( datalog.compA, "%07.4f   ", log_compA.roll[row]  );
      fprintf( datalog.compA, "%07.4f   ", log_compA.pitch[row] );

      // Attitude/Heading Reference System A data
      fprintf( datalog.ahrsA, "\n     " );
      for ( i=0; i<4; i++ )  fprintf( datalog.ahrsA, "%07.4f   ", log_ahrsA.quat   [ row*4 +i ] );  fprintf( datalog.ahrsA, "     " );
      for ( i=0; i<4; i++ )  fprintf( datalog.ahrsA, "%07.4f   ", log_ahrsA.dquat  [ row*4 +i ] );  fprintf( datalog.ahrsA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.ahrsA, "%07.4f   ", log_ahrsA.eul    [ row*3 +i ] );  fprintf( datalog.ahrsA, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.ahrsA, "%07.4f   ", log_ahrsA.deul   [ row*3 +i ] );  fprintf( datalog.ahrsA, "     " );

    }

    // IMU B datalogs
    if (IMUB_ENABLED)  {

      // Gyroscope B data
      fprintf( datalog.gyrB, "\n     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%06d   ",   log_gyrB.raw    [ row*3 +i ] );   fprintf( datalog.gyrB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%07.4f   ", log_gyrB.scaled [ row*3 +i ] );   fprintf( datalog.gyrB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.gyrB, "%07.4f   ", log_gyrB.filter [ row*3 +i ] );   fprintf( datalog.gyrB, "     " );

      // Accelerometer B data
      fprintf( datalog.accB, "\n     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%06d   ",   log_accB.raw    [ row*3 +i ] );   fprintf( datalog.accB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%07.4f   ", log_accB.scaled [ row*3 +i ] );   fprintf( datalog.accB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.accB, "%07.4f   ", log_accB.filter [ row*3 +i ] );   fprintf( datalog.accB, "     " );

      // Magnetometer B data
      fprintf( datalog.magB, "\n     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%06d   ",   log_magB.raw    [ row*3 +i ] );   fprintf( datalog.magB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%07.4f   ", log_magB.scaled [ row*3 +i ] );   fprintf( datalog.magB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.magB, "%07.4f   ", log_magB.filter [ row*3 +i ] );   fprintf( datalog.magB, "     " );

      // Complimentary filter B data
      fprintf( datalog.compB, "\n     " );
      fprintf( datalog.compB, "%07.4f   ", log_compB.roll[row]  );
      fprintf( datalog.compB, "%07.4f   ", log_compB.pitch[row] );

      // Attitude/Heading Reference System B data
      fprintf( datalog.ahrsB, "\n     " );
      for ( i=0; i<4; i++ )  fprintf( datalog.ahrsB, "%07.4f   ", log_ahrsB.quat  [ row*4 +i ] );  fprintf( datalog.ahrsB, "     " );
      for ( i=0; i<4; i++ )  fprintf( datalog.ahrsB, "%07.4f   ", log_ahrsB.dquat [ row*4 +i ] );  fprintf( datalog.ahrsB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.ahrsB, "%07.4f   ", log_ahrsB.eul   [ row*3 +i ] );  fprintf( datalog.ahrsB, "     " );
      for ( i=0; i<3; i++ )  fprintf( datalog.ahrsB, "%07.4f   ", log_ahrsB.deul  [ row*3 +i ] );  fprintf( datalog.ahrsB, "     " );

    }

    // Rotational state data
    fprintf( datalog.rot, "\n     " );
    for ( i=0; i<3; i++ )  fprintf( datalog.rot, "%07.4f   ", log_rot.att [ row*3 +i ] );   fprintf( datalog.rot, "     " );
    for ( i=0; i<3; i++ )  fprintf( datalog.rot, "%07.4f   ", log_rot.ang [ row*3 +i ] );   fprintf( datalog.rot, "     " );

  }


  /*
  // Stabilization data
  for ( row = 0; row < log_stab.count; row++ )  {
    fprintf( datalog.stab, "\n %011.6f   %06ld      ", log_stab.time[row], log_stab.dur[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfX.r[row]  );
    fprintf( datalog.stab, "%07.4f  ",  log_sfX.xp[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfX.xd[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfX.u[row]  );
    fprintf( datalog.stab, "%07.4f  ",  log_sfX.kp[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfX.kd[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfX.ku[row] );
    fprintf( datalog.stab, "    " );
    fprintf( datalog.stab, "%07.4f  ",  log_sfY.r[row]  );
    fprintf( datalog.stab, "%07.4f  ",  log_sfY.xp[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfY.xd[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfY.u[row]  );
    fprintf( datalog.stab, "%07.4f  ",  log_sfY.kp[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfY.kd[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfY.ku[row] );
    fprintf( datalog.stab, "    " );
    fprintf( datalog.stab, "%07.4f  ",  log_sfZ.r[row]  );
    fprintf( datalog.stab, "%07.4f  ",  log_sfZ.xp[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfZ.xd[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfZ.u[row]  );
    fprintf( datalog.stab, "%07.4f  ",  log_sfZ.kp[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfZ.kd[row] );
    fprintf( datalog.stab, "%07.4f  ",  log_sfZ.ku[row] );
    fprintf( datalog.stab, "    " );
  }
  */
  /*
  // System identification data
  for ( row = 0; row < log_stab.count; row++ )  {
    fprintf( datalog.sysid, "\n %011.6f   %06ld      ", log_stab.time[row], log_stab.dur[row] );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidX.z1[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidX.z2[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidX.p1[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidX.p2[row]  );
    fprintf( datalog.sysid, "    " );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidY.z1[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidY.z2[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidY.p1[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidY.p2[row]  );
    fprintf( datalog.sysid, "    " );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidZ.z1[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidZ.z2[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidZ.p1[row]  );
    fprintf( datalog.sysid, "%07.4f  ",  log_sysidZ.p2[row]  );
    fprintf( datalog.sysid, "    " );
  }
  */
  /*
  // Extended Kalman Filter data
  ushort n = EKF_N, m = EKF_M;
  for ( row = 0; row < log_stab.count; row++ )  {
    fprintf( datalog.ekf, "\n %011.6f   %06ld      ", log_stab.time[row], log_stab.dur[row] );
    for ( i=0; i<n;   i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.x [ row*n   +i ] );   fprintf( datalog.ekf, "    " );
    for ( i=0; i<m;   i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.z [ row*m   +i ] );   fprintf( datalog.ekf, "    " );
    for ( i=0; i<n;   i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.f [ row*n   +i ] );   fprintf( datalog.ekf, "    " );
    for ( i=0; i<m;   i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.h [ row*m   +i ] );   fprintf( datalog.ekf, "    " );
    for ( i=0; i<n*n; i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.F [ row*n*n +i ] );   fprintf( datalog.ekf, "    " );
    //for ( i=0; i<n*n; i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.P [ row*n*n +i ] );   fprintf( datalog.ekf, "    " );
    //for ( i=0; i<n*n; i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.T [ row*n*n +i ] );   fprintf( datalog.ekf, "    " );
    //for ( i=0; i<m*m; i++ )  fprintf( datalog.ekf, "%07.4f  ",  log_ekf.S [ row*m*m +i ] );   fprintf( datalog.ekf, "    " );

  }
  */
  /*
  // Inertial Navigation System data
  for ( row = 0; row < log_ins.count; row++ )  {
    fprintf( datalog.ins, "\n %011.6f   %06ld      ", log_ins.time[row], log_ins.dur[row] );
    for ( i=0; i<n*m; i++ )  fprintf( datalog.ins, "%07.4f  ",  log_ins.K [ row*n*m +i ] );   fprintf( datalog.ins, "    " );
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

  fclose(datalog.imu);
  fclose(datalog.rot);  

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

  //fclose(datalog.stab);
  //fclose(datalog.sysid);

  return;
}



