

#include "imu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "i2c.h"
#include "led.h"
#include "mpu.h"
#include "sys.h"
#include "timer.h"


static void  imu_param   ( imu_struct *imu );
static void  imu_getcal  ( imu_struct *imu );
static void  imu_9DOF    ( imu_struct *imu );
static void  imu_6DOF    ( imu_struct *imu );


/**
 *  imu_init
 *  Initializes an MPU sensor.
 */
void imu_init ( void )  {
  if(DEBUG)  printf( "Initializing IMU \n" );

  // Start initialization
  led_blink( LED_IMU, 200, 200 );

  // Check timing conditions
  if ( HZ_IMU_FAST % HZ_IMU_SLOW != 0 )
    printf( "  *** WARNING ***  Slow loop must divide evenly into fast loop. \n" );
  uint loops = HZ_IMU_FAST / HZ_IMU_SLOW;

  // Setup IMUA
  if (IMUA_ENABLED)  {

    // I2C parameters
    imuA.id   = 'A';
    imuA.bus  = 1;
    imuA.addr = 0x68;

    // Assign pointers
    imuA.gyr  = &gyrA;
    imuA.acc  = &accA;
    imuA.mag  = &magA;
    imuA.comp = &compA;
    imuA.ahrs = &ahrsA;

    // Loop counter values
    imuA.loops  = loops;
    imuA.getmag = false;

    // Setup functions
    i2c_init( &(imuA.fd), imuA.bus, imuA.addr );
    imu_param(&imuA);
    imu_getcal(&imuA);

  }

  // Setup IMUB
  if (IMUB_ENABLED)  {

    // I2C parameters
    imuB.id   = 'B';
    imuB.bus  = 2;
    imuB.addr = 0x68;

    // Assign pointers
    imuB.gyr  = &gyrB;
    imuB.acc  = &accB;
    imuB.mag  = &magB;
    imuB.comp = &compB;
    imuB.ahrs = &ahrsB;

    // Loop counter values
    imuB.loops  = loops;
    imuB.getmag = false;

    // Setup functions
    i2c_init( &(imuB.fd), imuB.bus, imuB.addr );
    imu_param(&imuB);
    imu_getcal(&imuB);

  }

  // IMU warmup period
  usleep(300000);
  led_on(LED_IMU);

  return;
}


/**
 *  imu_exit
 *  Terminate an MPU sensor.
 */
void imu_exit ( void )  {
  if(DEBUG)  printf("Close IMU \n");
  if(IMUA_ENABLED)  i2c_exit( &(imuA.fd) );
  if(IMUB_ENABLED)  i2c_exit( &(imuB.fd) );
  led_off(LED_IMU);
  return;
}


/**
 *  imu_param
 *  Assign parameters to an MPU sensor.
 */
void imu_param ( imu_struct *imu )  {
  if(DEBUG) {  printf("  Assign IMU%c parameters ", imu->id );  fflush(stdout);  }

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_init( imu->fd, NULL ) )
    printf( "Error (imu_param): 'mpu_init' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sensors( imu->fd, INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS ) )
    printf( "Error (imu_param): 'mpu_set_sensors' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_sample_rate( imu->fd, HZ_IMU_FAST ) )
    printf( "Error (imu_param): 'mpu_set_sample_rate' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_compass_sample_rate( imu->fd, HZ_IMU_SLOW ) )
    printf( "Error (imu_param): 'mpu_set_compass_sample_rate' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_gyro_fsr( imu->fd, GYR_FSR ) )
    printf( "Error (imu_param): 'mpu_set_gyro_fsr' failed. \n" );

  if(DEBUG) {  printf(".");  fflush(stdout);  }
  if( mpu_set_accel_fsr( imu->fd, ACC_FSR ) )
    printf( "Error (imu_param): 'mpu_set_accel_fsr' failed. \n" );

  if(DEBUG)  printf(" complete \n");

  return;
}


/**
 *  imu_getcal
 *  Gets the calibration parameters for the MPU sensor.
 */
void imu_getcal ( imu_struct *imu )  {
  if(DEBUG)  printf( "  IMU%c calibration values: \n", imu->id );

  // Local variables
  int i;
  FILE *f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );

  // Set accelerometer bias
  sprintf( path, "../Param/board/bias/acc%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc%c bias' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->acc->bias[i] = atoi(buff);
  }
  fclose(f);

  // Set accelerometer range
  sprintf( path, "../Param/board/range/acc%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc%c range' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->acc->range[i] = atoi(buff);
  }
  fclose(f);

  // Set accelerometer lpf
  sprintf( path, "../Param/board/lpf/acc" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'acc lpf' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->acc->lpf[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer bias
  sprintf( path, "../Param/board/bias/mag%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'mag%c bias' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->mag->bias[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer range
  sprintf( path, "../Param/board/range/mag%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'mag%c range' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->mag->range[i] = atoi(buff);
  }
  fclose(f);

  // Set magnetometer lpf
  sprintf( path, "../Param/board/lpf/mag" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'mag lpf' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->mag->lpf[i] = atoi(buff);
  }
  fclose(f);

  // Set rate gyro bias
  sprintf( path, "../Param/board/bias/gyr%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'gyr%c bias' not found. \n", imu->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->gyr->bias[i] = atoi(buff);
  }
  fclose(f);

  // Set rate gyro lpf
  sprintf( path, "../Param/board/lpf/gyr" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'gyr lpf' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    imu->gyr->lpf[i] = atoi(buff);
  }
  fclose(f);

  // Set comp filter bias
  sprintf( path, "../Param/board/bias/comp%c", imu->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'comp%c bias' not found. \n", imu->id );
  for ( i=0; i<2; i++ ) {
    fgets( buff, 32, f );
    imu->comp->bias[i] = atoi(buff) / 10000.0;
  }
  fclose(f);

  // Set comp filter gain
  sprintf( path, "../Param/board/gain/comp" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'comp gain' not found. \n" );
  fgets( buff, 32, f );
  imu->comp->gain = atoi(buff);
  fclose(f);

  // Set AHRS gain
  sprintf( path, "../Param/board/gain/ahrs" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (imu_getcal): File for 'ahrs gain' not found. \n" );
  fgets( buff, 32, f );
  imu->ahrs->gain = atoi(buff);
  fclose(f);

  // Set unit quaternion value
  imu->ahrs->quat[0]  = 1.0;

  // Display calibration values
  if(DEBUG) {

    printf("   \
    abias%c arange%c alpf%c   \
    mbias%c mrange%c mlpf%c   \
    gbias%c glpf%c  \n", 
    imu->id, imu->id, imu->id, imu->id, imu->id, imu->id, imu->id, imu->id );

    for ( i=0; i<3; i++ ) {
      printf("     ");
      printf( "%4d    ",   imu->acc->bias[i]  );
      printf( "%4d     ",  imu->acc->range[i] );
      printf( "%4.2f   ",  imu->acc->lpf[i]   );
      printf( "%4d     ",  imu->mag->bias[i]  );
      printf( "%4d     ",  imu->mag->range[i] );
      printf( "%4.2f   ",  imu->mag->lpf[i]   );
      printf( "%4d     ",  imu->gyr->bias[i]  );
      printf( "%4.2f   ",  imu->gyr->lpf[i]   );
    } 
  }

  //printf( "%4.2f  \n", imu->comp->bias[0]  );
  //printf( "%4.2f  \n", imu->comp->bias[1]  );
  //printf( "%4.2f  \n", imu->comp->gain  );
  //printf( "%4.2f  \n", imu->ahrs->gain  );

  return;
}


/**
 *  imu_update
 *  Update system with new IMU sensor data.
 */
void imu_update ( imu_struct *imu )  {

  // Loop counter
  ushort i;

  // Array index
  ushort x=0, y=1, z=2;

  // Local IMU arrays
  short  Gr[3], Ar[3], Mr[3];    // Raw
  double Gs[3], As[3], Ms[3];    // Scaled
  double Gp[3], Ap[3], Mp[3];    // Previous
  double Gg[3], Ag[3], Mg[3];    // LPF Gain
  double Gf[3], Af[3], Mf[3];    // Filtered

  // Increment counter
  imu->getmag = false;
  if ( imu->count == 0 ) {
    imu->getmag = true;
    imu->count = imu->loops;
  }
  imu->count--;

  // Raw IMU data
  if( mpu_get_gyro_reg( imu->fd, Gr, NULL ) )
    printf( "Error (imu_data): 'mpu_get_gyro_reg' failed. \n" );
  if( mpu_get_accel_reg( imu->fd, Ar, NULL ) )
    printf( "Error (imu_data): 'mpu_get_accel_reg' failed. \n" );
  if(imu->getmag){
    if( mpu_get_compass_reg( imu->fd, Mr, NULL ) )
    printf( "Error (imu_data): 'mpu_get_compass_reg' failed. \n" );
  } 

  // Scaled gyroscope readings
  Gs[x] = - ( Gr[x] - imu->gyr->bias[x] ) * GYR_SCALE;
  Gs[y] =   ( Gr[y] - imu->gyr->bias[y] ) * GYR_SCALE;
  Gs[z] = - ( Gr[z] - imu->gyr->bias[z] ) * GYR_SCALE;

  // Scaled accelerometer readings
  As[x] = - ( Ar[x] - imu->acc->bias[x] ) / (double) (imu->acc->range[x]);
  As[y] =   ( Ar[y] - imu->acc->bias[y] ) / (double) (imu->acc->range[y]);
  As[z] = - ( Ar[z] - imu->acc->bias[z] ) / (double) (imu->acc->range[z]);

  // Scaled magnetometer readings
  if(imu->getmag) {
  Ms[x] = - ( Mr[y] - imu->mag->bias[y] ) / (double) (imu->mag->range[y]);
  Ms[y] =   ( Mr[x] - imu->mag->bias[x] ) / (double) (imu->mag->range[x]);
  Ms[z] =   ( Mr[z] - imu->mag->bias[z] ) / (double) (imu->mag->range[z]);
  }

  // Get gyroscope values
  pthread_mutex_lock( &(imu->gyr->mutex) );
  for ( i=0; i<3; i++ )  {  Gp[i] = imu->gyr->filter[i];  Gg[i] = imu->gyr->lpf[i];  }
  pthread_mutex_unlock( &(imu->gyr->mutex) );

  // Get accerometer values
  pthread_mutex_lock( &(imu->acc->mutex) );
  for ( i=0; i<3; i++ )  {  Ap[i] = imu->acc->filter[i];  Ag[i] = imu->acc->lpf[i];  }
  pthread_mutex_unlock( &(imu->acc->mutex) );

  // Get magnetometer values
  if(imu->getmag) {
  pthread_mutex_lock( &(imu->mag->mutex) );
  for ( i=0; i<3; i++ )  {  Mp[i] = imu->mag->filter[i];  Mg[i] = imu->mag->lpf[i];  }
  pthread_mutex_unlock( &(imu->mag->mutex) );
  }

  // Low pass filter
  for ( i=0; i<3; i++ )  {
    Gf[i] = Gp[i] + Gg[i] * ( Gs[i] - Gp[i] );
    Af[i] = Ap[i] + Ag[i] * ( As[i] - Ap[i] );
    if (imu->getmag)  {
    Mf[i] = Mp[i] + Mg[i] * ( Ms[i] - Mp[i] );
    }
  }

  // Complimentary filter variables
  double dt, gain, R, P;

  // Get complimentary filter values
  dt = 1.0 / HZ_IMU_FAST;
  pthread_mutex_lock( &(imu->mutex) );
  gain = imu->comp;
  R = imu->roll;
  P = imu->pitch;
  pthread_mutex_unlock( &(imu->mutex) );

  // Complimentary filter attitude angles
  R = gain * ( R + Gf[0] * dt ) + ( 1.0 - gain ) * atan2( -Af[1], -Af[2] );
  P = gain * ( P + Gf[1] * dt ) + ( 1.0 - gain ) * atan2(  Af[0], -Af[2] );

  // AHRS data fusion
  imu_9DOF( imu );

  // Push gyroscope values to data structure
  pthread_mutex_lock( &(imu->gyr->mutex) );
  for ( i=0; i<3; i++ )  {
    imu->gyr->raw[i]    = Gr[i];
    imu->gyr->scaled[i] = Gs[i];
    imu->gyr->filter[i] = Gf[i];
  }
  pthread_mutex_unlock( &(imu->gyr->mutex) );

  // Push accerometer values to data structure
  pthread_mutex_lock( &(imu->acc->mutex) );
  for ( i=0; i<3; i++ )  {
    imu->acc->raw[i]    = Ar[i];
    imu->acc->scaled[i] = As[i];
    imu->acc->filter[i] = Af[i];
  }
  pthread_mutex_unlock( &(imu->acc->mutex) );

  // Push magnetometer values to data structure
  if(imu->getmag) {
  pthread_mutex_lock( &(imu->mag->mutex) );
  for ( i=0; i<3; i++ )  {
    imu->mag->raw[i]    = Mr[i];
    imu->mag->scaled[i] = Ms[i];
    imu->mag->filter[i] = Mf[i];
  }
  pthread_mutex_unlock( &(imu->mag->mutex) );
  }

  // Push complimentary filter values to data structure
  pthread_mutex_lock( &(imu->mutex) );
  imu->roll  = R;
  imu->pitch = P;
  pthread_mutex_unlock( &(imu->mutex) );

  return;
}


/**
 *  imu_9DOF
 *  Implements 9DOF data fusion (gyr/acc/mag sensors) algorithm.
 */
void imu_9DOF ( imu_struct *imu )  {

  // Local variables
  ushort i;
  double dt, recipNorm;
  double q[4], dq[4], g[3], a[3], m[3], s[4], e[3], de[3], off[3];
  double hx, hy;
  double _2qWmx, _2qWmy, _2qWmz, _2qXmx, _2bx, _2bz, _4bx, _4bz;
  double _2qW, _2qX, _2qY, _2qZ, _2qWqY, _2qYqZ;
  double  qWqW, qWqX, qWqY, qWqZ, qXqX, qXqY, qXqZ, qYqY, qYqZ, qZqZ;

  // Local index
  ushort x=0, y=1, z=2;
  ushort W=0, X=1, Y=2, Z=3;

  // Time step
  dt = 1.0 / HZ_IMU_FAST;

  // Get gyr IMU data
  pthread_mutex_lock(&(imu->gyr->mutex));
  for ( i=0; i<3; i++ )  g[i] = imu->gyr->filter[i];
  pthread_mutex_unlock(&(imu->gyr->mutex));

  // Get acc IMU data
  pthread_mutex_lock(&(imu->acc->mutex));
  for ( i=0; i<3; i++ )  a[i] = -imu->acc->filter[i];
  pthread_mutex_unlock(&(imu->acc->mutex));

  // Get mag IMU data
  pthread_mutex_lock(&(imu->mag->mutex));
  for ( i=0; i<3; i++ )  m[i] = imu->mag->filter[i];
  pthread_mutex_unlock(&(imu->mag->mutex));

  // Get AHRS data
  pthread_mutex_lock(&(imu->ahrs->mutex));
  for ( i=0; i<4; i++ )  q[i]   = imu->ahrs->quat[i];
  for ( i=0; i<3; i++ )  off[i] = imu->offset[i];
  pthread_mutex_unlock(&(imu->ahrs->mutex));

  // Use IMU algorithm if magnetometer measurement invalid
  if ( (m[x] == 0.0f) && (m[y] == 0.0f) && (m[z] == 0.0f) )  {
    imu_6DOF ( imu );
    return;
  }

  // Rate of change of quaternion from gyroscope
  dq[W] = 0.5f * ( -q[X] * g[x]  - q[Y] * g[y]  - q[Z] * g[z] );
  dq[X] = 0.5f * (  q[W] * g[x]  + q[Y] * g[z]  - q[Z] * g[y] );
  dq[Y] = 0.5f * (  q[W] * g[y]  - q[X] * g[z]  + q[Z] * g[x] );
  dq[Z] = 0.5f * (  q[W] * g[z]  + q[X] * g[y]  - q[Y] * g[x] );

  // Compute feedback only if accelerometer measurement is valid
  if( ! ( (a[x] == 0.0f) && (a[y] == 0.0f) && (a[z] == 0.0f) ) )  {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / sqrt( a[x] * a[x] + a[y] * a[y] + a[z] * a[z] );
    for ( i=0; i<3; i++ )  a[i] *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = 1.0 / sqrt( m[x] * m[x] + m[y] * m[y] + m[z] * m[z] );
    for ( i=0; i<3; i++ )  m[i] *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2qWmx  = 2.0f * q[W] * m[x];
    _2qWmy  = 2.0f * q[W] * m[y];
    _2qWmz  = 2.0f * q[W] * m[z];
    _2qXmx  = 2.0f * q[X] * m[x];
    _2qW    = 2.0f * q[W];
    _2qX    = 2.0f * q[X];
    _2qY    = 2.0f * q[Y];
    _2qZ    = 2.0f * q[Z];
    _2qWqY  = 2.0f * q[W] * q[Y];
    _2qYqZ  = 2.0f * q[Y] * q[Z];
    qWqW    = q[W] * q[W];
    qWqX    = q[W] * q[X];
    qWqY    = q[W] * q[Y];
    qWqZ    = q[W] * q[Z];
    qXqX    = q[X] * q[X];
    qXqY    = q[X] * q[Y];
    qXqZ    = q[X] * q[Z];
    qYqY    = q[Y] * q[Y];
    qYqZ    = q[Y] * q[Z];
    qZqZ    = q[Z] * q[Z];

    // Reference direction of Earth's magnetic field
    hx = m[x] * qWqW - _2qWmy * q[Z] + _2qWmz * q[Y] + m[x] * qXqX + _2qX * m[y] * q[Y] + _2qX * m[z] * q[Z] - m[x] * qYqY - m[x] * qZqZ;
    hy = _2qWmx * q[Z] + m[y] * qWqW - _2qWmz * q[X] + _2qXmx * q[Y] - m[y] * qXqX + m[y] * qYqY + _2qY * m[z] * q[Z] - m[y] * qZqZ;
    _2bx = sqrt( hx * hx + hy * hy );
    _2bz = -_2qWmx * q[Y] + _2qWmy * q[X] + m[z] * qWqW + _2qXmx * q[Z] - m[z] * qXqX + _2qY * m[y] * q[Z] - m[z] * qYqY + m[z] * qZqZ;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient descent algorithm corrective step
    s[W] = - _2qY * ( 2.0f * qXqZ - _2qWqY - a[x] )
         + _2qX * ( 2.0f * qWqX + _2qYqZ - a[y] )
         - _2bz * q[Y] * ( _2bx * ( 0.5f - qYqY - qZqZ ) + _2bz * ( qXqZ - qWqY ) - m[x] )
         + ( -_2bx * q[Z] + _2bz * q[X] ) * ( _2bx * ( qXqY - qWqZ ) + _2bz * ( qWqX + qYqZ ) - m[y] )
         + _2bx * q[Y] * ( _2bx * ( qWqY + qXqZ ) + _2bz * ( 0.5f - qXqX - qYqY ) - m[z] );
    s[X] = _2qZ * ( 2.0f * qXqZ - _2qWqY - a[x] )
         + _2qW * ( 2.0f * qWqX + _2qYqZ - a[y] )
         - 4.0f * q[X] * ( 1.0f - 2.0f * qXqX - 2.0f * qYqY - a[z] )
         + _2bz * q[Z] * ( _2bx * ( 0.5f - qYqY - qZqZ ) + _2bz * ( qXqZ - qWqY ) - m[x] )
         + ( _2bx * q[Y] + _2bz * q[W] ) * ( _2bx * ( qXqY - qWqZ ) + _2bz * ( qWqX + qYqZ ) - m[y] )
         + ( _2bx * q[Z] - _4bz * q[X] ) * ( _2bx * ( qWqY + qXqZ ) + _2bz * ( 0.5f - qXqX - qYqY ) - m[z] );
    s[Y] = - _2qW * ( 2.0f * qXqZ - _2qWqY - a[x] )
         + _2qZ * ( 2.0f * qWqX + _2qYqZ - a[y] )
         - 4.0f * q[Y] * ( 1.0f - 2.0f * qXqX - 2.0f * qYqY - a[z] )
         + ( -_4bx * q[Y] - _2bz * q[W] ) * ( _2bx * ( 0.5f - qYqY - qZqZ ) + _2bz * ( qXqZ - qWqY ) - m[x] )
         + ( _2bx * q[X] + _2bz * q[Z] ) * ( _2bx * ( qXqY - qWqZ ) + _2bz * ( qWqX + qYqZ ) - m[y] )
         + ( _2bx * q[W] - _4bz * q[Y] ) * ( _2bx * ( qWqY + qXqZ ) + _2bz * ( 0.5f - qXqX - qYqY ) - m[z] );
    s[Z] = _2qX * ( 2.0f * qXqZ - _2qWqY - a[x] )
         + _2qY * ( 2.0f * qWqX + _2qYqZ - a[y] )
         + ( -_4bx * q[Z] + _2bz * q[X] ) * ( _2bx * ( 0.5f - qYqY - qZqZ ) + _2bz * ( qXqZ - qWqY ) - m[x] )
         + ( -_2bx * q[W] + _2bz * q[Y] ) * ( _2bx * ( qXqY - qWqZ ) + _2bz * ( qWqX + qYqZ ) - m[y] )
         + _2bx * q[X] * ( _2bx * ( qWqY + qXqZ ) + _2bz * ( 0.5f - qXqX - qYqY ) - m[z] );

    // Normalize step magnitude
    recipNorm = 1.0 / sqrt( s[W] * s[W] + s[X] * s[X] + s[Y] * s[Y] + s[Z] * s[Z] );
    for ( i=0; i<4; i++ )  s[i] *= recipNorm;

    // Apply feedback step
    for ( i=0; i<4; i++ )  dq[i] -= IMU_GAIN * s[i];

  }

  // Integrate rate of change of quaternion to yield quaternion
  for ( i=0; i<4; i++ )  q[i] += dq[i] * dt;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt( q[W] * q[W] + q[X] * q[X] + q[Y] * q[Y] + q[Z] * q[Z] );
  for ( i=0; i<4; i++ )  q[i] *= recipNorm;

  // Calculate Euler angles
  e[x] = atan2 ( ( 2.0f * ( q[W]*q[X] + q[Y]*q[Z] ) ), ( 1.0f - 2.0f * ( q[X]*q[X] + q[Y]*q[Y] ) ) ) - off[x];
  e[y] = asin  (   2.0f * ( q[W]*q[Y] - q[X]*q[Z] ) )                                                - off[y];
  e[z] = atan2 ( ( 2.0f * ( q[W]*q[Z] + q[X]*q[Y] ) ), ( 1.0f - 2.0f * ( q[Y]*q[Y] + q[Z]*q[Z] ) ) ) - off[z];

  // Calculate Euler derivatives
  de[x] = - q[X] * dq[W] + q[W] * dq[X] + q[Z] * dq[Y] - q[Y] * dq[Z];
  de[y] = - q[Y] * dq[W] - q[Z] * dq[X] + q[W] * dq[Y] + q[X] * dq[Z];
  de[z] = - q[Z] * dq[W] + q[Y] * dq[X] - q[X] * dq[Y] + q[W] * dq[Z];
  for ( i=0; i<3; i++ )  de[i] *= 2.0f;

  // Push values to AHRS data structure
  pthread_mutex_lock(&(imu->ahrs->mutex));
  for ( i=0; i<4; i++ )  {  imu->ahrs->quat[i] = q[i];  imu->ahrs->dquat[i] = dq[i];  }
  for ( i=0; i<3; i++ )  {  imu->ahrs->eul[i]  = e[i];  imu->ahrs->deul[i]  = de[i];  }
  pthread_mutex_unlock(&(imu->ahrs->mutex));

  return;
}


/**
 *  imu_6DOF
 *  Implements 6DOF data fusion (gyr/acc sensors only) algorithm.
 */
void imu_6DOF ( imu_struct *imu )  {

  // Local variables
  ushort i;
  double dt, recipNorm;
  double q[4], dq[4], g[3], a[3], s[4], e[3], de[3], off[3];
  double _2qW, _2qX, _2qY, _2qZ, _4qW, _4qX, _4qY ,_8qX, _8qY;
  double qWqW, qXqX, qYqY, qZqZ;

  // Local index
  ushort x=0, y=1, z=2;
  ushort W=0, X=1, Y=2, Z=3;

  // Time step
  dt = 1.0 / HZ_IMU_FAST;

  // Get gyr IMU data
  pthread_mutex_lock(&(imu->gyr->mutex));
  for ( i=0; i<3; i++ )  g[i] = imu->gyr->filter[i];
  pthread_mutex_unlock(&(imu->gyr->mutex));

  // Get acc IMU data
  pthread_mutex_lock(&(imu->acc->mutex));
  for ( i=0; i<3; i++ )  a[i] = -imu->acc->filter[i];
  pthread_mutex_unlock(&(imu->acc->mutex));

  // Get AHRS data
  pthread_mutex_lock(&(imu->ahrs->mutex));
  for ( i=0; i<4; i++ )  q[i]   = imu->ahrs->quat[i];
  for ( i=0; i<3; i++ )  off[i] = imu->offset[i];
  pthread_mutex_unlock(&(imu->ahrs->mutex));

  // Rate of change of quaternion from gyroscope
  dq[W] = 0.5f * ( -q[X] * g[x]  - q[Y] * g[y]  - q[Z] * g[z] );
  dq[X] = 0.5f * (  q[W] * g[x]  + q[Y] * g[z]  - q[Z] * g[y] );
  dq[Y] = 0.5f * (  q[W] * g[y]  - q[X] * g[z]  + q[Z] * g[x] );
  dq[Z] = 0.5f * (  q[W] * g[z]  + q[X] * g[y]  - q[Y] * g[x] );

  // Compute feedback only if accelerometer measurement is valid
  if( ! ( (a[x] == 0.0f) && (a[y] == 0.0f) && (a[z] == 0.0f) ) ) {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / sqrt( a[x] * a[x] + a[y] * a[y] + a[z] * a[z] );
    for ( i=0; i<3; i++ )  a[i] *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2qW = 2.0f * q[W];
    _2qX = 2.0f * q[X];
    _2qY = 2.0f * q[Y];
    _2qZ = 2.0f * q[Z];
    _4qW = 4.0f * q[W];
    _4qX = 4.0f * q[X];
    _4qY = 4.0f * q[Y];
    _8qX = 8.0f * q[X];
    _8qY = 8.0f * q[Y];
    qWqW = q[W] * q[W];
    qXqX = q[X] * q[X];
    qYqY = q[Y] * q[Y];
    qZqZ = q[Z] * q[Z];

    // Gradient decent algorithm corrective step
    s[W] = _4qW * qYqY + _2qY * a[x] + _4qW * qXqX - _2qX * a[y];
    s[X] = _4qX * qZqZ - _2qZ * a[x] + 4.0f * qWqW * q[X] - _2qW * a[y] - _4qX + _8qX * qXqX + _8qX * qYqY + _4qX * a[z];
    s[Y] = 4.0f * qWqW * q[Y] + _2qW * a[x] + _4qY * qZqZ - _2qZ * a[y] - _4qY + _8qY * qXqX + _8qY * qYqY + _4qY * a[z];
    s[Z] = 4.0f * qXqX * q[Z] - _2qX * a[x] + 4.0f * qYqY * q[Z] - _2qY * a[y];

    // Normalise step magnitude
    recipNorm = 1.0 / sqrt( s[W] * s[W] + s[X] * s[X] + s[Y] * s[Y] + s[Z] * s[Z] );
    for ( i=0; i<4; i++ )  s[i] *= recipNorm;

    // Apply feedback step
    for ( i=0; i<4; i++ )  dq[i] -= IMU_GAIN * s[i];

  }

  // Integrate rate of change of quaternion to yield quaternion
  for ( i=0; i<4; i++ )  q[i] += dq[i] * dt;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt( q[W] * q[W] + q[X] * q[X] + q[Y] * q[Y] + q[Z] * q[Z] );
  for ( i=0; i<4; i++ )  q[i] *= recipNorm;

  // Calculate Euler angles
  e[x] = atan2 ( ( 2.0f * ( q[W]*q[X] + q[Y]*q[Z] ) ), ( 1.0f - 2.0f * ( q[X]*q[X] + q[Y]*q[Y] ) ) ) - off[x];
  e[y] = asin  (   2.0f * ( q[W]*q[Y] - q[X]*q[Z] ) )                                                - off[y];
  e[z] = atan2 ( ( 2.0f * ( q[W]*q[Z] + q[X]*q[Y] ) ), ( 1.0f - 2.0f * ( q[Y]*q[Y] + q[Z]*q[Z] ) ) ) - off[z];

  // Calculate Euler derivatives
  de[x] = - q[X] * dq[W] + q[W] * dq[X] + q[Z] * dq[Y] - q[Y] * dq[Z];
  de[y] = - q[Y] * dq[W] - q[Z] * dq[X] + q[W] * dq[Y] + q[X] * dq[Z];
  de[z] = - q[Z] * dq[W] + q[Y] * dq[X] - q[X] * dq[Y] + q[W] * dq[Z];
  for ( i=0; i<3; i++ )  de[i] *= 2.0f;

  // Push values to AHRS data structure
  pthread_mutex_lock(&(imu->ahrs->mutex));
  for ( i=0; i<4; i++ )  {  imu->ahrs->quat[i] = q[i];  imu->ahrs->dquat[i] = dq[i];  }
  for ( i=0; i<3; i++ )  {  imu->ahrs->eul[i]  = e[i];  imu->ahrs->deul[i]  = de[i];  }
  pthread_mutex_unlock(&(imu->ahrs->mutex));

  return;
}


/**
 *  imu_state
 *  Calculate the rotational state from various signals and filters
 */
void imu_state ( void )  {

  // Local variables
  ushort i;
  double att[3], ang[3], off[3];

  // Zero out states
  for ( i=0; i<3; i++ )  {  att[i] = 0.0;  ang[i] = 0.0;  }

  // IMUA states
  if (IMUA_ENABLED)  {

    // Get offset values
    pthread_mutex_lock(&ahrsA.mutex);
    for ( i=0; i<3; i++ )  off[i] = imuA.offset[i];
    pthread_mutex_unlock(&ahrsA.mutex);

    // Comp filter values (no yaw value)
    pthread_mutex_lock(&imuA.mutex);
    att[0] += imuA.roll  - off[0];
    att[1] += imuA.pitch - off[1];
    pthread_mutex_unlock(&imuA.mutex);

    // Loop through states
    for ( i=0; i<3; i++ )  {

      // LPF gyro values
      pthread_mutex_lock(&gyrA.mutex);
      ang[i] += gyrA.filter[i];
      pthread_mutex_unlock(&gyrA.mutex);

      // AHRS values
      pthread_mutex_lock(&ahrsA.mutex);
      att[i] += ahrsA.eul[i];
      //ang[i] += ahrsA.deul[i];
      pthread_mutex_unlock(&ahrsA.mutex);

    }
  }

  // IMUB states
  if (IMUB_ENABLED)  {

    // Get offset values
    pthread_mutex_lock(&ahrsB.mutex);
    for ( i=0; i<3; i++ )  off[i] = imuB.offset[i];
    pthread_mutex_unlock(&ahrsB.mutex);

    // Comp filter values (no yaw value)
    pthread_mutex_lock(&imuB.mutex);
    att[0] += imuB.roll  - off[0];
    att[1] += imuB.pitch - off[1];
    pthread_mutex_unlock(&imuB.mutex);

    // Loop through states
    for ( i=0; i<3; i++ )  {

      // LPF gyro values
      pthread_mutex_lock(&gyrB.mutex);
      ang[i] += gyrB.filter[i];
      pthread_mutex_unlock(&gyrB.mutex);

      // AHRS values
      pthread_mutex_lock(&ahrsB.mutex);
      att[i] += ahrsB.eul[i];
      //ang[i] += ahrsB.deul[i];
      pthread_mutex_unlock(&ahrsB.mutex);

    }
  }

  // Average all the data sources
  if ( IMUA_ENABLED && IMUB_ENABLED )  {  for ( i=0; i<3; i++ )  {  att[i] /= 4.0;  ang[i] /= 2.0;  }  }
  else                                 {  for ( i=0; i<3; i++ )  {  att[i] /= 2.0;  ang[i] /= 1.0;  }  }

  // Correction b/c comp filter has no yaw value  
  att[2] *= 2.0; 

  // Push to data structure
  pthread_mutex_lock(&rot.mutex);
  for ( i=0; i<3; i++ ) rot.att[i] = att[i];
  for ( i=0; i<3; i++ ) rot.ang[i] = ang[i];
  pthread_mutex_unlock(&rot.mutex);

  return;
}



