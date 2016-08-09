

#include "ahrs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"
#include "timer.h"


static void  ahrs_marg  ( ahrs_struct *ahrs, imu_struct *imu );
static void  ahrs_imu   ( ahrs_struct *ahrs, imu_struct *imu );


/**
 *  ahrs_init
 *  Initializes the attitude and heading reference algorithms.
 */
void ahrs_init ( void )  {
  if(DEBUG)  printf( "Initializing AHRS \n" );
  if (IMUA_ENABLED) {  ahrsA.id = 'A';  ahrs_setup(&ahrsA);  }
  if (IMUB_ENABLED) {  ahrsB.id = 'B';  ahrs_setup(&ahrsB);  }
  return;
}


/**
 *  ahrs_setup
 *  Sets up the AHRS data structure.
 */
void ahrs_setup ( ahrs_struct *ahrs )  {

  // Local variables
  ushort i;

  // Initialize quaternion
  ahrs->quat[0]  = 1.0;
  ahrs->quat[1]  = 0.0;
  ahrs->quat[2]  = 0.0;
  ahrs->quat[3]  = 0.0;

  // Initialize quaternion derivative
  ahrs->dquat[0] = 0.0;
  ahrs->dquat[1] = 0.0;
  ahrs->dquat[2] = 0.0;
  ahrs->dquat[3] = 0.0;

  // Initialize Euler angles
  ahrs->eul[0] = 0.0;
  ahrs->eul[1] = 0.0;
  ahrs->eul[2] = 0.0;

  // Initialize Euler derivatives
  ahrs->deul[0] = 0.0;
  ahrs->deul[1] = 0.0;
  ahrs->deul[2] = 0.0;

  // Time step increment
  ahrs->dt = 1.0 / HZ_IMU_FAST;

  // Assign AHRS offset
  FILE *f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );
  sprintf( path, "../Param/board/bias/ahrs%c", ahrs->id );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (ahr_setup): File for 'ahrs%c bias' not found. \n", ahrs->id );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    ahrs->orient[i] = atoi(buff) / 10000.0;
  }
  fclose(f);

  // Display AHRS offset
  if (DEBUG)  {
    printf("  AHRS%c offset:", ahrs->id );
    for ( i=0; i<3; i++ )  printf(" %6.3f", ahrs->orient[i] );
    printf("\n");
  }

  return;
}


/**
 *  ahrs_exit
 *  Terminate the AHR algorithms.
 */
void ahrs_exit ( void )  {
  if(DEBUG)  printf("Close AHRS \n");
  // Insert code if needed...
  return;
}


/**
 *  ahrs_update
 *  Implement the Attitude Heading Reference System algorithm
 */
void ahrs_update ( ahrs_struct *ahrs, imu_struct *imu )  {
  ahrs_marg( ahrs, imu );
  return;
}


/**
 *  ahrs_marg
 *  Implement 9DOF data fusion (magnetic angular rate gravity) algorithm.
 */
void ahrs_marg ( ahrs_struct *ahrs, imu_struct *imu )  {

  // Local variables
  ushort i;
  double dt, recipNorm;
  double q[4], dq[4], g[3], a[3], m[3], s[4], e[3];
  double hx, hy;
  double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
  double _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
  double  q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Local index
  ushort x=0, y=1, z=2;
  ushort W=0, X=1, Y=2, Z=3;

  // Get values from AHRS data structure
  pthread_mutex_lock(&ahrs->mutex);
  dt = ahrs->dt;
  for ( i=0; i<4; i++ )  q[i] = ahrs->quat[i];
  pthread_mutex_unlock(&ahrs->mutex);

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

  // Use IMU algorithm if magnetometer measurement invalid
  if ( (m[x] == 0.0f) && (m[y] == 0.0f) && (m[z] == 0.0f) )  {
    ahrs_imu ( ahrs, imu );
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
    _2q0mx  = 2.0f * q[W] * m[x];
    _2q0my  = 2.0f * q[W] * m[y];
    _2q0mz  = 2.0f * q[W] * m[z];
    _2q1mx  = 2.0f * q[X] * m[x];
    _2q0    = 2.0f * q[W];
    _2q1    = 2.0f * q[X];
    _2q2    = 2.0f * q[Y];
    _2q3    = 2.0f * q[Z];
    _2q0q2  = 2.0f * q[W] * q[Y];
    _2q2q3  = 2.0f * q[Y] * q[Z];
    q0q0    = q[W] * q[W];
    q0q1    = q[W] * q[X];
    q0q2    = q[W] * q[Y];
    q0q3    = q[W] * q[Z];
    q1q1    = q[X] * q[X];
    q1q2    = q[X] * q[Y];
    q1q3    = q[X] * q[Z];
    q2q2    = q[Y] * q[Y];
    q2q3    = q[Y] * q[Z];
    q3q3    = q[Z] * q[Z];

    // Reference direction of Earth's magnetic field
    hx = m[x] * q0q0 - _2q0my * q[Z] + _2q0mz * q[Y] + m[x] * q1q1 + _2q1 * m[y] * q[Y] + _2q1 * m[z] * q[Z] - m[x] * q2q2 - m[x] * q3q3;
    hy = _2q0mx * q[Z] + m[y] * q0q0 - _2q0mz * q[X] + _2q1mx * q[Y] - m[y] * q1q1 + m[y] * q2q2 + _2q2 * m[z] * q[Z] - m[y] * q3q3;
    _2bx = sqrt( hx * hx + hy * hy );
    _2bz = -_2q0mx * q[Y] + _2q0my * q[X] + m[z] * q0q0 + _2q1mx * q[Z] - m[z] * q1q1 + _2q2 * m[y] * q[Z] - m[z] * q2q2 + m[z] * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient descent algorithm corrective step
    s[W] = -_2q2 * ( 2.0f * q1q3 - _2q0q2 - a[x] )
         + _2q1 * ( 2.0f * q0q1 + _2q2q3 - a[y] )
         - _2bz * q[Y] * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - m[x] )
         + ( -_2bx * q[Z] + _2bz * q[X] ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - m[y] )
         + _2bx * q[Y] * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - m[z] );
    s[X] = _2q3 * ( 2.0f * q1q3 - _2q0q2 - a[x] )
         + _2q0 * ( 2.0f * q0q1 + _2q2q3 - a[y] )
         - 4.0f * q[X] * ( 1.0f - 2.0f * q1q1 - 2.0f * q2q2 - a[z] )
         + _2bz * q[Z] * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - m[x] )
         + ( _2bx * q[Y] + _2bz * q[W] ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - m[y] )
         + ( _2bx * q[Z] - _4bz * q[X] ) * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - m[z] );
    s[Y] = -_2q0 * ( 2.0f * q1q3 - _2q0q2 - a[x] )
         + _2q3 * ( 2.0f * q0q1 + _2q2q3 - a[y] )
         - 4.0f * q[Y] * ( 1.0f - 2.0f * q1q1 - 2.0f * q2q2 - a[z] )
         + ( -_4bx * q[Y] - _2bz * q[W] ) * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - m[x] )
         + ( _2bx * q[X] + _2bz * q[Z] ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - m[y] )
         + ( _2bx * q[W] - _4bz * q[Y] ) * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - m[z] );
    s[Z] = _2q1 * ( 2.0f * q1q3 - _2q0q2 - a[x] )
         + _2q2 * ( 2.0f * q0q1 + _2q2q3 - a[y] )
         + ( -_4bx * q[Z] + _2bz * q[X] ) * ( _2bx * ( 0.5f - q2q2 - q3q3 ) + _2bz * ( q1q3 - q0q2 ) - m[x] )
         + ( -_2bx * q[W] + _2bz * q[Y] ) * ( _2bx * ( q1q2 - q0q3 ) + _2bz * ( q0q1 + q2q3 ) - m[y] )
         + _2bx * q[X] * ( _2bx * ( q0q2 + q1q3 ) + _2bz * ( 0.5f - q1q1 - q2q2 ) - m[z] );

    // Normalize step magnitude
    recipNorm = 1.0 / sqrt( s[W] * s[W] + s[X] * s[X] + s[Y] * s[Y] + s[Z] * s[Z] );
    for ( i=0; i<4; i++ )  s[i] *= recipNorm;

    // Apply feedback step
    for ( i=0; i<4; i++ )  dq[i] -= AHRS_GAIN * s[i];

  }

  // Integrate rate of change of quaternion to yield quaternion
  for ( i=0; i<4; i++ )  q[i] += dq[i] * dt;

  // Normalise quaternion
  recipNorm = 1.0 / sqrt( q[W] * q[W] + q[X] * q[X] + q[Y] * q[Y] + q[Z] * q[Z] );
  for ( i=0; i<4; i++ )  q[i] *= recipNorm;

  // Calculate Euler angles
  e[x] = atan2 ( ( 2.0 * ( q[W]*q[X] + q[Y]*q[Z] ) ), ( 1.0 - 2.0 * ( q[X]*q[X] + q[Y]*q[Y] ) ) );  //- ahrs->orient[X];
  e[y] = asin  (   2.0 * ( q[W]*q[Y] - q[X]*q[Z] ) );                                               //- ahrs->orient[Y];
  e[z] = atan2 ( ( 2.0 * ( q[W]*q[Z] + q[X]*q[Y] ) ), ( 1.0 - 2.0 * ( q[Y]*q[Y] + q[Z]*q[Z] ) ) );  //- ahrs->orient[Z];

  // Push values to AHRS data structure
  pthread_mutex_lock(&ahrs->mutex);
  for ( i=0; i<4; i++ )  {  ahrs->quat[i] = q[i];  ahrs->dquat[i] = dq[i];  }
  for ( i=0; i<3; i++ )  {  ahrs->eul[i]  = e[i];  ahrs->deul[i]  = 0.0;    }
  pthread_mutex_unlock(&ahrs->mutex);

  return;
}


/**
 *  ahrs_imu
 *  Implement 6DOF data fusion (IMU sensor only) algorithm.
 */
void ahrs_imu ( ahrs_struct *ahrs, imu_struct *imu )  {

  // Get values from AHRS data structure
  double dt, q0, q1, q2, q3;
  pthread_mutex_lock(&ahrs->mutex);
  dt = ahrs->dt;
  q0 = ahrs->quat[0];
  q1 = ahrs->quat[1];
  q2 = ahrs->quat[2];
  q3 = ahrs->quat[3];
  pthread_mutex_unlock(&ahrs->mutex);

  // Get gyr IMU data
  double gx, gy, gz;
  pthread_mutex_lock(&(imu->gyr->mutex));
  gx = imu->gyr->filter[0];
  gy = imu->gyr->filter[1];
  gz = imu->gyr->filter[2];
  pthread_mutex_unlock(&(imu->gyr->mutex));

  // Get acc IMU data
  double ax, ay, az;
  pthread_mutex_lock(&(imu->acc->mutex));
  ax = -imu->acc->filter[0];
  ay = -imu->acc->filter[1];
  az = -imu->acc->filter[2];
  pthread_mutex_unlock(&(imu->acc->mutex));

  // Local variables
  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * ( -q1 * gx  - q2 * gy  - q3 * gz );
  qDot2 = 0.5f * (  q0 * gx  + q2 * gz  - q3 * gy );
  qDot3 = 0.5f * (  q0 * gy  - q1 * gz  + q3 * gx );
  qDot4 = 0.5f * (  q0 * gz  + q1 * gy  - q2 * gx );

  // Compute feedback only if accelerometer measurement is valid
  if( ! ( (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f) ) ) {

    // Normalise accelerometer measurement
    recipNorm = 1.0 / sqrt( ax * ax + ay * ay + az * az );
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // Normalise step magnitude
    recipNorm = 1.0 / sqrt( s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3 );
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= AHRS_GAIN * s0;
    qDot2 -= AHRS_GAIN * s1;
    qDot3 -= AHRS_GAIN * s2;
    qDot4 -= AHRS_GAIN * s3;

  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;  // (1.0f / HZ_IMU_FAST );
  q1 += qDot2 * dt;  // (1.0f / HZ_IMU_FAST );
  q2 += qDot3 * dt;  // (1.0f / HZ_IMU_FAST );
  q3 += qDot4 * dt;  // (1.0f / HZ_IMU_FAST );

  // Normalise quaternion
  recipNorm = 1.0 / sqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 );
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Calculate Euler angles
  float ex, ey, ez;
  ex = atan2 ( ( 2.0 * ( q0*q1 + q2*q3 ) ), ( 1.0 - 2.0 * ( q1*q1 + q2*q2 ) ) );  //- ahrs->orient[X];
  ey = asin  (   2.0 * ( q0*q2 - q1*q3 ) );                                       //- ahrs->orient[Y];
  ez = atan2 ( ( 2.0 * ( q0*q3 + q1*q2 ) ), ( 1.0 - 2.0 * ( q2*q2 + q3*q3 ) ) );  //- ahrs->orient[Z];

  // Push values to AHRS data structure
  pthread_mutex_lock(&ahrs->mutex);
  ahrs->quat[0]  = q0;
  ahrs->quat[1]  = q1;
  ahrs->quat[2]  = q2;
  ahrs->quat[3]  = q3;
  ahrs->dquat[0] = qDot1;
  ahrs->dquat[1] = qDot2;
  ahrs->dquat[2] = qDot3;
  ahrs->dquat[3] = qDot4;
  ahrs->eul[0]   = ex;
  ahrs->eul[1]   = ey;
  ahrs->eul[2]   = ez;
  ahrs->deul[0]  = 0.0;
  ahrs->deul[1]  = 0.0;
  ahrs->deul[2]  = 0.0;
  pthread_mutex_unlock(&ahrs->mutex);

  return;
}



