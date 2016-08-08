

#include "ahrs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys.h"
#include "timer.h"


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

  // Local variable
  ushort i;

  // Loop through entries with known zero values
  for ( i=0; i<3; i++ ) {
    ahrs->quat   [i+1] = 0.0;
    ahrs->dquat  [i+1] = 0.0;
    ahrs->eul    [i]   = 0.0;
    //ahrs->deul   [i]   = 0.0;
  }

  // Populate remaining values
  ahrs->quat[0]  = 1.0;
  ahrs->dquat[0] = 0.0;
  ahrs->dt       = 1.0 / HZ_IMU_FAST;

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

  // Gain value
  float beta = 0.1;

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
  float gx, gy, gz;
  pthread_mutex_lock(&(imu->gyr->mutex));
  gx = imu->gyr->filter[0];
  gy = imu->gyr->filter[1];
  gz = imu->gyr->filter[2];
  pthread_mutex_unlock(&(imu->gyr->mutex));

  // Get acc IMU data
  float ax, ay, az;
  pthread_mutex_lock(&(imu->acc->mutex));
  ax = -imu->acc->filter[0];
  ay = -imu->acc->filter[1];
  az = -imu->acc->filter[2];
  pthread_mutex_unlock(&(imu->acc->mutex));

  // Get mag IMU data
  float mx, my, mz;
  pthread_mutex_lock(&(imu->mag->mutex));
  mx = imu->mag->filter[0];
  my = imu->mag->filter[1];
  mz = imu->mag->filter[2];
  pthread_mutex_unlock(&(imu->mag->mutex));

  // Local variables
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
  float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
  float  q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid
  if ( (mx == 0.0f) && (my == 0.0f) && (mz == 0.0f) )  {
    ahrs_imu ( ahrs, imu );
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * ( -q1 * gx  - q2 * gy  - q3 * gz );
  qDot2 = 0.5f * (  q0 * gx  + q2 * gz  - q3 * gy );
  qDot3 = 0.5f * (  q0 * gy  - q1 * gz  + q3 * gx );
  qDot4 = 0.5f * (  q0 * gz  + q1 * gy  - q2 * gx );

  // Compute feedback only if accelerometer measurement is valid
  if( ! ( (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f) ) )  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt( ax * ax + ay * ay + az * az );
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt( mx * mx + my * my + mz * mz );
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
         - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
         - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
         + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
         - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
         + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
         + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    // Normalize step magnitude
    recipNorm = invSqrt( s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3 );
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;  // ( 1.0f / HZ_IMU_FAST );
  q1 += qDot2 * dt;  // ( 1.0f / HZ_IMU_FAST );
  q2 += qDot3 * dt;  // ( 1.0f / HZ_IMU_FAST );
  q3 += qDot4 * dt;  // ( 1.0f / HZ_IMU_FAST );

  // Normalise quaternion
  recipNorm = invSqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 );
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
  pthread_mutex_unlock(&ahrs->mutex);

  return;
}


/**
 *  ahrs_imu
 *  Implement 6DOF data fusion (IMU sensor only) algorithm.
 */
void ahrs_imu ( ahrs_struct *ahrs, imu_struct *imu )  {

  // Gain value
  float beta = 0.1;

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
  float gx, gy, gz;
  pthread_mutex_lock(&(imu->gyr->mutex));
  gx = imu->gyr->filter[0];
  gy = imu->gyr->filter[1];
  gz = imu->gyr->filter[2];
  pthread_mutex_unlock(&(imu->gyr->mutex));

  // Get acc IMU data
  float ax, ay, az;
  pthread_mutex_lock(&(imu->acc->mutex));
  ax = -imu->acc->filter[0];
  ay = -imu->acc->filter[1];
  az = -imu->acc->filter[2];
  pthread_mutex_unlock(&(imu->acc->mutex));

  // Local variables
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * ( -q1 * gx  - q2 * gy  - q3 * gz );
  qDot2 = 0.5f * (  q0 * gx  + q2 * gz  - q3 * gy );
  qDot3 = 0.5f * (  q0 * gy  - q1 * gz  + q3 * gx );
  qDot4 = 0.5f * (  q0 * gz  + q1 * gy  - q2 * gx );

  // Compute feedback only if accelerometer measurement is valid
  if( ! ( (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f) ) ) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt( ax * ax + ay * ay + az * az );
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
    recipNorm = invSqrt( s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3 );
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;  // (1.0f / HZ_IMU_FAST );
  q1 += qDot2 * dt;  // (1.0f / HZ_IMU_FAST );
  q2 += qDot3 * dt;  // (1.0f / HZ_IMU_FAST );
  q3 += qDot4 * dt;  // (1.0f / HZ_IMU_FAST );

  // Normalise quaternion
  recipNorm = invSqrt( q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 );
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
  ahrs->quat[0] = q0;
  ahrs->quat[1] = q1;
  ahrs->quat[2] = q2;
  ahrs->quat[3] = q3;
  ahrs->dquat[0] = qDot1;
  ahrs->dquat[1] = qDot2;
  ahrs->dquat[2] = qDot3;
  ahrs->dquat[3] = qDot4;
  ahrs->eul[0]  = ex;
  ahrs->eul[1]  = ey;
  ahrs->eul[2]  = ez;
  pthread_mutex_unlock(&ahrs->mutex);

  return;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {

  /*
  if (instability_fix == 0)
    {
      // original code
      float halfx = 0.5f * x;
      float y = x;
      //long i = *(long*)&y;  // Original
      long i = (union { float f; long l; }) {y} .l;  // Revised
      i = 0x5f3759df - (i>>1);
      //y = *(float*)&i;  // Original
      y = (union { long l; float f; }) {i} .f;
      y = y * (1.5f - (halfx * y * y));
      return y;
    }
  else if (instability_fix == 1)
    {
      // close-to-optimal  method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root
      unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
      float tmp = *(float*)&i;
      return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
    }
  else
    {
  */
      // optimal but expensive method:
      return 1.0f / sqrt(x);
      //}


      //return 0.0;
}



