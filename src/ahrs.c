

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
    ahrs->deul   [i]   = 0.0;
    ahrs->bias   [i]   = 0.0;
  }

  // Populate remaining values
  ahrs->quat[0]  = 1.0;
  ahrs->dquat[0] = 0.0;
  ahrs->fx       = 1.0;
  ahrs->fz       = -0.006;
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
 *  Implement 9DOF data fusion algorithm.
 */
void ahrs_update ( ahrs_struct *ahrs, imu_struct *imu )  {

  // Local variables
  //ushort i;
  //double norm;

  // Quaternion index
  //ushort w=0, x=1, y=2, z=3;
  //ushort X=0, Y=1, Z=2;

  /*
  // Get values from AHRS data structure
  double q[4], b[3], fx, fz, dt;
  pthread_mutex_lock(&ahrs->mutex);
  for ( i=0; i<4; i++ )  q[i] = ahrs->quat[i];
  for ( i=0; i<3; i++ )  b[i] = ahrs->bias[i];
  fx = ahrs->fx;  fz = ahrs->fz;  dt = ahrs->dt;
  pthread_mutex_unlock(&ahrs->mutex);

  // Zero out local variables
  double g[3], a[3], m[3];
  for ( i=0; i<3; i++ )  {  g[i] = 0.0;  a[i] = 0.0;  m[i] = 0.0;  }

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
  */

  /*
  // Normalize magnetometer
  norm = 0.0;
  for ( i=0; i<3; i++ )  norm += m[i] * m[i];
  norm = sqrt(norm);
  for ( i=0; i<3; i++ )  m[i] /= norm;
 
  // Normalize accelerometer
  norm = 0.0;
  for ( i=0; i<3; i++ )  norm += a[i] * a[i];
  norm = sqrt(norm);
  for ( i=0; i<3; i++ )  a[i] /= norm;

  // Define auxiliary vectors
  double halfq[4], twoq[4], twom[3], twofxq[4], twofzq[4], twofx, twofz;
  twofx = 2.0 * fx;  twofz = 2.0 * fz;
  for ( i=0; i<4; i++ )  {
    halfq[i]  =   0.5 * q[i];
    twoq[i]   =   2.0 * q[i];
    twofxq[i] = twofx * q[i];
    twofzq[i] = twofz * q[i];
    if(i<3)  twom[i] = 2.0 * m[i];
  }

  // Quaternion multiplication values 
  double qwx, qwy, qwz, qxy, qxz, qyz, qxx, qyy, qzz;
  qwx = q[w] * q[x];  qwy = q[w] * q[y];  qwz = q[w] * q[z];
  qxy = q[x] * q[y];  qxz = q[x] * q[z];  qyz = q[y] * q[z];
  qxx = q[x] * q[x];  qyy = q[y] * q[y];  qzz = q[z] * q[z];

  // Calculate objective function
  double F1, F2, F3, F4, F5, F6;
  F1 =        twoq[x] * q[z]        - twoq[w] * q[y]             - a[X];
  F2 =        twoq[w] * q[x]        + twoq[y] * q[z]             - a[Y];
  F3 = 1.0  - twoq[x] * q[x]        - twoq[y] * q[y]             - a[Z];
  F4 = twofx * ( 0.5 - qyy - qzz ) + twofz * (       qxz - qwy ) - m[X]; 
  F5 = twofx * (       qxy - qwz ) + twofz * (       qwx + qyz ) - m[Y];
  F6 = twofx * (       qwy + qxz ) + twofz * ( 0.5 - qxx - qyy ) - m[Z];

  // Calculate jacobian matrix
  double J11J24, J12J23, J13J22, J14J21, J32, J33;
  double J41, J42, J43, J44;
  double J51, J52, J53, J54;
  double J61, J62, J63, J64;
  J11J24 = twoq[y];
  J12J23 = 2.0 * q[z];
  J13J22 = twoq[w];
  J14J21 = twoq[x];
  J32 = 2.0 * J14J21;
  J33 = 2.0 * J11J24;
  J41 = twofzq[y];
  J42 = twofzq[z];
  J43 = 2.0 * twofxq[y] + twofzq[w]; 
  J44 = 2.0 * twofxq[z] - twofzq[x];
  J51 = twofxq[z] - twofzq[x];
  J52 = twofxq[y] + twofzq[w];
  J53 = twofxq[x] + twofzq[z];
  J54 = twofxq[w] - twofzq[y];
  J61 = twofxq[y];
  J62 = twofxq[z] - 2.0 * twofzq[x];
  J63 = twofxq[w] - 2.0 * twofzq[y];
  J64 = twofxq[x];

  // Quaternion gradient from objective function
  double qdf[4];
  qdf[w] = - J11J24 * F1 + J14J21 * F2            - J41 * F4 - J51 * F5 + J61 * F6;
  qdf[x] =   J12J23 * F1 + J13J22 * F2 - J32 * F3 + J42 * F4 + J52 * F5 + J62 * F6;
  qdf[y] = - J13J22 * F1 + J12J23 * F2 - J33 * F3 - J43 * F4 + J53 * F5 + J63 * F6;
  qdf[z] =   J14J21 * F1 + J11J24 * F2            - J44 * F4 - J54 * F5 + J64 * F6;

  // Normalize gradient
  norm = 0.0;
  for ( i=0; i<4; i++ )  norm += qdf[i] * qdf[i];
  norm = sqrt(norm);
  for ( i=0; i<4; i++ )  qdf[i] /= norm;

  // Compute gyroscope error
  double err[3];
  err[X] = twoq[w] * qdf[x] - twoq[x] * qdf[w] - twoq[y] * qdf[z] + twoq[z] * qdf[y];
  err[Y] = twoq[w] * qdf[y] + twoq[x] * qdf[z] - twoq[y] * qdf[w] - twoq[z] * qdf[x];
  err[Z] = twoq[w] * qdf[z] - twoq[x] * qdf[y] + twoq[y] * qdf[x] - twoq[z] * qdf[w];

  // Adjust for gyroscope baises
  for ( i=0; i<3; i++ )  {
    b[i] += err[i] * dt * IMU_ZETA;
    g[i] -= b[i];
  }

  // Quaternion derivative from rate gyro
  double qdr[4];
  qdr[w] = -halfq[x] * g[X] - halfq[y] * g[Y] - halfq[z] * g[Z];
  qdr[x] =  halfq[w] * g[X] + halfq[y] * g[Z] - halfq[z] * g[Y];
  qdr[y] =  halfq[w] * g[Y] - halfq[x] * g[Z] + halfq[z] * g[X];
  qdr[z] =  halfq[w] * g[Z] + halfq[x] * g[Y] - halfq[y] * g[X];

  // Fused quaternion and derivative values
  double qd[4];
  for ( i=0; i<4; i++ )  {
    qd[i] = qdr[i] - ( IMU_BETA * qdf[i] );
    q[i] += qd[i] * dt;
  }

  // Normalise quaternion
  norm = 0.0;
  for ( i=0; i<4; i++ )  norm += q[i] * q[i];
  norm = sqrt(norm);
  for ( i=0; i<4; i++ )  q[i] /= norm;

  // Compute Earth frame flux
  float h[3];
  h[X] = twom[X] * ( 0.5 - qyy - qzz ) + twom[Y] * ( qxy - qwz ) + twom[Z] * ( qxz + qwy );
  h[Y] = twom[X] * ( qxy + qwz ) + twom[Y] * ( 0.5 - qxx - qzz ) + twom[Z] * ( qyz - qwx ); 
  h[Z] = twom[X] * ( qxz - qwy ) + twom[Y] * ( qyz + qwx ) + twom[Z] * ( 0.5 - qxx - qyy );

  // Adjust flux vector
  fx = sqrt( ( h[X] * h[X] ) + ( h[Y] * h[Y] ) );
  fz = h[Z];

  // Calculate euler angles
  double e[3];
  e[X] = atan2 ( ( 2.0 * ( qwx + qyz ) ), ( 1.0 - 2.0 * ( qxx + qyy ) ) ) - ahrs->orient[X];
  e[Y] = asin  (   2.0 * ( qwy - qxz ) )                                  - ahrs->orient[Y];
  e[Z] = atan2 ( ( 2.0 * ( qwz + qxy ) ), ( 1.0 - 2.0 * ( qyy + qzz ) ) ) - ahrs->orient[Z];
  */

  MadgwickAHRSupdateIMU( ahrs, imu );


  /*
  // Push AHRS data to struct
  pthread_mutex_lock(&ahrs->mutex);
  for ( i=0; i<4; i++ )  {
    ahrs->quat[i]  = q[i];
    ahrs->dquat[i] = qd[i];
  }
  for ( i=0; i<3; i++ ) {
    ahrs->eul[i]  = e[i];
    ahrs->deul[i] = g[i];
    ahrs->bias[i] = b[i];
  }
  ahrs->fx = fx;  ahrs->fz = fz;
  pthread_mutex_unlock(&ahrs->mutex);
  */

  return;
}



//---------------------------------------------------------------------------------------------------
// Header files
//#include "MadgwickAHRS.h"
//#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions
//#define sampleFreq   512.0f    // sample frequency in Hz
//#define betaDef        0.1f    // 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions
//volatile float beta = betaDef; // 2 * proportional gain (Kp)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
//float invSqrt(float x);


//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate( ) { //float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {

  /*
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
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
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  */

  return;
}


//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void MadgwickAHRSupdateIMU( ahrs_struct *ahrs, imu_struct *imu ) {

  // Define beta gain (move to GCS???)
  float beta = 0.1;

  // Get values from AHRS data structure
  double q0, q1, q2, q3;
  pthread_mutex_lock(&ahrs->mutex);
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
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
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
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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

  // Integrate rate of change of quaternion to yield quaternion  // Orig: sampleFreq
  q0 += qDot1 * (1.0f / HZ_IMU_FAST );
  q1 += qDot2 * (1.0f / HZ_IMU_FAST );
  q2 += qDot3 * (1.0f / HZ_IMU_FAST );
  q3 += qDot4 * (1.0f / HZ_IMU_FAST );

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
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
      return 1.0f / sqrtf(x);
      //}


      //return 0.0;
}



