

#include "ahrs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "filter.h"
//#include "imu.h"
#include "sys.h"
#include "timer.h"


/**
 *  ahrs_init
 *  Initializes the attitude and heading reference algorithms.
 */
void ahrs_init ( void )  {
  if(DEBUG)  printf( "Initializing AHRS \n" );

  // Local variable
  //ushort i;

  // Setup data structures
  ahrs_setup( &ahrsA );
  ahrs_setup( &ahrsB );

  /*
  // Set Euler angle bias
  FILE* f;
  char buff [32];  memset( buff, 0, sizeof(buff) );
  char path [32];  memset( path, 0, sizeof(path) );
  sprintf( path, "../Param/board/bias/eul" );
  f = fopen( path, "r" );
  if(!f)  printf( "Error (ahr_init): File for 'eul bias' not found. \n" );
  for ( i=0; i<3; i++ ) {
    fgets( buff, 32, f );
    ahrs.orient[i] = atoi(buff) / 10000.0;
  }
  fclose(f);

  // Display Euler attitude offset
  if (DEBUG)  {
    printf("  Euler offset:" );
    for ( i=0; i<3; i++ )  printf(" %6.3f", ahrs.orient[i] );
    printf("\n");
  }
  */

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
  ahrs->dt       = 1.0 / HZ_AHRS;

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
  ushort i;
  double norm;

  // Quaternion index
  ushort w=0, x=1, y=2, z=3;
  ushort X=0, Y=1, Z=2;

  // Get values from AHRS data structure
  double q[4], b[3], fx, fz, dt;
  pthread_mutex_lock(&ahrs->mutex);
  for ( i=0; i<4; i++ )  q[i] = ahrs->quat[i];
  fx = ahrs->fx;  fz = ahrs->fz;  dt = ahrs->dt;
  for ( i=0; i<3; i++ )  b[i] = ahrs->bias[i];
  pthread_mutex_unlock(&ahrs->mutex);

  // Get values from IMU data structure
  double g[3], a[3], m[3];
  for ( i=0; i<3; i++ )  {  g[i] = 0.0;  a[i] = 0.0;  m[i] = 0.0;  }

  //if (IMUA_ENABLED) {

  //if ( imu->id == 'A' )  pthread_mutex_lock(&mutex_gyrA);
  //if ( imu->id == 'B' )  pthread_mutex_lock(&mutex_gyrB);
  pthread_mutex_lock(&(imu->gyr->mutex));
  for ( i=0; i<3; i++ )  g[i] +=  imu->gyr->filter[i];
  pthread_mutex_unlock(&(imu->gyr->mutex));
  //if ( imu->id == 'A' )  pthread_mutex_unlock(&mutex_gyrA);
  //if ( imu->id == 'B' )  pthread_mutex_unlock(&mutex_gyrB);

  //if ( imu->id == 'A' )  pthread_mutex_lock(&mutex_accA);
  //if ( imu->id == 'B' )  pthread_mutex_lock(&mutex_accB);
  pthread_mutex_lock(&(imu->acc->mutex));
  for ( i=0; i<3; i++ )  a[i] += -imu->acc->filter[i];
  pthread_mutex_lock(&(imu->acc->mutex));
  //if ( imu->id == 'A' )  pthread_mutex_unlock(&mutex_accA);
  //if ( imu->id == 'B' )  pthread_mutex_unlock(&mutex_accB);

  //if ( imu->id == 'A' )  pthread_mutex_lock(&mutex_magA);
  //if ( imu->id == 'B' )  pthread_mutex_lock(&mutex_magB);
  pthread_mutex_lock(&(imu->mag->mutex));
  for ( i=0; i<3; i++ )  m[i] +=  imu->mag->filter[i];
  pthread_mutex_lock(&(imu->mag->mutex));
  //if ( imu->id == 'A' )  pthread_mutex_unlock(&mutex_magA);
  //if ( imu->id == 'B' )  pthread_mutex_unlock(&mutex_magB);

  //}
  /*
  if (IMUB_ENABLED) {

    pthread_mutex_lock(&mutex_gyrB);
    for ( i=0; i<3; i++ )  g[i] +=  gyrB.filter[i];
    pthread_mutex_unlock(&mutex_gyrB);

    pthread_mutex_lock(&mutex_accB);
    for ( i=0; i<3; i++ )  a[i] += -accB.filter[i];
    pthread_mutex_unlock(&mutex_accB);

    pthread_mutex_lock(&mutex_magB);
    for ( i=0; i<3; i++ )  m[i] +=  magB.filter[i];
    pthread_mutex_unlock(&mutex_magB);

  }

  if ( IMUA_ENABLED && IMUB_ENABLED )  {
    for ( i=0; i<3; i++ )  {
      g[i] /= 2.0;
      a[i] /= 2.0;
      m[i] /= 2.0;
    }
  }
  */

  // Store averaged values to AHRS data structure
  //pthread_mutex_lock(&mutex_ahrs);
  //for ( i=0; i<3; i++ )  {
    //ahrs.gyr[i] = g[i];
    //ahrs.acc[i] = a[i];
    //ahrs.mag[i] = m[i];
  //}
  //pthread_mutex_unlock(&mutex_ahrs);

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
  e[X] = atan2 ( ( 2.0 * ( qwx + qyz ) ), ( 1.0 - 2.0 * ( qxx + qyy ) ) );// - ahrs.orient[X];
  e[Y] = asin  (   2.0 * ( qwy - qxz ) )                                 ;// - ahrs.orient[Y];
  e[Z] = atan2 ( ( 2.0 * ( qwz + qxy ) ), ( 1.0 - 2.0 * ( qyy + qzz ) ) );// - ahrs.orient[Z];

  // Apply low pass filters
  //double lpfe[3], lpfg[3];
  //filter_lpf ( &filter_eul, e, lpfe );
  //filter_lpf ( &filter_ang, g, lpfg );

  // Push quaternion data to struct
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

  // Push LPF Euler data to struct
  //pthread_mutex_lock(&mutex_lpfeul);
  //for ( i=0; i<3; i++ ) {
  //  ahrs.lpfeul[i] = lpfe[i];
  //  ahrs.lpfang[i] = lpfg[i];
  //}
  //pthread_mutex_unlock(&mutex_lpfeul);

  return;
}



