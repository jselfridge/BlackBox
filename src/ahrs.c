
//============================================================
//  ahrs.c
//  Justin M Selfridge
//============================================================
#include "ahrs.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ahrs_init
//  Initializes the AHRS algorithms.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ahrs_init (  )  {
  if(DEBUG)  printf( "Initializing AHRS \n" );

/*
  // Data fusion variables
  imu->fx = 0.5;  imu->fz = 0.866;
  for ( i=0; i<4; i++ ) {
    //imu->Prev[i]  = 0;
    imu->Quat[i]  = 0;
    imu->dQuat[i] = 0;
    if (i<3) {
      imu->Eul[i]  = 0;
      imu->dEul[i] = 0;
      imu->bias[i] = 0;
    }
  }
  //imu->Prev[0] = 1;
  imu->Quat[0] = 1;
*/


  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ahrs_exit
//  Terminate the AHRS algorithms.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ahrs_exit ( void )  {
  if(DEBUG)  printf("Close AHRS \n");
  // Insert code if needed...
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ahrs_fusion
//  Implement 9DOF data fusion algorithm.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ahrs_fusion ( void )  {

  /*
  // Local variables
  unsigned short i;
  double norm;

  // Quaternion index
  unsigned short w=0, x=1, y=2, z=3;

  // Get values from mpu structure
  double q[4], m[3], a[3], g[3], b[3], fx, fz, dt;
  fx = imu->fx;  fz = imu->fz;  dt = imu->fus_dt;

  //pthread_mutex_lock(&mutex_cal);

  for ( i=0; i<4; i++ ) {
    q[i] = imu->Quat[i];
    if (i<3) {
      g[i] = imu->calGyr[i];
      a[i] = imu->calAcc[i];
      m[i] = imu->calMag[i];
      b[i] = imu->bias[i];
    }
  }

  //pthread_mutex_unlock(&mutex_cal);

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
  twofx = 2.0f * fx;  twofz = 2.0f * fz;
  for ( i=0; i<4; i++ ) {
    halfq[i]  =  0.5f * q[i];
    twoq[i]   =  2.0f * q[i];
    twofxq[i] = twofx * q[i];
    twofzq[i] = twofz * q[i];
    if(i<3)  twom[i] = 2.0f * m[i];
  }

  // Quaternion multiplication values 
  double qwx, qwy, qwz, qxy, qxz, qyz, qxx, qyy, qzz;
  qwx = q[w] * q[x];  qwy = q[w] * q[y];  qwz = q[w] * q[z];
  qxy = q[x] * q[y];  qxz = q[x] * q[z];  qyz = q[y] * q[z];
  qxx = q[x] * q[x];  qyy = q[y] * q[y];  qzz = q[z] * q[z];

  // Calculate objective function
  double F1, F2, F3, F4, F5, F6;
  F1 =        twoq[x] * q[z]        - twoq[w] * q[y]               - a[X];
  F2 =        twoq[w] * q[x]        + twoq[y] * q[z]               - a[Y];
  F3 = 1.0f - twoq[x] * q[x]        - twoq[y] * q[y]               - a[Z];
  F4 = twofx * ( 0.5f - qyy - qzz ) + twofz * (        qxz - qwy ) - m[X]; 
  F5 = twofx * (        qxy - qwz ) + twofz * (        qwx + qyz ) - m[Y];
  F6 = twofx * (        qwy + qxz ) + twofz * ( 0.5f - qxx - qyy ) - m[Z];

  // Calculate jacobian matrix
  double J11J24, J12J23, J13J22, J14J21, J32, J33;
  double J41, J42, J43, J44;
  double J51, J52, J53, J54;
  double J61, J62, J63, J64;
  J11J24 = twoq[y];
  J12J23 = 2.0f * q[z];
  J13J22 = twoq[w];
  J14J21 = twoq[x];
  J32 = 2.0f * J14J21;
  J33 = 2.0f * J11J24;
  J41 = twofzq[y];
  J42 = twofzq[z];
  J43 = 2.0f * twofxq[y] + twofzq[w]; 
  J44 = 2.0f * twofxq[z] - twofzq[x];
  J51 = twofxq[z] - twofzq[x];
  J52 = twofxq[y] + twofzq[w];
  J53 = twofxq[x] + twofzq[z];
  J54 = twofxq[w] - twofzq[y];
  J61 = twofxq[y];
  J62 = twofxq[z] - 2.0f * twofzq[x];
  J63 = twofxq[w] - 2.0f * twofzq[y];
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
  for ( i=0; i<3; i++ ) {
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
  for ( i=0; i<4; i++ ) {
    qd[i] = qdr[i] - ( IMU_BETA * qdf[i] );
    q[i] += qd[i] * dt;
  }

  // Normalise quaternion
  norm = 0.0;
  for ( i=0; i<4; i++ )  norm += q[i] * q[i];
  norm = sqrt(norm);
  for ( i=0; i<4; i++ )  q[i] /= norm;

  // Compute earth frame flux
  float h[3];
  h[X] = twom[X] * ( 0.5f - qyy - qzz ) + twom[Y] * ( qxy - qwz ) + twom[Z] * ( qxz + qwy );
  h[Y] = twom[X] * ( qxy + qwz ) + twom[Y] * ( 0.5f - qxx - qzz ) + twom[Z] * ( qyz - qwx ); 
  h[Z] = twom[X] * ( qxz - qwy ) + twom[Y] * ( qyz + qwx ) + twom[Z] * ( 0.5f - qxx - qyy );

  // Adjust flux vector
  fx = sqrt( ( h[X] * h[X] ) + ( h[Y] * h[Y] ) );
  fz = h[Z];

  // Calculate euler angles
  double e[3];
  e[X] = atan2 ( ( 2* ( qwx + qyz ) ), ( 1- 2* ( qxx + qyy ) ) ) - R_BIAS;
  e[Y] = asin  (   2* ( qwy - qxz ) )                            - P_BIAS;
  e[Z] = atan2 ( ( 2* ( qwz + qxy ) ), ( 1- 2* ( qyy + qzz ) ) ) - Y_BIAS;

  // Update imu structure

  //pthread_mutex_lock(&mutex_fusion);

  imu->fx = fx;  imu->fz = fz;
  for ( i=0; i<4; i++ ) {
    imu->Quat[i]  = q[i];
    imu->dQuat[i] = qd[i];
    if(i<3) {
      imu->Eul[i]  = e[i];
      imu->dEul[i] = g[i];
      imu->bias[i] = b[i];
    }
  }

  //pthread_mutex_unlock(&mutex_fusion);
  */

  return;
}




//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  ahrs_kalman
//  Implement Kalman filter algorithm.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ahrs_kalman ( void )  {
  // Future work
  return;
}




/*
  //~~~  MOVE TO 'CONVERGENCE' FUNCTION  ~~~//
  
  double temp[3];
  for( i=0; i<3; i++ )  temp[i] = imu->Eul[i] - imu->Prev[i];
  // Norm of delta
  norm = 0.0;
  for ( i=0; i<3; i++ )  norm += temp[i] * temp[i];
  norm = sqrt(norm);
  // Debugging statements
  printf("E: %7.4f %7.4f %7.4f    ", imu->Eul[0], imu->Eul[1], imu->Eul[2] );
  printf("D: %7.4f %7.4f %7.4f    ", temp[0], temp[1], temp[2] );
  printf("N: %7.4f", norm);
  //printf("E: %7.4f  A: %7.4f    ", imu->Eul[0], temp );
  printf("\n");
  fflush(stdout);
  // Assign previous value
  for ( i=0; i<3; i++ )  imu->Prev[i] = imu->Eul[i];
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  */



