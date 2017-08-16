

//#include "ekf.h"
//#include <stdio.h>
//#include "imu.h"
//#include "sys.h"
//#include "timer.h"


/**
 * Initializes the EKF structure.
 * @param ekf pointer to an EKF structure to be initialized
 * @param n number of system states
 * @param m number of measurements
 *
 * <tt>ekf</tt> should be a pointer to a structure defined as follows, 
 * where <tt>N</tt> and </tt>M</tt> are constants:
 * <pre>
      int n;           // number of state values
      int m;           // number of observables
      double x[N];     // state vector
      double P[N][N];  // prediction error covariance
      double Q[N][N];  // process noise covariance
      double R[M][M];  // measurement error covariance
      double G[N][M];  // Kalman gain; a.k.a. K
      double F[N][N];  // Jacobian of process model
      double H[M][N];  // Jacobian of measurement model
      double Ht[N][M]; // transpose of measurement Jacobian
      double Ft[N][N]; // transpose of process Jacobian
      double Pp[N][N]; // P, post-prediction, pre-update
      double fx[N];    // output of user defined f() state-transition function
      double hx[M];    // output of user defined h() measurement function
 * </pre>
 */
/*
void ekf_init ( void )  {
  if (DEBUG)  printf( "Initializing EKF \n" );

  // Local variables
  uint n  = EKF_N;
  uint m  = EKF_M;

  // Enable mutex lock
  pthread_mutex_init( &ekf.mutex, NULL );

  // Allocate memory for storage arrays
  ekf.x  = mat_init(n,1);
  ekf.z  = mat_init(m,1);
  ekf.f  = mat_init(n,1);
  ekf.h  = mat_init(m,1);
  ekf.F  = mat_init(n,n);
  ekf.H  = mat_init(m,n);
  ekf.Q  = mat_init(n,n);
  ekf.R  = mat_init(m,m);
  ekf.P  = mat_init(n,n);
  ekf.T  = mat_init(n,n);
  ekf.S  = mat_init(m,m);
  ekf.K  = mat_init(n,m);

  // Assign plant covariance values
  mat_set( ekf.Q, 1, 1, 0.1 );
  mat_set( ekf.Q, 2, 2, 0.1 );

  // Assign measurement covariance values
  mat_set( ekf.R, 1, 1, 0.1 );
  mat_set( ekf.R, 2, 2, 0.1 );
  mat_set( ekf.R, 3, 3, 0.1 );
  mat_set( ekf.R, 4, 4, 0.1 );

  // Initialize prediction matrix
  mat_set( ekf.P, 1, 1, 0.1 );
  mat_set( ekf.P, 2, 2, 0.1 );

  // Static F matrix
  double dt  = 1.0 / HZ_IMU;
  ekf.F = mat_eye(n);
  mat_set( ekf.F, 1,2, dt );

  // Static H matrix
  mat_set( ekf.H, 1, 1, 1 );  mat_set( ekf.H, 1, 2, 0 );
  mat_set( ekf.H, 2, 1, 1 );  mat_set( ekf.H, 2, 2, 0 );
  mat_set( ekf.H, 3, 1, 0 );  mat_set( ekf.H, 3, 2, 1 );
  mat_set( ekf.H, 4, 1, 0 );  mat_set( ekf.H, 4, 2, 1 );

  // Display EKF settings
  if (DEBUG) {
    printf("  Q ");  mat_print(ekf.Q);
    printf("  R ");  mat_print(ekf.R);
    printf("  P ");  mat_print(ekf.P);
    printf("  F ");  mat_print(ekf.F);
    printf("  H ");  mat_print(ekf.H);
  }

  return;
}
*/

/**
 * Exits the EKF routine
 */
/*
void ekf_exit ( void )  {
  if (DEBUG)  printf("Close EKF \n");
  pthread_mutex_destroy(&ekf.mutex);
  return;
}
*/

/**
 *
 * Runs one step of EKF prediction and update. Your code should first build
 * a model, setting the contents of <tt>ekf.fx</tt>, <tt>ekf.F</tt>,
 * <tt>ekf.hx</tt>, and <tt>ekf.H</tt> to appropriate values.
 * @param ekf pointer to structure EKF
 * @param z array of measurement (observation) values
 * @return 0 on success, -1 on failure caused by non-positive-definite matrix.
 */
/*
void ekf_update ( void )  {

  // Define local variables
  ushort n = EKF_N;
  ushort m = EKF_M;
  //ushort r, c;
  //double num, den, val;

  // Initialize local EKF arrays
  matrix *x  = mat_init(n,1);
  matrix *z  = mat_init(m,1);
  matrix *f  = mat_init(n,1);
  matrix *h  = mat_init(m,1);
  //matrix *xp = mat_init(n,1);
  //matrix *xd = mat_init(n,1);
  //matrix *fp = mat_init(n,1);
  //matrix *fd = mat_init(n,1);

  // Initialize local EKF matrices
  matrix *F = mat_init(n,n);
  matrix *H = mat_init(m,n);
  matrix *Q = mat_init(n,n);
  matrix *R = mat_init(m,m);  
  matrix *P = mat_init(n,n);
  matrix *T = mat_init(n,n);
  matrix *S = mat_init(m,m);
  matrix *K = mat_init(n,m);

  // Initialize transpose matrices
  matrix *Ft  = mat_init(n,n);
  matrix *Ht  = mat_init(n,m);

  // Initialize temp storage matrices
  matrix *tmpNN = mat_init(n,n);
  matrix *tmpNM = mat_init(n,m);
  matrix *tmpMN = mat_init(m,n);
  matrix *tmpN  = mat_init(n,1);
  matrix *tmpM  = mat_init(m,1);

  // Obtain EKF values from data structure
  pthread_mutex_lock(&ekf.mutex);
  x = mat_copy(ekf.x);
  f = mat_copy(ekf.f);
  F = mat_copy(ekf.F);
  H = mat_copy(ekf.H);
  Q = mat_copy(ekf.Q);
  R = mat_copy(ekf.R);
  P = mat_copy(ekf.P);
  K = mat_copy(ekf.K);
  pthread_mutex_unlock(&ekf.mutex);

  // Obtain AHRS A measurements
  pthread_mutex_lock(&ahrsA.mutex);
  double Ra = ahrsA.eul[0];
  //double Pa = ahrsA.eul[1];
  //double Ya = ahrsA.eul[2];
  pthread_mutex_unlock(&ahrsA.mutex);

  // Obtain AHRS B measurements
  pthread_mutex_lock(&ahrsB.mutex);
  double Rb = ahrsB.eul[0];
  //double Pb = ahrsB.eul[1];
  //double Yb = ahrsB.eul[2];
  pthread_mutex_unlock(&ahrsB.mutex);

  // Obtain gyro A measurements
  pthread_mutex_lock(&gyrA.mutex);
  double dRa = gyrA.filter[0];
  //double dPa = gyrA.filter[1];
  //double dYa = gyrA.filter[2];
  pthread_mutex_unlock(&gyrA.mutex);

  // Obtain gyro B measurements
  pthread_mutex_lock(&gyrB.mutex);
  double dRb = gyrB.filter[0];
  //double dPb = gyrB.filter[1];
  //double dYb = gyrB.filter[2];
  pthread_mutex_unlock(&gyrB.mutex);

  // Populate measurement vector
  mat_set( z, 1,1,  Ra );
  mat_set( z, 2,1,  Rb );
  mat_set( z, 3,1, dRa );
  mat_set( z, 4,1, dRb );

  // Save previous states before updating
  //xp = mat_copy(x);
  //fp = mat_copy(f);

  // Find derivatives and transposes
  f = mat_mul( F, x );  Ft = mat_trans(F);
  h = mat_mul( H, x );  Ht = mat_trans(H);

  // T = F P Ft + Q
  tmpNN = mat_mul( F, P );
  T = mat_mul( tmpNN, Ft );
  T = mat_add( T, Q );
  mat_sym( T, 0.00001 );

  // S = H T Ht + R
  tmpMN = mat_mul( H, T );
  S = mat_mul( tmpMN, Ht );
  S = mat_add( S, R );
  mat_sym( S, 0.00001 );

  // x = f + K ( z - h )
  tmpM = mat_sub( z, h );
  tmpN = mat_mul( K, tmpM );
  x = mat_add( f, tmpN );

  // P = ( I - K H ) T
  tmpNN = mat_mul( K, H );
  tmpNN = mat_sub( mat_eye(n), tmpNN );
  P = mat_mul( tmpNN, T );
  mat_sym( P, 0.00001 );

  // Update F matrix
//  fd = mat_sub( f, fp );  //mat_print(fd);
//  xd = mat_sub( x, xp );  //mat_print(xd);
//  for ( r=1; r<=n; r++ )  {
//    num = mat_get( xd,r,1 );  //printf("num: %f\n",num);  fflush(stdout);
//    for ( c=1; c<=n; c++ )  {
//      den = mat_get( xd,c,1 );  //printf("den: %f\n",den);  fflush(stdout);
//      if (den)  {
//        val = num / den;          //printf("val: %f\n",val);  fflush(stdout);
//        mat_set( F,r,c, val );
//      }
//    }
//  }
//  mat_print(F);

  // Push data to structure
  pthread_mutex_lock(&ekf.mutex);
  ekf.x  = mat_copy(x);
  ekf.z  = mat_copy(z);
  ekf.f  = mat_copy(f);
  ekf.h  = mat_copy(h);  
  ekf.F  = mat_copy(F);
  ekf.P  = mat_copy(P);
  ekf.T  = mat_copy(T);
  ekf.S  = mat_copy(S);
  pthread_mutex_unlock(&ekf.mutex);

  // Clear local EKF arrays
  mat_clear(x);
  mat_clear(z);
  mat_clear(f);
  mat_clear(h);
  //mat_clear(xp);
  //mat_clear(xd);
  //mat_clear(fp);
  //mat_clear(fd);

  // Clear local EKF matrices
  mat_clear(F);
  mat_clear(H);
  mat_clear(Q);
  mat_clear(R);
  mat_clear(P);
  mat_clear(T);
  mat_clear(S);
  mat_clear(K);

  // Clear transpose matrices
  mat_clear(Ft);
  mat_clear(Ht);

  // Clear temp storage matrices
  mat_clear(tmpNN);
  mat_clear(tmpNM);
  mat_clear(tmpMN);
  mat_clear(tmpN);
  mat_clear(tmpM);

  return;
}
*/

/**
 *
 */
/*
void ekf_gain ( void )  {

  // Define local dimension sizes
  ushort n = EKF_N;
  ushort m = EKF_M;

  // Initialize local matrices
  matrix *H   = mat_init(m,n);
  matrix *Ht  = mat_init(n,m);
  matrix *T   = mat_init(n,n);
  matrix *S   = mat_init(m,m);
  matrix *K   = mat_init(n,m);
  matrix *tmp = mat_init(n,m);
  matrix *inv = mat_init(m,m);

  // Obtain EKF values
  pthread_mutex_lock(&ekf.mutex);
  H = mat_copy(ekf.H);
  T = mat_copy(ekf.T);
  S = mat_copy(ekf.S);
  pthread_mutex_unlock(&ekf.mutex);

  // K = T Ht S^{-1}
  Ht = mat_trans(H);
  tmp = mat_mul( T, Ht );
  inv = mat_inv(S);
  K = mat_mul( tmp, inv );

  // Push data to structure
  pthread_mutex_lock(&ekf.mutex);
  ekf.K = mat_copy(K);
  pthread_mutex_unlock(&ekf.mutex);

  // Clear local matrices
  mat_clear(H);
  mat_clear(Ht);
  mat_clear(T);
  mat_clear(S);
  mat_clear(K);
  mat_clear(tmp);
  mat_clear(inv);

  return;
}
*/


