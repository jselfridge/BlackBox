

#include "ekf.h"
#include <stdio.h>
//#include "ahrs.h"
//#include "imu.h"
#include "sys.h"
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
void ekf_init ( void )  {
  if (DEBUG)  printf( "Initializing EKF \n" );
  /*
  // Local variables
  uint n  = EKF_N;
  uint m  = EKF_M;

  // Allocate memory for storage arrays
  ekf.x = mat_init(n,1);
  ekf.z = mat_init(m,1);
  ekf.f = mat_init(n,1);
  ekf.h = mat_init(m,1);
  ekf.F = mat_init(n,n);
  ekf.H = mat_init(m,n);
  ekf.Q = mat_init(n,n);
  ekf.R = mat_init(m,m);
  ekf.P = mat_init(n,n);
  ekf.S = mat_init(m,m);
  ekf.K = mat_init(n,m);

  // Assign plant covariance values
  mat_set( ekf.Q, 1, 1, 0.01 );
  mat_set( ekf.Q, 2, 2, 0.01 );
  mat_set( ekf.Q, 3, 3, 0.01 );

  // Assign measurement covariance values
  mat_set( ekf.R, 1, 1, 0.01 );
  mat_set( ekf.R, 2, 2, 0.01 );
  mat_set( ekf.R, 3, 3, 0.01 );
  //mat_set( ekf.R, 4, 4, 0.01 );

  // Initialize prediction matrix
  mat_set( ekf.P, 1, 1, 0.01 );
  mat_set( ekf.P, 2, 2, 0.01 );
  mat_set( ekf.P, 3, 3, 0.01 );

  // Static F matrix    //--- TODO: Adaptively updated from MRAC or L1 ---//
  double dt  = 1.0 / HZ_EKF;
  mat_set( ekf.F, 1, 1, 1.0 );  mat_set( ekf.F, 1, 2, dt  );  mat_set( ekf.F, 1, 3, (dt*dt)/2.0 );
  mat_set( ekf.F, 2, 1, 0.0 );  mat_set( ekf.F, 2, 2, 1.0 );  mat_set( ekf.F, 2, 3, dt  );
  mat_set( ekf.F, 3, 1, 0.0 );  mat_set( ekf.F, 3, 2, 0.0 );  mat_set( ekf.F, 3, 3, 1.0  );
  
  // Static H matrix   //--- TODO: Adaptively updated from MRAC or L1 ---//
  mat_set( ekf.H, 1, 1, 1.0 );  mat_set( ekf.H, 1, 2, 0.0 );  mat_set( ekf.H, 1, 3, 0.0 );
  mat_set( ekf.H, 2, 1, 0.0 );  mat_set( ekf.H, 2, 2, 1.0 );  mat_set( ekf.H, 2, 3, 0.0 );
  mat_set( ekf.H, 3, 1, 0.0 );  mat_set( ekf.H, 3, 2, 0.0 );  mat_set( ekf.H, 3, 3, 1.0 );
  //mat_set( ekf.H, 4, 1, 0.0 );  mat_set( ekf.H, 4, 2, 1.0 );

  // Display EKF settings
  if (DEBUG) {
    printf("Q  ");  mat_print(ekf.Q);
    printf("R  ");  mat_print(ekf.R);
    printf("P  ");  mat_print(ekf.P);
    printf("F  ");  mat_print(ekf.F);
    printf("H  ");  mat_print(ekf.H);
  }
  */
  return;
}


/**
 * Exits the EKF routine
 */
void ekf_exit ( void )  {
  if(DEBUG)  printf( "Close EKF \n" );  
  // Add code as needed...
  return;
}


/**
 *
 * Runs one step of EKF prediction and update. Your code should first build
 * a model, setting the contents of <tt>ekf.fx</tt>, <tt>ekf.F</tt>,
 * <tt>ekf.hx</tt>, and <tt>ekf.H</tt> to appropriate values.
 * @param ekf pointer to structure EKF
 * @param z array of measurement (observation) values
 * @return 0 on success, -1 on failure caused by non-positive-definite matrix.
 */
void ekf_update ( void )  {

  /*
  uint debug = 0;

  // Define local dimension sizes and counters
  uint n  = EKF_N;
  uint m  = EKF_M;

  // Initialize local EKF arrays
  matrix *x = mat_init(n,1);
  matrix *z = mat_init(m,1);
  matrix *f = mat_init(n,1);
  matrix *h = mat_init(m,1);

  // Initialize local EKF matrices
  matrix *F = mat_init(n,n);
  matrix *H = mat_init(m,n);
  matrix *Q = mat_init(n,n);
  matrix *R = mat_init(m,m);
  matrix *P = mat_init(n,n);
  matrix *S = mat_init(m,m);
  matrix *K = mat_init(n,m);

  // Initialize intermediate storage arrays
  matrix *Ft  = mat_init(n,n);
  matrix *Ht  = mat_init(n,m);
  matrix *Pt  = mat_init(n,n);
  matrix *Inv = mat_init(m,m);

  // Initialize temp storage arrays
  matrix *tmpNN = mat_init(n,n);
  matrix *tmpNM = mat_init(n,m);
  matrix *tmpMN = mat_init(m,n);
  matrix *tmpN  = mat_init(n,1);
  matrix *tmpM  = mat_init(m,1);

  // Obtain EKF values
  pthread_mutex_lock(&ekf.mutex);
  x = mat_copy(ekf.x);
  F = mat_copy(ekf.F);
  H = mat_copy(ekf.H);
  P = mat_copy(ekf.P);
  Q = mat_copy(ekf.Q);
  R = mat_copy(ekf.R);
  pthread_mutex_unlock(&ekf.mutex);

  // Obtain comp filter A attitude
  //pthread_mutex_lock(&imuA.mutex);
  //double R_compA = imuA.roll;
  //double P_compA = imuA.pitch;
  //pthread_mutex_unlock(&imuA.mutex);

  // Obtain comp filter B attitude
  //pthread_mutex_lock(&imuB.mutex);
  //double R_compB = imuB.roll;
  //double P_compB = imuB.pitch;
  //pthread_mutex_unlock(&imuB.mutex);

  // Obtain rate gyro A 
  //pthread_mutex_lock(&gyrA.mutex);
  //double dR_gyrA = gyrA.filter[0];
  //double dP_gyrA = gyrA.filter[1];
  //double dY_gyrA = gyrA.filter[2];
  //pthread_mutex_unlock(&gyrA.mutex);

  // Obtain rate gyro B 
  //pthread_mutex_lock(&gyrB.mutex);
  //double dR_gyrB = gyrB.filter[0];
  //double dP_gyrB = gyrB.filter[1];
  //double dY_gyrB = gyrB.filter[2];
  //pthread_mutex_unlock(&gyrB.mutex);

  // Obtain AHRSA measurements
  //pthread_mutex_lock(&ahrsA.mutex);
  //z[ 0] = ahrsA.eul[0];  z[ 2] = ahrsA.deul[0];
  //z[ 2] = ahrsA.eul[1];  z[ 8] = ahrsA.deul[1];
  //z[ 4] = ahrsA.eul[2];  z[10] = ahrsA.deul[2];
  //pthread_mutex_unlock(&ahrsA.mutex);

  // Obtain AHRSB measurements
  //pthread_mutex_lock(&ahrsB.mutex);
  //z[ 1] = ahrsB.eul[0];  z[ 3] = ahrsB.deul[0];
  //z[ 3] = ahrsB.eul[1];  z[ 9] = ahrsB.deul[1];
  //z[ 5] = ahrsB.eul[2];  z[11] = ahrsB.deul[2];
  //pthread_mutex_unlock(&ahrsB.mutex);

  // Debugging vector
  mat_set( x, 1, 1, 0.0 );
  mat_set( x, 2, 1, 0.0 );
  mat_set( x, 3, 1, 0.0 );

  // Populate measurement vector
  mat_set( z, 1, 1,  0.0 );
  mat_set( z, 2, 1,  0.0 );
  mat_set( z, 3, 1,  0.0 );
  //mat_set( z, 4, 1, -0.001 );

  // Evaluate derivatives
  f = mat_mul( F, x );
  h = mat_mul( H, x );
  if(debug) mat_print(f);
  if(debug) mat_print(h);

  // Transpose matrices
  Ft = mat_trans(F);
  Ht = mat_trans(H);
  if(debug) mat_print(Ft);
  if(debug) mat_print(Ht);

  // Pt = F P Ft + Q
  tmpNN = mat_mul( F, P );
  Pt = mat_mul( tmpNN, Ft );
  Pt = mat_add( Pt, Q );
  mat_sym( Pt, 0.0001 );
  if(debug) mat_print(Pt);

  // S = H Pt Ht + R
  tmpMN = mat_mul( H, Pt );
  S = mat_mul( tmpMN, Ht );
  S = mat_add( S, R );
  mat_sym( S, 0.0001 );
  if(debug) mat_print(S);

  // K = Pt Ht S^{-1}    //--- ORIG ---//
  tmpNM = mat_mul( Pt, Ht );
  //Inv = mat_inv(S);
  //K = mat_mul( tmpNM, Inv );
  K = mat_mul( tmpNM, S );
  if(debug) mat_print(K);

  // K = Pt Ht S^{-1}  ==>>  K^T = (S^T)^(-1) H Pt^T
  //tmpMN = mat_mul( H, mat_trans(Pt) );
  //matrix *Kt = mat_init(m,n);
  //Kt = mat_divL( mat_trans(S), tmpMN );
  //K = mat_trans( Kt );
  //mat_clear(Kt);
  //if(debug) mat_print(K);

  // x = f + K ( z - h )
  tmpM = mat_sub( z, h );
  tmpN = mat_mul( K, tmpM );
  x = mat_add( f, tmpN );
  if(debug) mat_print(x);

  // P = ( I - K H ) Pt
  tmpNN = mat_mul( K, H );
  tmpNN = mat_sub( mat_eye(n), tmpNN );
  P = mat_mul( tmpNN, Pt );
  mat_sym( P, 0.0001 );
  if(debug) mat_print(P);

  // Push data to structure
  pthread_mutex_lock(&ekf.mutex);
  ekf.x = mat_copy(x);
  ekf.z = mat_copy(z);
  ekf.f = mat_copy(f);
  ekf.h = mat_copy(h);
  ekf.P = mat_copy(P);
  ekf.S = mat_copy(S);
  ekf.K = mat_copy(K);
  pthread_mutex_unlock(&ekf.mutex);

  // Clear local EKF arrays
  mat_clear(x);
  mat_clear(z);
  mat_clear(f);
  mat_clear(h);

  // Clear local EKF matrices
  mat_clear(F);
  mat_clear(H);
  mat_clear(Q);
  mat_clear(R);
  mat_clear(P);
  mat_clear(S);
  mat_clear(K);

  // Clear intermediate storage arrays
  mat_clear(Ft);
  mat_clear(Ht);
  mat_clear(Pt);
  mat_clear(Inv);

  // Clear temp storage arrays
  mat_clear(tmpNN);
  mat_clear(tmpNM);
  mat_clear(tmpMN);
  mat_clear(tmpN);
  mat_clear(tmpM);
  */

  return;
}


/**
 *
 */
void ekf_gain ( void )  {

  return;
}



