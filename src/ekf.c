

#include "ekf.h"
//#include <math.h>
//#include <pthread.h>
#include <stdio.h>
//#include <stdlib.h>
#include "ahrs.h"
#include "imu.h"
#include "sys.h"
#include "timer.h"


//static int  choldc1   ( double *a, double *p, int n );
//static int  choldcsl  ( double *A, double *a, double *p, int n );
//static int  cholsl    ( double *A, double *a, double *p, int n );
//static void mulmat    ( double *a, double *b, double *c, int arows, int acols, int bcols );
//static void mulvec    ( double *a, double *x, double *y, int m, int n );
//static void transpose ( double *a, double *at, int m, int n );
//static void accum     ( double *a, double *b, int m, int n );
//static void add       ( double *a, double *b, double *c, int n );
//static void sub       ( double *a, double *b, double *c, int n );
//static void negate    ( double *a, int m, int n );
//static void addeye    ( double *a, int n );


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

  // Local variables
  uint n  = EKF_N;
  uint m  = EKF_M;

  // Allocate memory for storage arrays
  ekf.x = mat_init(n,1);
  ekf.z = mat_init(m,1);
  ekf.f = mat_init(n,1);
  ekf.h = mat_init(m,1);
  ekf.F = mat_init(n,n);
  ekf.H = mat_init(n,m);
  ekf.Q = mat_init(n,n);
  ekf.R = mat_init(m,m);
  ekf.P = mat_init(n,n);
  ekf.S = mat_init(m,m);
  ekf.K = mat_init(n,m);

  // Assign plant covariance values (WIP: pull from file with error checking)
  mat_set( ekf.Q, 1, 1, 0.01 );
  mat_set( ekf.Q, 2, 2, 0.01 );

  // Assign measurement covariance values (WIP: pull from file with error checking)
  mat_set( ekf.R, 1, 1, 0.01 );
  mat_set( ekf.R, 2, 2, 0.01 );

  // Static F matrix    //--- TODO: Adaptively updated from MRAC or L1 ---//
  double dt  = 1.0 / HZ_EKF;
  mat_set( ekf.F, 1, 1, 1.0 );  mat_set( ekf.F, 1, 2, dt );
  mat_set( ekf.F, 2, 1, 0.0 );  mat_set( ekf.F, 2, 2, 1.0 );
  
  // Static H matrix   //--- TODO: Adaptively updated from MRAC or L1 ---//
  mat_set( ekf.H, 1, 1, 1.0 );  mat_set( ekf.H, 1, 2, 0.0 );
  mat_set( ekf.H, 2, 1, 0.0 );  mat_set( ekf.H, 2, 2, 1.0 );

  // P matrix
  mat_set( ekf.P, 1, 1, 1.0 );  mat_set( ekf.P, 1, 2, 1.0 );
  mat_set( ekf.P, 2, 1, 0.0 );  mat_set( ekf.P, 2, 2, 0.0 );

  // Display EKF settings
  if (DEBUG) {
    printf("Q  ");  mat_print(ekf.Q);
    printf("R  ");  mat_print(ekf.R);
    printf("F  ");  mat_print(ekf.F);
    printf("H  ");  mat_print(ekf.H);
    printf("P  ");  mat_print(ekf.P);
  }

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
int ekf_update ( void )  {

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
  matrix *Ht  = mat_init(m,n);
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
  mat_set( x, 1, 1, 0.1 );
  mat_set( x, 2, 1, 0.3 );

  // Populate measurement vector
  mat_set( z, 1, 1, 0.11 );
  mat_set( z, 2, 1, 0.29 );

  // Evaluate derivatives
  f = mat_mul( F, x );
  h = mat_mul( H, x );
  mat_print(f);
  mat_print(h);

  // Transpose matrices
  Ft = mat_trans(F);
  Ht = mat_trans(H);

  // Pt = F P Ft + Q
  tmpNN = mat_mul( F, P );
  Pt = mat_mul( tmpNN, Ft );
  Pt = mat_add( Pt, Q );
  mat_print(Pt);

  // S = H Pt Ht + R
  tmpMN = mat_mul( H, Pt );
  S = mat_mul( tmpMN, Ht );
  S = mat_add( S, R );
  mat_print(S);

  // K = Pt Ht S^{-1}
  tmpNM = mat_mul( Pt, Ht );
  Inv = mat_inv(S);
  K = mat_mul( tmpNM, Inv );
  mat_print(K);

  // x = f + K ( z - h )
  tmpM = mat_sub( z, h );
  tmpN = mat_mul( K, tmpM );
  x = mat_add( f, tmpN );
  mat_print(x);

  // P = ( I - K H ) Pt
  tmpNN = mat_mul( K, H );
  tmpNN = mat_sub( mat_eye(n), tmpNN );
  P = mat_mul( tmpNN, Pt );

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

  return 0;
}


/**
 * Cholesky-decomposition matrix-inversion code, adapated from
 * http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt
 */
/*
static int choldc1 ( double *a, double *p, int n )  {

  // Local variables
  int i, j, k;
  double sum;

  for ( i=0; i<n; i++ )  {
    for ( j=i; j<n; j++ )  {
      sum = a[i*n+j];
      for ( k=i-1; k>=0; k-- )  sum -= a[i*n+k] * a[j*n+k];
      if (i==j)  {
        if (sum<=0)  return -1;
        p[i] = sqrt(sum);
      }
      else  a[j*n+i] = sum / p[i];
    }
  }

  return 0;
}
*/

/**
 * 
 */
/*
static int choldcsl ( double *A, double *a, double *p, int n )  {

  // Local variables
  int i, j, k;
  double sum;

  // Assign A to a
  for ( i=0; i<n; i++ )  {
    for ( j=0; j<n; j++ )  {
      a[i*n+j] = A[i*n+j];
    }
  }

  // Check error condition
  if ( choldc1( a, p, n ) )  return -1;

  // Apply algorithm
  for ( i=0; i<n; i++ )  {
    a[i*n+i] = 1 / p[i];
    for ( j=i+1; j<n; j++ )  {
      sum = 0;
      for ( k=i; k<j; k++ )  sum -= a[j*n+k] * a[k*n+i];
      a[j*n+i] = sum / p[j];
    }
  }

  return 0;    
}
*/

/**
 *
 */
/*
static int cholsl ( double *A, double *a, double *p, int n )  {

  // Local variables
  int i, j, k;

  // Check error condition
  if ( choldcsl( A, a, p, n ) )  return -1;

  // Zeros for upper triangular elements
  for ( i=0; i<n; i++ )  {
    for ( j=i+1; j<n; j++ )  {
      a[i*n+j] = 0.0;
    }
  }

  for ( i=0; i<n; i++ )  {

    // Square diagonal elements
    a[i*n+i] *= a[i*n+i];

    // Sum square of non diagonal entries
    for ( k=i+1; k<n; k++ )  a[i*n+i] += a[k*n+i] * a[k*n+i];

    // Populate lower diagonal elements 
    for ( j=i+1; j<n; j++ )  {
      for ( k=j; k<n; k++ )  {
        a[i*n+j] += a[k*n+i] * a[k*n+j];
      }
    }

  }

  // Pseudo transpose
  for ( i=0; i<n; i++ )  {
    for ( j=0; j<i; j++ )  {
      a[i*n+j] = a[j*n+i];
    }
  }

  return 0;
}
*/

/**
 *
 */
// C <- A * B
/*
static void mulmat ( double *a, double *b, double *c, int arows, int acols, int bcols )  {
  int i, j, l;
  for( i=0; i<arows; ++i )  {
    for( j=0; j<bcols; ++j )  {
      c[i*bcols+j] = 0;
      for( l=0; l<acols; ++l )  c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j];
    }
  }
  return;
}
*/

/**
 *
 */
/*
static void mulvec ( double *a, double *x, double *y, int m, int n )  {
  int i, j;
  for( i=0; i<m; ++i )  {
    y[i] = 0;
    for( j=0; j<n; ++j )  y[i] += x[j] * a[i*n+j];
  }
  return;
}
*/

/**
 *
 */
/*
static void transpose ( double *a, double *at, int m, int n )  {
  int i, j;
  for( i=0; i<m; ++i )  {
    for( j=0; j<n; ++j )  {
      at[j*m+i] = a[i*n+j];
    }
  }
  return;
}
*/

/**
 *
 */
// A <- A + B
/*
static void accum ( double *a, double *b, int m, int n )  {
  int i, j;
  for( i=0; i<m; ++i )  {
    for( j=0; j<n; ++j )  {
      a[i*n+j] += b[i*n+j];
    }
  }
  return;
}
*/

/**
 *
 */
// C <- A + B
/*
static void add ( double *a, double *b, double *c, int n )  {
  int j;
  for( j=0; j<n; ++j )  c[j] = a[j] + b[j];
  return;
}
*/

/**
 *
 */
// C <- A - B
/*
static void sub ( double *a, double *b, double *c, int n )  {
  int j;
  for( j=0; j<n; ++j )  c[j] = a[j] - b[j];
  return;
}
*/

/**
 *
 */
/*
static void negate ( double *a, int m, int n )  {        
  int i, j;
  for( i=0; i<m; ++i )  {
    for( j=0; j<n; ++j )  {
      a[i*n+j] = -a[i*n+j];
    }
  }
  return;
}
*/

/**
 *
 */
/*
static void addeye ( double *a, int n )  {
  int i;
  for( i=0; i<n; ++i )  a[i*n+i] += 1;
  return;
}
*/


