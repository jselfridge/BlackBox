

#include "ekf.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "imu.h"
#include "sys.h"
#include "timer.h"


static int  choldc1   ( double *a, double *p, int n );
static int  choldcsl  ( double *A, double *a, double *p, int n );
static int  cholsl    ( double *A, double *a, double *p, int n );
static void mulmat    ( double *a, double *b, double *c, int arows, int acols, int bcols );
static void mulvec    ( double *a, double *x, double *y, int m, int n );
static void transpose ( double *a, double *at, int m, int n );
static void accum     ( double *a, double *b, int m, int n );
static void add       ( double *a, double *b, double *c, int n );
static void sub       ( double *a, double *b, double *c, int n );
static void negate    ( double *a, int m, int n );
static void addeye    ( double *a, int n );


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
      double tmp1[N][N];
      double tmp2[M][N];
      double tmp3[M][M];
      double tmp4[M][M];
      double tmp5[M];
 * </pre>
 */
void ekf_init ( void )  {
  if (DEBUG)  printf( "Initializing EKF \n" );

  // Local variables
  int n  = EKF_N;
  int m  = EKF_M;
  int nn = n*n;
  int mm = m*m;
  int nm = n*m;
  int i;

  // Allocate memory for storage arrays
  ekf.x = malloc( sizeof(double) * n  );
  ekf.z = malloc( sizeof(double) * m  );
  ekf.f = malloc( sizeof(double) * n  );
  ekf.h = malloc( sizeof(double) * m  );
  ekf.F = malloc( sizeof(double) * nn );
  ekf.H = malloc( sizeof(double) * nm );
  ekf.Q = malloc( sizeof(double) * nn );
  ekf.R = malloc( sizeof(double) * mm );
  ekf.P = malloc( sizeof(double) * nn );
  ekf.S = malloc( sizeof(double) * mm );
  ekf.K = malloc( sizeof(double) * nm );

  // Zero out initial values
  for ( i=0; i<n;  i++ )  ekf.x[i] = 0.0;
  for ( i=0; i<m;  i++ )  ekf.z[i] = 0.0;
  for ( i=0; i<n;  i++ )  ekf.f[i] = 0.0;
  for ( i=0; i<m;  i++ )  ekf.h[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.F[i] = 0.0;
  for ( i=0; i<nm; i++ )  ekf.H[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.Q[i] = 0.0;
  for ( i=0; i<mm; i++ )  ekf.R[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.P[i] = 0.0;
  for ( i=0; i<mm; i++ )  ekf.S[i] = 0.0;
  for ( i=0; i<nm; i++ )  ekf.K[i] = 0.0;

  // Assign plant covariance values (WIP: pull from file with error checking)
  ekf.Q[ 0] = 0.01;    // R angle
  ekf.Q[10] = 0.01;    // P angle
  ekf.Q[20] = 0.01;    // Y angle
  ekf.Q[30] = 0.01;    // R rate
  ekf.Q[40] = 0.01;    // P rate
  ekf.Q[50] = 0.01;    // Y rate
  ekf.Q[60] = 0.01;    // R acc
  ekf.Q[70] = 0.01;    // P acc
  ekf.Q[80] = 0.01;    // Y acc

  // Assign measurement covariance values (WIP: pull from file with error checking)
  ekf.R[0]  = 0.01;    // gyro x
  ekf.R[4]  = 0.01;    // gyro y
  ekf.R[8]  = 0.01;    // gyro z

  // Static F matrix
  // WIP: Adaptively updated from MRAC or L1
  // WIP: Add acceleration state?
  // Unity diagonal, time step block diagonal in upper right
  double dt  = 1.0 / HZ_EKF;
  double dt2 = dt * dt / 2;
  ekf.F[ 0] = 1.0;  ekf.F[ 3] = dt;  ekf.F[ 6] = dt2;
  ekf.F[10] = 1.0;  ekf.F[13] = dt;  ekf.F[16] = dt2;
  ekf.F[20] = 1.0;  ekf.F[23] = dt;  ekf.F[26] = dt2;
  ekf.F[30] = 1.0;  ekf.F[33] = dt;
  ekf.F[40] = 1.0;  ekf.F[43] = dt;
  ekf.F[50] = 1.0;  ekf.F[53] = dt;
  ekf.F[60] = 1.0;
  ekf.F[70] = 1.0;
  ekf.F[80] = 1.0;

  // Static H matrix (WIP: adaptively updated from MRAC or L1)
  ekf.H[ 3] = 1.0;
  ekf.H[13] = 1.0;
  ekf.H[23] = 1.0;

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
  uint n, m, nn, nm, mm, i;
  n  = EKF_N;
  m  = EKF_M;
  nn = n*n;
  mm = m*m;
  nm = n*m;

  // Define local EKF arrays
  double x[n], z[m], f[n], h[m];

  // Define local EKF matrices
  double F[nn], H[nm], Q[nn], R[mm], P[nn], S[mm], K[nm];

  // Define intermediate storage arrays
  double Ft[nn], Ht[nm], Pt[nn], Inv[mm];

  // Define temp storage arrays
  double tmpNN[nn], tmpNM[nm], tmpMN[nm], tmpN[n], tmpM[m];

  // Obtain EKF values
  pthread_mutex_lock(&mutex_ekf);
  for ( i=0; i<n;  i++ )  x[i] = ekf.x[i];
  for ( i=0; i<nn; i++ )  F[i] = ekf.F[i];
  for ( i=0; i<nm; i++ )  H[i] = ekf.H[i];
  for ( i=0; i<nn; i++ )  Q[i] = ekf.Q[i];
  for ( i=0; i<mm; i++ )  R[i] = ekf.R[i];
  pthread_mutex_unlock(&mutex_ekf);

  // Obtain measurements
  pthread_mutex_lock(&mutex_gyrA);
  for ( i=0; i<3; i++ )  z[i] = imuA.gyr->filter[i];
  pthread_mutex_unlock(&mutex_gyrA);

  // Evaluate derivatives
  mulvec( F, x, f, n, n );
  mulvec( H, x, h, m, n );

  // Find transpose matrices
  transpose( F, Ft, n, n );
  transpose( H, Ht, m, n );

  // Pt = F P Ft + Q
  mulmat( F, P, tmpNN, n, n, n );
  mulmat( tmpNN, Ft, Pt, n, n, n );
  accum( Pt, Q, n, n );

  // S = H Pt Ht + R
  mulmat( H, Pt, tmpMN, m, n, n );
  mulmat( tmpMN, Ht, S, m, n, m );
  accum( S, R, m, m );

  // K = Pt Ht S^{-1}
  mulmat( Pt, Ht, tmpNM, n, n, m );
  if ( cholsl( S, Inv, tmpM, m ) )  {  printf("Too bad... \n");  return -1;  }
  mulmat( tmpNM, Inv, K, n, m, m );

  // x = f + K ( z - h )
  sub( z, h, tmpM, m );
  mulvec( K, tmpM, tmpN, n, m );
  add( f, tmpN, x, n );

  // P = ( I - K H ) Pt
  mulmat( K, H, tmpNN, n, m, n );
  negate( tmpNN, n, n );
  addeye( tmpNN, n );
  mulmat( tmpNN, Pt, P, n, n, n );

  // Push data to structure
  pthread_mutex_lock(&mutex_ekf);
  for ( i=0; i<n;  i++ )  ekf.x[i] = x[i];
  for ( i=0; i<m;  i++ )  ekf.z[i] = z[i];
  for ( i=0; i<n;  i++ )  ekf.f[i] = f[i];
  for ( i=0; i<m;  i++ )  ekf.h[i] = h[i];
  for ( i=0; i<nn; i++ )  ekf.P[i] = P[i];
  for ( i=0; i<mm; i++ )  ekf.S[i] = S[i];
  for ( i=0; i<nm; i++ )  ekf.K[i] = K[i];
  pthread_mutex_unlock(&mutex_ekf);

  return 0;
}


/**
 * Cholesky-decomposition matrix-inversion code, adapated from
 * http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt
 */
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


/**
 * 
 */
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


/**
 *
 */
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


/**
 *
 */
// C <- A * B
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


/**
 *
 */
static void mulvec ( double *a, double *x, double *y, int m, int n )  {
  int i, j;
  for( i=0; i<m; ++i )  {
    y[i] = 0;
    for( j=0; j<n; ++j )  y[i] += x[j] * a[i*n+j];
  }
  return;
}


/**
 *
 */
static void transpose ( double *a, double *at, int m, int n )  {
  int i, j;
  for( i=0; i<m; ++i )  {
    for( j=0; j<n; ++j )  {
      at[j*m+i] = a[i*n+j];
    }
  }
  return;
}


/**
 *
 */
// A <- A + B
static void accum ( double *a, double *b, int m, int n )  {
  int i, j;
  for( i=0; i<m; ++i )  {
    for( j=0; j<n; ++j )  {
      a[i*n+j] += b[i*n+j];
    }
  }
  return;
}


/**
 *
 */
// C <- A + B
static void add ( double *a, double *b, double *c, int n )  {
  int j;
  for( j=0; j<n; ++j )  c[j] = a[j] + b[j];
  return;
}


/**
 *
 */
// C <- A - B
static void sub ( double *a, double *b, double *c, int n )  {
  int j;
  for( j=0; j<n; ++j )  c[j] = a[j] - b[j];
  return;
}


/**
 *
 */
static void negate ( double *a, int m, int n )  {        
  int i, j;
  for( i=0; i<m; ++i )  {
    for( j=0; j<n; ++j )  {
      a[i*n+j] = -a[i*n+j];
    }
  }
  return;
}


/**
 *
 */
static void addeye ( double *a, int n )  {
  int i;
  for( i=0; i<n; ++i )  a[i*n+i] += 1;
  return;
}



