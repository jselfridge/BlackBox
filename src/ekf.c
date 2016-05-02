

#include "ekf.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
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
  int nm = n*m;
  int mm = m*m;
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
  ekf.K = malloc( sizeof(double) * nm );

  // Zero out initial values
  for ( i=0; i<n;  i++ )  ekf.x[i] = 0.0;
  for ( i=0; i<m;  i++ )  ekf.z[i] = 0.0;
  for ( i=0; i<n;  i++ )  ekf.f[i] = 0.0;
  for ( i=0; i<n;  i++ )  ekf.h[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.F[i] = 0.0;
  for ( i=0; i<nm; i++ )  ekf.H[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.Q[i] = 0.0;
  for ( i=0; i<mm; i++ )  ekf.R[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.P[i] = 0.0;
  for ( i=0; i<nm; i++ )  ekf.K[i] = 0.0;

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Debugging values

  ekf.x[0] =  1.7;    ekf.x[1] =  1.3;
  ekf.z[0] =  2.7;    ekf.z[1] =  4.7;    ekf.z[2] =  2.1;

  ekf.f[0] =  3.2;    ekf.f[1] =  9.1;
  ekf.h[0] = -4.3;    ekf.h[1] = -2.2;    ekf.h[2] =  3.8;

  ekf.F[0] =  2.5;    ekf.F[1] =  3.4;
  ekf.F[2] =  7.2;    ekf.F[3] =  1.4;

  ekf.H[0] =  4.3;    ekf.H[1] =  5.1;
  ekf.H[2] =  2.0;    ekf.H[3] =  2.6;
  ekf.H[4] =  2.5;    ekf.H[5] =  2.4;

  ekf.Q[0] = -1.3;    ekf.Q[1] =  0.2;
  ekf.Q[2] =  0.2;    ekf.Q[3] =  1.9;

  ekf.R[0] =  2.4;    ekf.R[1] =  0.3;    ekf.R[2] =  1.5;
  ekf.R[3] =  0.3;    ekf.R[4] =  6.2;    ekf.R[5] =  0.0;
  ekf.R[6] =  1.5;    ekf.R[7] =  0.0;    ekf.R[8] =  3.7;

  ekf.P[0] =  5.2;    ekf.P[1] =  3.1;
  ekf.P[2] =  3.1;    ekf.P[3] =  8.2;

  return;
}


/**
 * Exits the EKF routine
 */
void ekf_exit ( void )  {
  if(DEBUG)  printf( "Close EKF \n" );  
  free(ekf.x);
  free(ekf.z);
  free(ekf.f);
  free(ekf.h);
  free(ekf.F);
  free(ekf.H);
  free(ekf.Q);
  free(ekf.R);
  free(ekf.P);
  free(ekf.K);
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

  // Local variables
  int n  = EKF_N;
  int m  = EKF_M;
  int nn = n*n;
  int nm = n*m;
  int mm = m*m;
  int i;

  // Pull data from structure
  pthread_mutex_lock(&mutex_ekf);
  double *x = ekf.x;
  double *z = ekf.z;
  double *h = ekf.h;
  double *f = ekf.f;
  double *F = ekf.F;
  double *H = ekf.H;
  double *Q = ekf.Q;
  double *R = ekf.R;
  double *P = ekf.P;
  double *K = ekf.K;
  pthread_mutex_unlock(&mutex_ekf);

  // Debugging statements
  printf("\n");
  printf("x: ");  for ( i=0; i<n;  i++ )  printf( "%4.1f ", x[i] );  printf("\n");
  printf("z: ");  for ( i=0; i<m;  i++ )  printf( "%4.1f ", z[i] );  printf("\n");
  printf("f: ");  for ( i=0; i<n;  i++ )  printf( "%4.1f ", f[i] );  printf("\n");
  printf("h: ");  for ( i=0; i<m;  i++ )  printf( "%4.1f ", h[i] );  printf("\n");
  printf("F: ");  for ( i=0; i<nn; i++ )  printf( "%4.1f ", F[i] );  printf("\n");
  printf("H: ");  for ( i=0; i<nm; i++ )  printf( "%4.1f ", H[i] );  printf("\n");
  printf("Q: ");  for ( i=0; i<nn; i++ )  printf( "%4.1f ", Q[i] );  printf("\n");
  printf("R: ");  for ( i=0; i<mm; i++ )  printf( "%4.1f ", R[i] );  printf("\n");
  printf("P: ");  for ( i=0; i<nn; i++ )  printf( "%4.1f ", P[i] );  printf("\n");

  // Define intermediate storage arrays
  double Ft[nn], Ht[nm], Pp[nn];

  // Define temp storage arrays
  double tmpNN[nn], tmpNM[nm], tmpMN[nm], tmpMM[mm], tmpinv[mm], tmpN[n], tmpM[m];

  // P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}
  mulmat( F, P, tmpNN, n, n, n );
  transpose( F, Ft, n, n );
  mulmat( tmpNN, Ft, Pp, n, n, n );
  accum( Pp, Q, n, n );
  printf("\n\nPp: ");  for ( i=0; i<nn; i++ )  printf( "%f ", Pp[i] );  printf("\n");

  // K_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1}
  transpose( H, Ht, m, n );
  mulmat( Pp, Ht, tmpNM, n, n, m );
  mulmat( H, Pp, tmpMN, m, n, n );
  mulmat( tmpMN, Ht, tmpMM, m, n, m );
  accum( tmpMM, R, m, m );
  printf("S: ");  for ( i=0; i<mm; i++ )  printf( "%f ", tmpMM[i]  );  printf("\n");
  if ( cholsl( tmpMM, tmpinv, tmpM, m ) )  return -1;
  mulmat( tmpNM, tmpinv, K, n, m, m );
  printf("I: ");  for ( i=0; i<mm; i++ )  printf( "%f ", tmpinv[i] );  printf("\n");
  printf("K: ");  for ( i=0; i<nm; i++ )  printf( "%f ", K[i]      );  printf("\n");

  // \hat{x}_k = \hat{f}_k + K_k ( z_k - h(\hat{x}_k) )
  sub( z, h, tmpM, m );
  mulvec( K, tmpM, tmpN, n, m );
  add( f, tmpN, x, n );
  printf("x: ");  for ( i=0; i<n; i++ )  printf( "%f ", x[i] );  printf("\n");

  // P_k = (I - K_k H_k) P_k
  mulmat( K, H, tmpNN, n, m, n );
  negate( tmpNN, n, n );
  addeye( tmpNN, n );
  mulmat( tmpNN, Pp, P, n, n, n );
  printf("P: ");  for ( i=0; i<nn; i++ )  printf( "%f ", P[i] );  printf("\n");

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



