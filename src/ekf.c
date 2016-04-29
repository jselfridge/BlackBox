

#include "ekf.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "timer.h"


//static void unpack    ( void *v, ekf_struct *ekf, int n, int m );
static int  choldc1   ( double *a, double *p, int n );
static int  choldcsl  ( double *A, double *a, double *p, int n );
static int  cholsl    ( double *A, double *a, double *p, int n );
//static void zeros     ( double *a, int m, int n );
static void mulmat    ( double *a, double *b, double *c, int arows, int acols, int bcols );
static void mulvec    ( double *a, double *x, double *y, int m, int n );
static void transpose ( double *a, double *at, int m, int n );
static void accum     ( double *a, double *b, int m, int n );
static void add       ( double *a, double *b, double *c, int n );
static void sub       ( double *a, double *b, double *c, int n );
static void negate    ( double *a, int m, int n );
static void addeye    ( double *a, int n );


/*
static void dump ( double *a, int m, int n, const char *fmt )  {
  int i, j;
  char f[100];
  sprintf( f, "%s ", fmt );
  for( i=0; i<m; ++i )  {
    for( j=0; j<n; ++j )  {
      printf( f, a[i*n+j] );
    }
    printf("\n");
  }
  return;
}
*/


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
      double hx[N];    // output of user defined h() measurement function
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
  ekf.F = malloc( sizeof(double) * nn );
  ekf.H = malloc( sizeof(double) * nm );
  ekf.Q = malloc( sizeof(double) * nn );
  ekf.R = malloc( sizeof(double) * mm );
  ekf.P = malloc( sizeof(double) * nn );
  ekf.K = malloc( sizeof(double) * nm );

  // Zero out initial values
  for ( i=0; i<n;  i++ )  ekf.x[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.F[i] = 0.0;
  for ( i=0; i<nm; i++ )  ekf.H[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.Q[i] = 0.0;
  for ( i=0; i<mm; i++ )  ekf.R[i] = 0.0;
  for ( i=0; i<nn; i++ )  ekf.P[i] = 0.0;
  for ( i=0; i<nm; i++ )  ekf.K[i] = 0.0;

  return;
}


/**
 * Exits the EKF routine
 */
void ekf_exit ( void )  {
  if(DEBUG)  printf( "Close EKF \n" );  
  free(ekf.x);
  free(ekf.F);
  free(ekf.H);
  free(ekf.Q);
  free(ekf.R);
  free(ekf.P);
  free(ekf.K);
  return;
}


/**
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
  //int nn = n*n;
  //int nm = n*m;
  //int mm = m*m;
  //int i;

  // Local storage arrays

  // Pull data from structure
  pthread_mutex_lock(&mutex_ekf);
  double *x = ekf.x;
  double *F = ekf.F;
  double *H = ekf.H;
  double *Q = ekf.Q;
  double *R = ekf.R;
  double *P = ekf.P;
  double *K = ekf.K;
  pthread_mutex_unlock(&mutex_ekf);

  // Define intermediate storage arrays
  double Ft, Ht, fx, hx, Pp, z;

  // Define temp storage arrays
  double tmp1, tmp2, tmp3, tmp4, tmp5;

  // P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}
  mulmat( F, P, &tmp1, n, n, n );
  transpose( F, &Ft, n, n );
  mulmat( &tmp1, &Ft, &Pp, n, n, n );
  accum( &Pp, Q, n, n );

  // G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1}
  transpose( H, &Ht, m, n );
  mulmat( &Pp, &Ht, &tmp1, n, n, m );
  mulmat( H, &Pp, &tmp2, m, n, n );
  mulmat( &tmp2, &Ht, &tmp3, m, n, m );
  accum( &tmp3, R, m, m );
  if ( cholsl( &tmp3, &tmp4, &tmp5, m ) )  return 1;
  mulmat( &tmp1, &tmp4, K, n, m, m );

  // \hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k))
  sub( &z, &hx, &tmp5, m );
  mulvec( K, &tmp5, &tmp2, n, m );
  add( &fx, &tmp2, x, n );

  // P_k = (I - G_k H_k) P_k
  mulmat( K, H, &tmp1, n, m, n );
  negate( &tmp1, n, n );
  addeye( &tmp1, n );
  mulmat( &tmp1, &Pp, P, n, n, n );

  return 0;
}


/**
 *
 */
/*
static void unpack ( void *v, ekf_struct *ekf, int n, int m )  {

  // Skip over n, m in data structure
  // NOTE: Find a better way...
  char *cptr = (char *)v;
  cptr += 2*sizeof(int);

  double *dptr = (double *)cptr;
  ekf->x = dptr;
  dptr += n;
  ekf->P = dptr;
  dptr += n*n;
  ekf->Q = dptr;
  dptr += n*n;
  ekf->R = dptr;
  dptr += m*m;
  ekf->G = dptr;
  dptr += n*m;
  ekf->F = dptr;
  dptr += n*n;
  ekf->H = dptr;
  dptr += m*n;
  ekf->Ht = dptr;
  dptr += n*m;
  ekf->Ft = dptr;
  dptr += n*n;
  ekf->Pp = dptr;
  dptr += n*n;
  ekf->fx = dptr;
  dptr += n;
  ekf->hx = dptr;
  dptr += m;
  ekf->tmp1 = dptr;
  dptr += n*m;
  ekf->tmp2 = dptr;
  dptr += m*n;
  ekf->tmp3 = dptr;
  dptr += m*m;
  ekf->tmp4 = dptr;
  dptr += m*m;
  ekf->tmp5 = dptr;

  return;
}
*/

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
/*static void zeros ( double *a, int m, int n )  {
  int j;
  for ( j=0; j<m*n; ++j )  a[j] = 0;
}
*/

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



