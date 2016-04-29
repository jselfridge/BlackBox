

#include "ekf.h"
//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>


//static void unpack ( void *v, ekf_t *ekf, int n, int m );


/**
 * Initializes the EKF structure.
 * @param ekf pointer to and EKF structure to be initialized
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
/*
void ekf_init ( void *v, int n, int m )  {
  if (DEBUG)  printf( "Initializing EKF \n" );

  // Assign dimensions to incoming data structure
  int *ptr = (int *)v;
  *ptr = n;
  ptr++;
  *ptr = m;

  // Unpack rest of incoming structure for initlization
  ekf_t ekf;  // NOTE: Why is this declared here?
  unpack(v, &ekf, n, m);  // NOTE: Do I really need this?

  // Zero out matrices
  zeros( ekf.P, n, n );
  zeros( ekf.Q, n, n );
  zeros( ekf.R, m, m );
  zeros( ekf.G, n, m );
  zeros( ekf.F, n, n );
  zeros( ekf.H, m, n );

  return;
}
*/

/**
 * Exits the EKF routine
 */
/*
void ekf_exit ( void )  {
  if(DEBUG)  printf( "Close EKF \n" );
  // Add code as needed...
  return;
}
*/

/**
 * Runs one step of EKF prediction and update. Your code should first build
 * a model, setting the contents of <tt>ekf.fx</tt>, <tt>ekf.F</tt>,
 * <tt>ekf.hx</tt>, and <tt>ekf.H</tt> to appropriate values.
 * @param ekf pointer to structure EKF
 * @param z array of measurement (observation) values
 * @return 0 on success, 1 on failure caused by non-positive-definite matrix.
 */
/*
int ekf_update ( void *v, double *z )  {

  // Unpack incoming structure
  // NOTE: Look for a better way...
  int *ptr = (int *)v;
  int n = *ptr;
  ptr++;
  int m = *ptr;

  // NOTE: Why unpack again?
  ekf_t ekf;
  unpack( v, &ekf, n, m );

  // P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}
  mulmat( ekf.F, ekf.P, ekf.tmp1, n, n, n );
  transpose( ekf.F, ekf.Ft, n, n );
  mulmat( ekf.tmp1, ekf.Ft, ekf.Pp, n, n, n );
  accum( ekf.Pp, ekf.Q, n, n );

  // G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1}
  transpose( ekf.H, ekf.Ht, m, n );
  mulmat( ekf.Pp, ekf.Ht, ekf.tmp1, n, n, m );
  mulmat( ekf.H, ekf.Pp, ekf.tmp2, m, n, n );
  mulmat( ekf.tmp2, ekf.Ht, ekf.tmp3, m, n, m );
  accum( ekf.tmp3, ekf.R, m, m );
  if ( cholsl( ekf.tmp3, ekf.tmp4, ekf.tmp5, m ) )  return 1;
  mulmat( ekf.tmp1, ekf.tmp4, ekf.G, n, m, m );

  // \hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k))
  sub( z, ekf.hx, ekf.tmp5, m );
  mulvec( ekf.G, ekf.tmp5, ekf.tmp2, n, m );
  add( ekf.fx, ekf.tmp2, ekf.x, n );

  // P_k = (I - G_k H_k) P_k
  mulmat( ekf.G, ekf.H, ekf.tmp1, n, m, n );
  negate( ekf.tmp1, n, n );
  mat_addeye( ekf.tmp1, n );
  mulmat( ekf.tmp1, ekf.Pp, ekf.P, n, n, n );

  return 0;
}
*/

/**
 *
 */
/*
static void unpack ( void *v, ekf_t *ekf, int n, int m )  {

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



//------------------------------------
// START HERE!!!!
//------------------------------------


/* Cholesky-decomposition matrix-inversion code, adapated from
   http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt */
/*static int choldc1(double * a, double * p, int n) {
    int i,j,k;
    double sum;

    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            sum = a[i*n+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= a[i*n+k] * a[j*n+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    return 1; // error
                }
                p[i] = sqrt(sum);
            }
            else {
                a[j*n+i] = sum / p[i];
            }
        }
    }

  return 0; // success
}*/

/*static int choldcsl(double * A, double * a, double * p, int n) 
{
    int i,j,k; double sum;
    for (i = 0; i < n; i++) 
        for (j = 0; j < n; j++) 
            a[i*n+j] = A[i*n+j];
    if (choldc1(a, p, n)) return 1;
    for (i = 0; i < n; i++) {
        a[i*n+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*n+k] * a[k*n+i];
            }
            a[j*n+i] = sum / p[j];
        }
    }

    return 0; // success
    
}*/

/*static int cholsl(double * A, double * a, double * p, int n) 
{
    int i,j,k;
    if (choldcsl(A,a,p,n)) return 1;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            a[i*n+j] = 0.0;
        }
    }
    for (i = 0; i < n; i++) {
        a[i*n+i] *= a[i*n+i];
        for (k = i + 1; k < n; k++) {
            a[i*n+i] += a[k*n+i] * a[k*n+i];
        }
        for (j = i + 1; j < n; j++) {
            for (k = j; k < n; k++) {
                a[i*n+j] += a[k*n+i] * a[k*n+j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            a[i*n+j] = a[j*n+i];
        }
    }

    return 0; // success
}*/

/*static void zeros(double * a, int m, int n)
{
    int j;
    for (j=0; j<m*n; ++j)
        a[j] = 0;
}
*/

/*
#ifdef DEBUG
static void dump(double * a, int m, int n, const char * fmt)
{
    int i,j;

    char f[100];
    sprintf(f, "%s ", fmt);
    for(i=0; i<m; ++i) {
        for(j=0; j<n; ++j)
            printf(f, a[i*n+j]);
        printf("\n");
    }
}
#endif
*/



/* C <- A * B */
 /*static void mulmat(double * a, double * b, double * c, int arows, int acols, int bcols)
{
    int i, j,l;

    for(i=0; i<arows; ++i)
        for(j=0; j<bcols; ++j) {
            c[i*bcols+j] = 0;
            for(l=0; l<acols; ++l)
                c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j];
        }
}
 */

/*static void mulvec(double * a, double * x, double * y, int m, int n)
{
    int i, j;

    for(i=0; i<m; ++i) {
        y[i] = 0;
        for(j=0; j<n; ++j)
            y[i] += x[j] * a[i*n+j];
    }
}
*/

/*static void transpose(double * a, double * at, int m, int n)
{
    int i,j;

    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j) {
            at[j*m+i] = a[i*n+j];
        }
}
*/

/* A <- A + B */
/*static void accum(double * a, double * b, int m, int n)
{        
    int i,j;

    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j)
            a[i*n+j] += b[i*n+j];
}
*/

/* C <- A + B */
/*static void add(double * a, double * b, double * c, int n)
{
    int j;

    for(j=0; j<n; ++j)
        c[j] = a[j] + b[j];
}
*/

/* C <- A - B */
/*static void sub(double * a, double * b, double * c, int n)
{
    int j;

    for(j=0; j<n; ++j)
        c[j] = a[j] - b[j];
}
*/

/*static void negate(double * a, int m, int n)
{        
    int i, j;

    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j)
            a[i*n+j] = -a[i*n+j];
}
*/

/*static void mat_addeye(double * a, int n)
{
    int i;
    for (i=0; i<n; ++i)
        a[i*n+i] += 1;
}
*/



