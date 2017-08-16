

#ifndef MATLIB_H
#define MATLIB_H


// Standard includes
#include <math.h>
#include <complex.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Define values of PI
#define PI  M_PI
#define PI2 (2.0*PI)
#define PIH (PI/2.0)
#define PIQ (PI/4.0)
#define PIE (PI/8.0)
#define PIS (PI/16.0)


// Matrix structure
typedef struct matrix {
  int rows;
  int cols;
  double *data;
} matrix;


// Complex matrix structure
typedef struct matrixz {
  int rows;
  int cols;
  double complex *data;
} matrixz;


// MatManip Functions
void      mat_err     ( bool cond, char *msg );
matrix*   mat_init    ( uint rows, uint cols );
matrix*   mat_read    ( char *file );
void      mat_print   ( matrix *mat );
void      mat_write   ( matrix *mat, char *file );
void      mat_clear   ( matrix *mat );
double    mat_get     ( matrix *mat, uint row, uint col );
matrix*   mat_getr    ( matrix *mat, uint row );
matrix*   mat_getc    ( matrix *mat, uint col );
void      mat_set     ( matrix *mat, uint row, uint col, double val );
void      mat_setr    ( matrix *mat, uint row, matrix *vec );
void      mat_setc    ( matrix *mat, uint col, matrix *vec );
matrix*   mat_copy    ( matrix *mat );
matrix*   mat_eye     ( uint n );
matrix*   mat_ones    ( uint rows, uint cols );
matrix*   mat_scale   ( matrix *mat, double scale );
void      mat_swapr   ( matrix *mat, uint p, uint q );
void      mat_swapc   ( matrix *mat, uint p, uint q );
matrix*   mat_appr    ( matrix *matT, matrix *matB );
matrix*   mat_appc    ( matrix *matL, matrix *matR );
void      mat_rmtiny  ( matrix **mat, double tol );


// MatComplex functions
matrixz*  mat_initz   ( uint rows, uint cols );
matrixz*  mat_readz   ( char *file );
void      mat_printz  ( matrixz *mat );
void      mat_writez  ( matrixz *mat, char *file );
void      mat_clearz  ( matrixz *mat );
double    mat_getre   ( matrixz *mat, uint row, uint col );
double    mat_getim   ( matrixz *mat, uint row, uint col );
matrixz*  mat_getzr   ( matrixz *mat, uint row );
matrixz*  mat_getzc   ( matrixz *mat, uint col );
void      mat_setz    ( matrixz *mat, uint row, uint col, double re, double im );
void      mat_setzr   ( matrixz *mat, uint row, matrixz *vec );
void      mat_setzc   ( matrixz *mat, uint col, matrixz *vec );


// MatVec functions
matrix*   mat_skew    ( matrix *vec );
matrix*   mat_sskew   ( matrix *vec );
matrix*   mat_cross   ( matrix *vecA, matrix *vecB );
double    mat_dot     ( matrix *vecA, matrix *vecB );
double    mat_norm    ( matrix *vec, uint p );
double    mat_mag     ( matrix *vec );
matrix*   mat_uvec    ( matrix *vec );
matrix*   mat_proj    ( matrix *u, matrix *v );


// MatArith functions
matrix*   mat_add     ( matrix *matA, matrix *matB );
matrix*   mat_sub     ( matrix *matA, matrix *matB );
matrix*   mat_emul    ( matrix *matA, matrix *matB );
matrix*   mat_ediv    ( matrix *matA, matrix *matB );
matrix*   mat_mul     ( matrix *matA, matrix *matB );
matrix*   mat_inv     ( matrix *mat );
matrix*   mat_divL    ( matrix *matA, matrix *matB );
matrix*   mat_divR    ( matrix *matA, matrix *matB );
matrix*   mat_pow     ( matrix *mat, uint power );
matrix*   mat_abs     ( matrix *mat );
matrix*   mat_trans   ( matrix *mat );


// MatRoot function
matrixz*  mat_root    ( matrix *poly, double tol, uint max );


// MatProp functions
double    mat_det     ( matrix *mat );
double    mat_trace   ( matrix *mat );
bool      mat_sym     ( matrix *mat, double tol );


// MatDecomp functions
void      mat_LU      ( matrix *mat, matrix **L, matrix **U );
void      mat_LDU     ( matrix *mat, matrix **L, matrix **D, matrix **U );
void      mat_QR      ( matrix *mat, matrix **Q, matrix **R );     //---  TODO: QR factorization on short matrices  ---//
//void      mat_chol    ( double a[], uint n, uint nn, double u[], uint *nullty, uint *ifault );
//void      mat_syminv  ( double a[], uint n, double c[], double w[], uint *nullty, uint *ifault );
matrix*   mat_tri2vec ( matrix *tri );
matrix*   mat_vec2tri ( matrix *vec );


#endif



