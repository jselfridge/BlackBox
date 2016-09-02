

#ifndef EKF_H
#define EKF_H


#include <sys/types.h>
#include <matLib.h>


#define EKF_N   2
#define EKF_M   2


typedef struct ekf_struct {
  matrix *x;   // [n,1] State vector
  matrix *z;   // [m,1] Measurement vector
  matrix *f;   // [n,1] Plant function output
  matrix *h;   // [m,1] Measurement function output
  matrix *F;   // [n,n] Plant Jacobian
  matrix *H;   // [m,n] Measurement Jacobian
  matrix *Q;   // [n,n] Plant error covariance
  matrix *R;   // [m,m] Measurement error covariance
  matrix *P;   // [n,n] Prediction error covariance
  matrix *T;   // [n,n] Temp prediction error covariance
  matrix *S;   // [m,m] Update error covariance
  matrix *K;   // [n,m] Kalman gain
  pthread_mutex_t mutex;
} ekf_struct;
ekf_struct ekf;


void ekf_init   ( void );
void ekf_exit   ( void );
void ekf_update ( void );
void ekf_gain   ( void );


#endif



