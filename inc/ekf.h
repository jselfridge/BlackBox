

#ifndef EKF_H
#define EKF_H


typedef struct ekf_struct {
  double *x;   // state vector
  double *P;   // prediction error covariance
  double *Q;   // process noise covariance
  double *R;   // measurement error covariance
  double *G;   // Kalman gain; a.k.a. K
  double *F;   // Jacobian of process model
  double *H;   // Jacobian of measurement model
  double *Ht;  // transpose of measurement Jacobian
  double *Ft;  // transpose of process Jacobian
  double *Pp;  // P, post-prediction, pre-update
  double *fx;  // output of user defined f() state-transition function
  double *hx;  // output of user defined h() measurement function
  double *tmp1;
  double *tmp2;
  double *tmp3;
  double *tmp4;
  double *tmp5;
} ekf_struct;


void ekf_init   ( void *ekf, int n, int m );
void ekf_exit   ( void );
int  ekf_update ( void *ekf, double *z );


#endif



