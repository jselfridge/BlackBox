

#ifndef EKF_H
#define EKF_H


#define EKF_ENABLED true
#define EKF_N   6
#define EKF_M   6


typedef struct ekf_struct {
  double *x;   // State vector
  double *F;   // Plant Jacobian
  double *H;   // Measurement Jacobian
  double *Q;   // Plant error covariance
  double *R;   // Measurement error covariance
  double *P;   // prediction error covariance
  double *K;   // Kalman gain

  //double *Ht;  // transpose of measurement Jacobian
  //double *Ft;  // transpose of process Jacobian
  //double *Pp;  // P, post-prediction, pre-update
  //double *fx;  // output of user defined f() state-transition function
  //double *hx;  // output of user defined h() measurement function
  //double *tmp1;
  //double *tmp2;
  //double *tmp3;
  //double *tmp4;
  //double *tmp5;
} ekf_struct;
ekf_struct ekf;


void ekf_init   ( void );
void ekf_exit   ( void );
int  ekf_update ( void );


#endif



