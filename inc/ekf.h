

#ifndef EKF_H
#define EKF_H


#define EKF_ENABLED true
#define EKF_N   2
#define EKF_M   1


typedef struct ekf_struct {
  double *x;   // State vector
  double *z;   // Measurement vector
  double *f;   // Plant function output
  double *h;   // Measurement function output
  double *F;   // Plant Jacobian
  double *H;   // Measurement Jacobian
  double *Q;   // Plant error covariance
  double *R;   // Measurement error covariance
  double *P;   // Prediction error covariance
  double *S;   // Update error covariance
  double *K;   // Kalman gain
} ekf_struct;
ekf_struct ekf;


void ekf_init   ( void );
void ekf_exit   ( void );
int  ekf_update ( void );


#endif



