

#ifndef EKF_H
#define EKF_H


#include <sys/types.h>
#include <matLib.h>


#define EKF_ENABLED true
#define EKF_N   2
#define EKF_M   2


typedef struct ekf_struct {
  matrix *x;   // State vector
  matrix *z;   // Measurement vector
  matrix *f;   // Plant function output
  matrix *h;   // Measurement function output
  matrix *F;   // Plant Jacobian
  matrix *H;   // Measurement Jacobian
  matrix *Q;   // Plant error covariance
  matrix *R;   // Measurement error covariance
  matrix *P;   // Prediction error covariance
  matrix *S;   // Update error covariance
  matrix *K;   // Kalman gain
  pthread_mutex_t mutex;
} ekf_struct;
ekf_struct ekf;


void ekf_init   ( void );
void ekf_exit   ( void );
int  ekf_update ( void );


#endif



