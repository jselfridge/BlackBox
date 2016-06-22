

#include "comp.h"
#include <stdio.h>
#include <math.h>
//#include <stdlib.h>
//#include <string.h>
#include "imu.h"
#include "sys.h"
#include "timer.h"


/**
 *  comp_init
 *  Initializes the complimentary filter.
 */
void comp_init ( void )  {
  if(DEBUG)  printf( "Initializing complimentary filter \n" );
  comp.dt    = 1.0 / HZ_COMP;
  comp.alpha = 0.95;
  comp.roll  = 0.0;
  comp.pitch = 0.0;
  return;
}


/**
 *  comp_exit
 *  Terminate the complimentary filter.
 */
void comp_exit ( void )  {
  if(DEBUG)  printf("Close complimentary filter \n");
  // Insert code if needed...
  return;
}


/**
 *  comp_update
 *  Implement gyr/acc complimentary filter
 */
void comp_update ( void )  {

  // Local variables
  double dt, alpha;
  double R, gR, aR;
  double P, gP, aP;
  double ax, ay, az;

  // Obtain previous values
  pthread_mutex_lock(&comp.mutex);
  dt    = comp.dt;
  alpha = comp.alpha;
  R     = comp.roll;
  P     = comp.pitch;
  pthread_mutex_unlock(&comp.mutex);

  // Obtain gyr data
  pthread_mutex_lock(&gyrA.mutex);
  gR = gyrA.scaled[0];
  gP = gyrA.scaled[1];
  pthread_mutex_unlock(&gyrA.mutex);

  // Obtain acc data
  pthread_mutex_lock(&accA.mutex);
  ax =  accA.scaled[0];
  ay = -accA.scaled[1];
  az = -accA.scaled[2];
  pthread_mutex_unlock(&accA.mutex);

  // Calculate accelerometer angle
  aR = atan2(ay,az);
  aP = atan2(ax,az);

  // Update angles
  R = alpha * ( R + gR * dt ) + ( 1.0 - alpha ) * aR;
  P = alpha * ( P + gP * dt ) + ( 1.0 - alpha ) * aP;

  // Store updated values
  pthread_mutex_lock(&comp.mutex);
  comp.roll  = R;
  comp.pitch = P;
  pthread_mutex_unlock(&comp.mutex);


  return;
}



