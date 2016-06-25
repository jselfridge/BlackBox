

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
  comp.dt     = 1.0 / HZ_COMP;
  comp.alpha  = 0.96;
  comp.roll   = 0.0;
  comp.pitch  = 0.0;
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
  double R, gyrR, accR;
  double P, gyrP, accP;
  double ax, ay, az;

  // Obtain previous values
  pthread_mutex_lock(&comp.mutex);
  dt    = comp.dt;
  alpha = comp.alpha;
  R     = comp.roll;
  P     = comp.pitch;
  pthread_mutex_unlock(&comp.mutex);

  // Zero out gyr values
  gyrR = 0.0;
  gyrP = 0.0;

  // Obtain gyrA data
  if(IMUA_ENABLED)  {
    pthread_mutex_lock(&gyrA.mutex);
    gyrR += gyrA.scaled[0];
    gyrP += gyrA.scaled[1];
    pthread_mutex_unlock(&gyrA.mutex);
  }

  // Obtain gyrB data
  if(IMUB_ENABLED)  {
    pthread_mutex_lock(&gyrB.mutex);
    gyrR += gyrB.scaled[0];
    gyrP += gyrB.scaled[1];
    pthread_mutex_unlock(&gyrB.mutex);
  }

  // Average gyr data
  if ( IMUA_ENABLED && IMUB_ENABLED )  {
    gyrR /= 2.0;
    gyrP /= 2.0;
  }

  // Zero out acc values
  ax = 0.0;
  ay = 0.0;
  az = 0.0;

  // Obtain accA data
  if (IMUA_ENABLED)  {
    pthread_mutex_lock(&accA.mutex);
    ax +=  accA.scaled[0];
    ay += -accA.scaled[1];
    az += -accA.scaled[2];
    pthread_mutex_unlock(&accA.mutex);
  }

  // Obtain accB data
  if (IMUB_ENABLED)  {
    pthread_mutex_lock(&accB.mutex);
    ax +=  accB.scaled[0];
    ay += -accB.scaled[1];
    az += -accB.scaled[2];
    pthread_mutex_unlock(&accB.mutex);
  }

  // Average acc data
  if ( IMUA_ENABLED && IMUB_ENABLED )  {
    ax /= 2.0;
    ay /= 2.0;
    az /= 2.0;
  }

  // Calculate accelerometer angle
  accR = atan2(ay,az);
  accP = atan2(ax,az);

  // Calculate attitude angles
  R = alpha * ( R + gyrR * dt ) + ( 1.0 - alpha ) * accR;
  P = alpha * ( P + gyrP * dt ) + ( 1.0 - alpha ) * accP;

  // Store updated values
  pthread_mutex_lock(&comp.mutex);
  comp.roll  = R;
  comp.pitch = P;
  pthread_mutex_unlock(&comp.mutex);

  return;
}



