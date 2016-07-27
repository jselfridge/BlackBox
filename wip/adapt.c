

#include "adapt.h"
#include <stdio.h>
#include "sys.h"


/**
 *  adapt_init
 *  Initializes the adaptive control structure.
 */
void adapt_init ( void )  {
  if (DEBUG)  printf("Initializing adaptive ctrl \n");

  adaptR.x1   = 2.6;
  adaptR.x2   = 0.4;
  adaptR.r    = 3.2;
  adaptR.u    = 0.0;
  adaptR.kx1  = 0.0;
  adaptR.kx2  = 0.0;
  adaptR.kr   = 0.0;
  adaptR.kp   = 0.0;
  adaptR.kx1p = 0.0;
  adaptR.kx2p = 0.0;
  adaptR.krp  = 0.0;
  adaptR.Gx1  = 0.6;
  adaptR.Gx2  = 0.8;
  adaptR.Gr   = 1.0;
  adaptR.Gp   = 1.2;

  return;
}


/**
 *  adapt_exit
 *  Exits the adaptive control code.
 */
void adapt_exit ( void )  {
  if (DEBUG)  printf("Close adaptive ctrl \n");
  // Add exit code as needed...
  return;
}


/**
 *  adapt_update
 *  Executes the adaptive control update loop.
 */
void adapt_update ( adapt_struct *adapt, double *states )  {

  // Get current states (from measurements, or fcn input)
  double x1, x2, r;
  x1 = states[0];
  x2 = states[1];
  r  = states[2];

  // Get previous states (from structure)
  double x1p, x2p, rp;
  x1p = adapt->x1;
  x2p = adapt->x2;
  rp  = adapt->r;

  // Get adaptive law gains
  double Gx1, Gx2, Gr, Gp;
  Gx1 = adapt->Gx1;
  Gx2 = adapt->Gx2;
  Gr  = adapt->Gr;
  Gp  = adapt->Gp;

  // Get current state gains (from structure)
  double kx1, kx2, kr, kp;
  kx1 = adapt->kx1;
  kx2 = adapt->kx2;
  kr  = adapt->kr;
  kp  = adapt->kp;

  // Get previous state gains (from structure)
  double kx1p, kx2p, krp;
  kx1p = adapt->kx1p;
  kx2p = adapt->kx2p;
  krp  = adapt->krp;

  // Assign previous state gains (before they are updated)
  adapt->kx1p = kx1;
  adapt->kx2p = kx2;
  adapt->krp  = kr;

  // Control input 
  double u = kx1 * x1 + kx2 * x2 + kr * r;

  // Auxilliary signals
  double xi =  ( kx1 - kx1p ) * x1p  + ( kx2 - kx2p ) * x2p  + ( kr - krp ) * rp;
  double eps = ( x1 - rp ) + ( kp * xi );

  // Normalizing signal
  double m = 1 + x1p * x1p + x2p * x2p + rp * rp + xi * xi;
  double n = eps / m;

  // Adaptive laws
  kx1 -= Gx1 * x1p * n;
  kx2 -= Gx2 * x2p * n;
  kr  -= Gr  * rp  * n;
  kp  -= Gp  * xi  * n;

  // Update states
  adapt->x1 = x1;
  adapt->x2 = x2;
  adapt->r  = r;
  adapt->u  = u;

  // Update current gains
  adapt->kx1 = kx1;
  adapt->kx2 = kx2;
  adapt->kr  = kr;
  adapt->kp  = kp;

  return;
}



