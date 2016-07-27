

#include "adapt.h"
#include <stdio.h>
#include "sys.h"


/**
 *  adapt_init
 *  Initializes the adaptive control structure.
 */
void adapt_init ( void )  {
  if (DEBUG)  printf("Initializing adaptive ctrl \n");


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
void adapt_update ( void )  {

  // Get current states (from measurements)
  double x1 = 3.0;
  double x2 = 0.4;
  double r  = 3.3;

  // Get previous states (from structure)
  double x1p = adapt_test.x1;
  double x2p = adapt_test.x2;
  double rp  = adapt_test.r;

  // Get current state gains (from structure)
  double kx1 = adapt_test.kx1;
  double kx2 = adapt_test.kx2;
  double kr  = adapt_test.kr;
  double kp  = adapt_test.kp;

  // Get previous state gains (from structure)
  double kx1p = adapt_test.kx1p;
  double kx2p = adapt_test.kx2p;
  double krp  = adapt_test.krp;

  // Get adaptive law gains
  double Gx1 = adapt_test.Gx1;
  double Gx2 = adapt_test.Gx2;
  double Gr  = adapt_test.Gr;
  double Gp  = adapt_test.Gp;

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
  adapt_test.x1 = x1;
  adapt_test.x2 = x2;
  adapt_test.r  = r;
  adapt_test.u  = u;

  // Update gains
  adapt_test.kx1 = kx1;
  adapt_test.kx2 = kx2;
  adapt_test.kr  = kr;

  return;
}



