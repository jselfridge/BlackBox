

#include "ins.h"
#include <stdio.h>
#include "sys.h"
#include "timer.h"


/**
 *  ins_init
 *  Initializes the inertial navigation system loop.
 */
void ins_init ( void )  {
  if (DEBUG)  printf("Initializing inertial nav \n");

  // Local variables

  // Enable mutex locks
  pthread_mutex_init( &ins.mutex, NULL );

  // Set timing values
  double dt = 1.0 / HZ_INS;
  ins.dt = dt;

  // Display settings
  if (DEBUG)  {
    printf("  INS settings... (to be added) \n");
  }

  return;
}


/**
 *  ins_exit
 *  Exits the inertial navigation system code.
 */
void ins_exit ( void )  {
  if (DEBUG)  printf("Close inertial nav \n");
  pthread_mutex_destroy(&ins.mutex);
  return;
}


/**
 *  ins_update
 *  Executes the inertial navigation system update loop.
 */
void ins_update ( void )  {

  return;
}



