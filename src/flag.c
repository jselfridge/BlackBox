

#include "flag.h"
#include <stdio.h>
#include "ahrs.h"
#include "ctrl.h"
#include "io.h"
#include "led.h"
#include "log.h"
#include "sys.h"
#include "timer.h"


/**
 *  flag_init
 *  Initializes the program execution flag structure.
 */
void flag_init ( void )  {
  if(DEBUG)  printf("Initializing program execution flags \n");

  // Zero out counters
  if(DEBUG)  printf("  Zero out counters \n");
  ushort ch;
  for ( ch=0; ch<4; ch++ )  {
    flag.upper[ch] = 0;
    flag.lower[ch] = 0;
  }

  // Set proper timing limits
  if(DEBUG)  printf("  Set timing limits \n");
  flag.limit[CH_R] = FLAG_HOLD_R * HZ_FLAG;
  flag.limit[CH_P] = FLAG_HOLD_P * HZ_FLAG;
  flag.limit[CH_Y] = FLAG_HOLD_Y * HZ_FLAG;
  flag.limit[CH_T] = FLAG_HOLD_T * HZ_FLAG;

  return;
}


/**
 *  flag_exit
 *  End of program related taskss.
 */
void flag_exit ( void )  {
  if(DEBUG)  printf("Close flags \n");
  // Add code and functions as needed...
  return;
}


/**
 *  flag_update
 *  Updates the current status of the program execution flags.
 */
void flag_update ( void )  {

  // Local variables
  ushort ch;
  bool energized = false;

  // Adjust state and counters
  pthread_mutex_lock(&input.mutex);
  energized = ( input.norm[CH_T] <= -0.9 ) ? ( false ) : ( true );
  for ( ch=0; ch<4; ch++ ) {
    ( input.norm[ch] >  0.9 ) ? ( flag.upper[ch]++ ) : ( flag.upper[ch] = 0 );
    ( input.norm[ch] < -0.9 ) ? ( flag.lower[ch]++ ) : ( flag.lower[ch] = 0 );
  }
  pthread_mutex_unlock(&input.mutex);

  // Data log: roll stick only, no yaw command
  if ( !energized && !flag.lower[CH_Y] && !flag.upper[CH_Y] )  {
    if ( flag.lower[CH_R] >= flag.limit[CH_R] ) {
      if (!datalog.setup)  log_start();
      datalog.enabled = true;
      led_on(LED_LOG);
    }
    if ( flag.upper[CH_R] >= flag.limit[CH_R] ) {
      datalog.enabled = false;
      if (datalog.setup)  log_finish();
      led_off(LED_LOG);
    }
  }

  // Motor arming: yaw stick only, no roll command
  if ( !energized && !flag.lower[CH_R] && !flag.upper[CH_R] )  {
    if ( flag.upper[CH_Y] >= flag.limit[CH_Y] ) {
      //pthread_mutex_lock(&mutex_ctrl);
      //pthread_mutex_lock(&mutex_eul);
      //ctrl.bank    = ahrs.eul[0];
      //ctrl.climb   = ahrs.eul[1];
      //ctrl.heading = ahrs.eul[2];
      //pthread_mutex_unlock(&mutex_eul);
      //pthread_mutex_unlock(&mutex_ctrl);
      armed = true;
      led_on(LED_MOT);
    }
    if ( flag.lower[CH_Y]  >= flag.limit[CH_Y] ) {
      armed = false;
      led_off(LED_MOT);
    }
  }

  // Exit program: roll and yaw together to exit program
  if (  flag.lower[CH_Y] >= flag.limit[CH_Y]  &&
        flag.upper[CH_R] >= flag.limit[CH_R]  &&
        !energized  )  running = false;

  return;
}



