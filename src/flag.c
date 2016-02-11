
//============================================================
//  flag.c
//  Justin M Selfridge
//============================================================
#include "flag.h"

/*
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  flg_init
//  Initializes the program execution flag structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void flg_init ( void )  {
  if(DEBUG)  printf("Initializing program execution flags \n");

  // Set boolean values
  if(DEBUG)  printf("  Set run time flags \n");
  datalog.enabled  = false;
  datalog.setup    = false;
  datalog.saving   = false;
  running          = true;
  armed            = false;

  // Zero out counters
  if(DEBUG)  printf("  Zero out counters \n");
  ushort ch;
  for ( ch=0; ch<4; ch++ )  {
    flag.upper[ch] = 0;
    flag.lower[ch] = 0;
  }

  // Set proper timing limits
  if(DEBUG)  printf("  Set timing limits \n");
  flag.limit[CH_R] = FLG_HOLD_R * HZ_FLAG;
  flag.limit[CH_P] = FLG_HOLD_P * HZ_FLAG;
  flag.limit[CH_Y] = FLG_HOLD_Y * HZ_FLAG;
  flag.limit[CH_T] = FLG_HOLD_T * HZ_FLAG;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  flg_exit
//  End of program related taskss.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void flg_exit ( void )  {
  if(DEBUG)  printf("Close flags \n");
  // Add code and functions as needed...
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  flg_update
//  Updates the current status of the program execution flags.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void flg_update ( void )  {

  // Local variables
  ushort ch;
  bool energized = false;

  // Adjust state and counters
  pthread_mutex_lock(&mutex_input);
  energized = ( input.norm[CH_T] <= -0.9 ) ? ( false ) : ( true );
  for ( ch=0; ch<4; ch++ ) {
    ( input.norm[ch] >  0.95 ) ? ( flag.upper[ch]++ ) : ( flag.upper[ch] = 0 );
    ( input.norm[ch] < -0.95 ) ? ( flag.lower[ch]++ ) : ( flag.lower[ch] = 0 );
  }
  pthread_mutex_unlock(&mutex_input);

  // Data log: roll stick only, no yaw command
  if ( !energized && !flag.lower[CH_Y] && !flag.upper[CH_Y] )  {
    if ( flag.lower[CH_R] >= flag.limit[CH_R] ) {
      if (!datalog.setup)  log_open();
      usleep(200000);
      datalog.enabled = true;
      led_on(LED_LOG);
    }
    if ( flag.upper[CH_R] >= flag.limit[CH_R] ) {
      datalog.enabled = false;
      usleep(200000);
      if (datalog.setup)  log_close();
      led_off(LED_LOG);
    }
  }

  // Motor arming: yaw stick only, no roll command
  if ( !energized && !flag.lower[CH_R] && !flag.upper[CH_R] )  {
    if ( flag.upper[CH_Y] >= flag.limit[CH_Y] ) {
      //pthread_mutex_lock(&mutex_ctrl);
      //pthread_mutex_lock(&mutex_eul);
      //ctrl.bank    = ahr.eul[0];
      //ctrl.climb   = ahr.eul[1];
      //ctrl.heading = ahr.eul[2];
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

*/

