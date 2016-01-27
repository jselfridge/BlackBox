
//============================================================
//  flag.c
//  Justin M Selfridge
//============================================================
#include "flag.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  flg_init
//  Initializes the program execution flag structure.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void flg_init ( void )  {
  if(DEBUG)  printf("Initializing program execution flags \n");
  // Add code and functions as needed...

  // Disarm the outputs
  //sio_disarm();
  
  // Set boolean values
  //running = true;
  //armed = false;

  // Zero out counters
  ushort ch;
  for ( ch=0; ch<IN_CH; ch++ )  {
    flag.upper[ch] = 0;
    flag.lower[ch] = 0;
  }

  // Set proper limits
  flag.limit[0] = FLG_HOLD0 * HZ_FLAG;
  flag.limit[1] = FLG_HOLD1 * HZ_FLAG;
  flag.limit[2] = FLG_HOLD2 * HZ_FLAG;
  flag.limit[3] = FLG_HOLD3 * HZ_FLAG;
  flag.limit[4] = FLG_HOLD4 * HZ_FLAG;
  flag.limit[5] = FLG_HOLD5 * HZ_FLAG;
  flag.limit[6] = FLG_HOLD6 * HZ_FLAG;
  flag.limit[7] = FLG_HOLD7 * HZ_FLAG;
  flag.limit[8] = FLG_HOLD8 * HZ_FLAG;
  flag.limit[9] = FLG_HOLD9 * HZ_FLAG;

  for ( ch=0; ch<IN_CH; ch++ ) {
    printf("limit %d: %d \n", ch, flag.limit[ch] );
  }
  
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
//  flg_debug
//  BLAH...
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void flg_debug ( void )  {

  // Channel index
  ushort ch;

  // Adjust counters
  for ( ch=0; ch<IN_CH; ch++ ) {
    if ( input.norm[ch] >  0.95 )  flag.upper[ch]++;  else flag.upper[ch] = 0;
    if ( input.norm[ch] < -0.95 )  flag.lower[ch]++;  else flag.lower[ch] = 0;
  }

  // Only execute stick commands with zero throttle
  if ( input.norm[3] <= -0.95 )  {

    // Data log: roll stick only, no yaw command
    if ( !flag.lower[2] && !flag.upper[2] )  {
      if ( flag.lower[0] >= flag.limit[0] ) {  /*datalog.enabled = true; */ led_on(LED_LOG);   }
      if ( flag.upper[0] >= flag.limit[0] ) {  /*datalog.enabled = false;*/ led_off(LED_LOG);  }
    }

    // Motor arming: yaw stick only, no roll command
    if ( !flag.lower[0] && !flag.upper[0] )  {
      if ( flag.upper[2] >= flag.limit[2] ) {
	//ctrl.motorsArmed = true;
	//ctrl.heading = imu1.Eul[Z];
	led_on(LED_MOT);
      }
      if ( flag.lower[2]  >= flag.limit[2] ) {
	//ctrl.motorsArmed = false;
	led_off(LED_MOT);
      }
    }

    // Exit program: roll and yaw together to exit program
    if (  flag.lower[2] >= flag.limit[2]  &&  flag.upper[0] >= flag.limit[0]  ) {
      //sys.running = false;
      led_off(LED_IMU);
      led_off(LED_PRU);
      led_off(LED_LOG);
      led_off(LED_MOT);
    }

  }

  return;
}



