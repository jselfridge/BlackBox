
//============================================================
//  led.c
//  Justin M Selfridge
//============================================================
#include "led.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  led_rmtrig
//  Sets the LED trigger status to 'none'.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int led_rmtrig ( unsigned int index )  {

  uav_err( index >3, "Error (led_rmtrig): LED index is between 0-3." );

  int fd, len;
  char path[MAX_BUF];

  len = snprintf ( path, sizeof(path), LED_PATH "%d/trigger", index );
  uav_err( len <=0, "Error (led_rmtrig): Failed to generate 'trigger' file path. ");

  fd = open( path , O_WRONLY );  
  uav_err( fd <0, "Error (led_rmtrig): Failed to open 'trigger' file. ");

  write( fd, "none", 5  );
  close(fd);

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  led_on
//  Turns on the specified user LED.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int led_on ( unsigned int index )  {

  uav_err( index >3, "Error (led_on): LED index is between 0-3." );

  int fd, len;
  char path[MAX_BUF];

  led_rmtrig(index);

  len = snprintf ( path, sizeof(path), LED_PATH "%d/brightness", index );
  uav_err( len <=0, "Error (led_on): Failed to generate 'brightness' file path. ");

  fd = open ( path, O_WRONLY );
  uav_err( fd <0, "Error (led_on): Failed to open 'brightness' file. ");

  write( fd, "1", 2 );
  close(fd);

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  led_off
//  Turns off the specified user LED.  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int led_off ( unsigned int index )  {

  uav_err( index >3, "Error (led_off): LED index is between 0-3." );

  int fd, len;
  char path[MAX_BUF];

  led_rmtrig(index);

  len = snprintf ( path, sizeof(path), LED_PATH "%d/brightness", index );
  uav_err( len <=0, "Error (led_off): Failed to generate 'brightness' file path. ");

  fd = open ( path, O_WRONLY );
  uav_err( fd <0, "Error (led_off): Failed to open 'brightness' file. ");

  write( fd, "0", 2 );
  close(fd);

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  led_blink
//  Blinks the specified user LED.  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int led_blink ( unsigned int index, unsigned int on, unsigned int off )  {

  uav_err( index  >3, "Error (led_blink): LED index is between 0-3."         );
  uav_err( on    <=0, "Error (led_blink): Duration 'on' must be positive."   );
  uav_err( off   <=0, "Error (led_blink): Duration 'off' must be positive."  );

  int fd, len;
  char path[MAX_BUF];
  char val[MAX_BUF];

  path[0] = '\0';
  len = snprintf ( path, sizeof(path), LED_PATH "%d/trigger", index );
  uav_err( len <=0, "Error (led_blink): Failed to generate 'trigger' file path. ");
  fd = open ( path, O_WRONLY );
  uav_err( fd <0, "Error (led_blink): Failed to open 'trigger' file. ");
  write( fd, "timer", 6 );
  close(fd);

  path[0] = '\0';
  val[0]  = '\0';
  len = snprintf ( path, sizeof(path), LED_PATH "%d/delay_on", index );
  uav_err( len <=0, "Error (led_blink): Failed to generate 'delay_on' file path. ");
  fd = open ( path, O_WRONLY );
  uav_err( fd <0, "Error (led_blink): Failed to open 'delay_on' file. ");
  len = snprintf ( val, sizeof(val), "%d", on );
  uav_err( len<=0, "Error (led_blink): Failed to generate 'delay_on' description. ");
  write( fd, val, len+1 );
  close(fd);

  path[0] = '\0';
  val[0]  = '\0';
  len = snprintf ( path, sizeof(path), LED_PATH "%d/delay_off", index );
  uav_err( len<=0, "Error (led_blink): Failed to generate 'delay_off' file path. ");
  fd = open ( path, O_WRONLY );
  uav_err( fd<0, "Error (led_blink): Failed to open 'delay_off' file. ");
  len = snprintf( val, sizeof(val), "%d", off );
  uav_err( len<=0, "Error (led_blink): Failed to generate 'delay_off' description. ");
  write( fd, val, len+1 );
  close(fd);

  return 0;
}



