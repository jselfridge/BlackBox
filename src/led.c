
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

  if( index >3 )
    printf( "Error (led_rmtrig): LED index is between 0-3. \n" );

  int fd, len;
  char path[MAX_BUF];

  len = snprintf ( path, sizeof(path), LED_PATH "%d/trigger", index );
  if( len <=0 )
    printf( "Error (led_rmtrig): Failed to generate 'trigger' file path. \n" );

  fd = open( path , O_WRONLY );  
  if( fd <0 )
    printf( "Error (led_rmtrig): Failed to open 'trigger' file. \n" );

  write( fd, "none", 5 );
  close(fd);

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  led_on
//  Turns on the specified user LED.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int led_on ( unsigned int index )  {

  if( index >3 )
    printf( "Error (led_on): LED index is between 0-3. \n" );

  int fd, len;
  char path[MAX_BUF];

  led_rmtrig(index);

  len = snprintf ( path, sizeof(path), LED_PATH "%d/brightness", index );
  if( len <=0 )
    printf( "Error (led_on): Failed to generate 'brightness' file path. \n" );

  fd = open ( path, O_WRONLY );
  if( fd <0 )
    printf( "Error (led_on): Failed to open 'brightness' file. \n" );

  write( fd, "1", 2 );
  close(fd);

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  led_off
//  Turns off the specified user LED.  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int led_off ( unsigned int index )  {

  if( index >3 )
    printf( "Error (led_off): LED index is between 0-3. \n" );

  int fd, len;
  char path[MAX_BUF];

  led_rmtrig(index);

  len = snprintf ( path, sizeof(path), LED_PATH "%d/brightness", index );
  if( len <=0 )
    printf( "Error (led_off): Failed to generate 'brightness' file path. \n" );

  fd = open ( path, O_WRONLY );
  if( fd <0 )
    printf( "Error (led_off): Failed to open 'brightness' file. \n" );

  write( fd, "0", 2 );
  close(fd);

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  led_blink
//  Blinks the specified user LED.  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int led_blink ( unsigned int index, unsigned int on, unsigned int off )  {

  if( index  >3 )
    printf( "Error (led_blink): LED index is between 0-3. \n"        );
  if( on    <=0 )
    printf( "Error (led_blink): Duration 'on' must be positive. \n"  );
  if( off   <=0 )
    printf( "Error (led_blink): Duration 'off' must be positive. \n" );

  int fd, len;
  char path[MAX_BUF];
  char val[MAX_BUF];

  path[0] = '\0';
  val[0]  = '\0';
  len = snprintf ( path, sizeof(path), LED_PATH "%d/trigger", index );
  if( len <=0 )
    printf( "Error (led_blink): Failed to generate 'trigger' file path. \n" );
  fd = open ( path, O_WRONLY );
  if( fd <0 )
    printf( "Error (led_blink): Failed to open 'trigger' file. \n" );
  write( fd, "timer", 6 );
  close(fd);

  path[0] = '\0';
  val[0]  = '\0';
  len = snprintf ( path, sizeof(path), LED_PATH "%d/delay_on", index );
  if( len <=0 )
    printf( "Error (led_blink): Failed to generate 'delay_on' file path. \n" );
  fd = open ( path, O_WRONLY );
  if( fd <0 )
    printf( "Error (led_blink): Failed to open 'delay_on' file. \n" );
  len = snprintf ( val, sizeof(val), "%d", on );
  if( len<=0 )
    printf( "Error (led_blink): Failed to generate 'delay_on' description. \n" );
  write( fd, val, len+1 );
  close(fd);

  path[0] = '\0';
  val[0]  = '\0';
  len = snprintf ( path, sizeof(path), LED_PATH "%d/delay_off", index );
  if( len<=0 )
    printf( "Error (led_blink): Failed to generate 'delay_off' file path. \n" );
  fd = open ( path, O_WRONLY );
  if( fd<0 )
    printf( "Error (led_blink): Failed to open 'delay_off' file. \n" );
  len = snprintf( val, sizeof(val), "%d", off );
  if( len<=0 )
    printf( "Error (led_blink): Failed to generate 'delay_off' description. \n" );
  write( fd, val, len+1 );
  close(fd);

  return 0;
}



