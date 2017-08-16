

//#include "gpio.h"
//#include <fcntl.h>
//#include <stdio.h>
//#include <string.h>
//#include <unistd.h>


/**
 *  gpio_export
 *  Establishes a GPIO configuration.
 */
/*
void gpio_export ( uint gpio )  {

  int fd, len;
  char buf[GPIO_BUF];

  len = snprintf( buf, sizeof(buf), "%d", gpio );
  if( len <=0 )  printf( "Error (gpio_export): Failed to assign path. \n" );

  fd = open( GPIO_PATH "/export", O_WRONLY );
  if( fd <0 )  printf( "Error (gpio_export): Failed to open file. \n" );

  write( fd, buf, len );
  close(fd);

  return;
}
*/

/**
 *  gpio_unexport
 *  Disables a GPIO configuration.
 */
/*
void gpio_unexport ( uint gpio )  {

  int fd, len;
  char buf[GPIO_BUF];

  len = snprintf( buf, sizeof(buf), "%d", gpio );
  if( len <=0 )  printf( "Error (gpio_unexport): Failed to assign path. \n" );

  fd = open( GPIO_PATH "/unexport", O_WRONLY );
  if( fd <0 )  printf( "Error (gpio_unexport): Failed to open file. \n" );
    
  write( fd, buf, len );
  close(fd);
    
  return;
}
*/

/**
 *  gpio_setdir
 *  Assign a GPIO pin as an input or output.
 */
/*
void gpio_setdir ( uint gpio, enum PIN_DIR dir )  {

  int fd, len;
  char buf[GPIO_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/direction", gpio );
  if( len <=0 )  printf( "Error (gpio_setdir): Failed to assign path. \n" );

  fd = open( buf, O_WRONLY );
  if( fd <0 )  printf( "Error (gpio_setdir): Failed to open file. \n" );

  if ( dir == OUTPUT_PIN )  write( fd, "out", 4 );
  else                      write( fd, "in",  3 );
  close(fd);

  return;
}
*/

/**
 *  gpio_setval
 *  Assign a value to a gpio pin.
 */
/*
void gpio_setval ( uint gpio, enum PIN_VAL val )  {

  int fd, len;
  char buf[GPIO_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/value", gpio );
  if( len <=0 )  printf( "Error (gpio_setval): Failed to assign path. \n" );

  fd = open( buf, O_WRONLY );
  if( fd <0 )  printf( "Error (gpio_setval): Failed to open file. \n" );

  if ( val == LOW )  write( fd, "0", 2 );
  else               write( fd, "1", 2 );
  close(fd);

  return;
}
*/

/**
 *  gpio_getval
 *  Determine the current value of a GPIO pin.
 */
/*
void gpio_getval ( uint gpio, uint *val )  {

  int fd, len;
  char buf[GPIO_BUF];
  char ch;

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/value", gpio );
  if( len <=0 )  printf( "Error (gpio_getval): Failed to assign path. \n" );

  fd = open( buf, O_RDONLY );
  if( fd <0 )  printf( "Error (gpio_getval): Failed to open file. \n" );

  read( fd, &ch, 1 );
  if ( ch == '0' )  *val = 0;
  else              *val = 1;
  close(fd);

  return;
}
*/

/**
 *  gpio_setedge
 *  Edge assignment options: "none", "rising", "falling", "both".
 */
/*
void gpio_setedge ( uint gpio, char *edge )  {

  int fd, len;
  char buf[GPIO_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/edge", gpio );
  if( len <=0 )  printf( "Error (gpio_setedge): Failed to assign path. \n" );

  fd = open( buf, O_WRONLY );
  if( fd <0 )  printf( "Error (gpio_setedge): Failed to open file. \n" );

  write( fd, edge, strlen(edge)+1 );
  close(fd);

  return;
}
*/

/**
 *  gpio_fdopen
 *  Opens a GPIO file description.
 */
/*
int gpio_fdopen ( uint gpio )  {

  int fd, len;
  char buf[GPIO_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/value", gpio );
  if( len <=0 )  printf( "Error (gpio_fdopen): Failed to assign path. \n" );

  fd = open( buf, O_RDONLY | O_NONBLOCK );
  if( fd <0 )  printf( "Error (gpio_fdopen): Failed to open file. \n" );

  return fd;
}
*/

/**
 *  gpio_fdclose
 *  Closes a GPIO file description.
 */
/*
void gpio_fdclose ( int fd )  {
  close(fd);
  return;
}
*/


