
//============================================================
//  gpio.c
//  Justin M Selfridge
//============================================================
#include "gpio.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_export
//  Establishes a GPIO configuration.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_export ( unsigned int gpio )  {

  int fd, len;
  char buf[MAX_BUF];

  len = snprintf( buf, sizeof(buf), "%d", gpio );
  sys_err( len <=0, "Error (gpio_export): Failed to assign path." );

  fd = open( GPIO_PATH "/export", O_WRONLY );
  sys_err( fd <0, "Error (gpio_export): Failed to open file." );

  write( fd, buf, len );
  close(fd);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_unexport
//  Disables a GPIO configuration.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_unexport ( unsigned int gpio )  {

  int fd, len;
  char buf[MAX_BUF];

  len = snprintf( buf, sizeof(buf), "%d", gpio );
  sys_err( len <=0, "Error (gpio_unexport): Failed to assign path." );

  fd = open( GPIO_PATH "/unexport", O_WRONLY );
  sys_err( fd <0, "Error (gpio_unexport): Failed to open file." );
    
  write( fd, buf, len );
  close(fd);
    
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_set_dir
//  Assign a GPIO pin as input or output.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_set_dir ( unsigned int gpio, enum PIN_DIR dir )  {

  int fd, len;
  char buf[MAX_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/direction", gpio );
  sys_err( len <=0, "Error (gpio_set_dir): Failed to assign path." );

  fd = open( buf, O_WRONLY );
  sys_err( fd <0, "Error (gpio_set_dir): Failed to open file." );

  if ( dir == OUTPUT_PIN )  write( fd, "out", 4 );
  else                      write( fd, "in",  3 );
  close(fd);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_set_val
//  Assign a value to a gpio pin.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_set_val ( unsigned int gpio, enum PIN_VAL val )  {

  int fd, len;
  char buf[MAX_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/value", gpio );
  sys_err( len <=0, "Error (gpio_set_val): Failed to assign path." );

  fd = open( buf, O_WRONLY );
  sys_err( fd <0, "Error (gpio_set_val): Failed to open file." );

  if ( val == LOW )  write( fd, "0", 2 );
  else               write( fd, "1", 2 );
  close(fd);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_get_val
//  Determine the current value of a GPIO pin.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_get_val ( unsigned int gpio, unsigned int *val )  {

  int fd, len;
  char buf[MAX_BUF];
  char ch;

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/value", gpio );
  sys_err( len <=0, "Error (gpio_get_val): Failed to assign path." );

  fd = open( buf, O_RDONLY );
  sys_err( fd <0, "Error (gpio_get_val): Failed to open file." );

  read( fd, &ch, 1 );
  if ( ch == '0' )  *val = 0;
  else              *val = 1;
  close(fd);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_set_edge
//  Assignment options: "none", "rising", "falling", "both".
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_set_edge ( unsigned int gpio, char *edge )  {

  int fd, len;
  char buf[MAX_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/edge", gpio );
  sys_err( len <=0, "Error (gpio_set_edge): Failed to assign path." );

  fd = open( buf, O_WRONLY );
  sys_err( fd <0, "Error (gpio_set_edge): Failed to open file." );

  write( fd, edge, strlen(edge)+1 );
  close(fd);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_fd_open
//  Opens a GPIO file description.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_fd_open ( unsigned int gpio )  {

  int fd, len;
  char buf[MAX_BUF];

  len = snprintf( buf, sizeof(buf), GPIO_PATH "/gpio%d/value", gpio );
  sys_err( len <=0, "Error (gpio_fd_open): Failed to assign path." );

  fd = open( buf, O_RDONLY | O_NONBLOCK );
  sys_err( fd <0, "Error (gpio_fd_open): Failed to open file." );
    
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gpio_fd_close
//  Closes a GPIO file description.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void gpio_fd_close ( int fd )  {
  close(fd);
  return;
}



