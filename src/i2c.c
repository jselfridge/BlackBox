
//============================================================
//  i2c.c
//  Justin M Selfridge
//============================================================
#include "i2c.h"




#define MAX_WRITE_LEN 511
//int i2c_bus = 1;
//int i2c_fd;
int current_slave;
//unsigned char txBuff[MAX_WRITE_LEN + 1];





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_open
//  Assigns a file descriptor after opening an I2C bus. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_open ( void )  {

  // i2c_bus is a global variable which can be directly passed into the function...
  // i2c_fd  is a global variable which needs to be passed in as a pointer... 

  // Local variables
  char buf[32];

  if (!i2c_fd) {

    sprintf( buf, "/dev/i2c-%d", i2c_bus );
    i2c_fd = open( buf, O_RDWR );

    if ( i2c_fd < 0 ) {
      printf( "Error (i2c_open): Returned a negative I2C file descriptor. \n" );
      i2c_fd = 0;
      return -1;
    }

  }

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_close
//  Closes an I2C channel.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void i2c_close ( void )  {
  // i2c_fd and current_slave are globals that need to be eliminated...
  if (i2c_fd) {
    close(i2c_fd);
    i2c_fd = 0;
    current_slave = 0;
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_xxxx
//  Don't think I need this...
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

int i2c_select_slave ( unsigned char slave_addr )  {

  if ( current_slave == slave_addr )
    return 0;

  if ( i2c_open() )
    return -1;

  //#ifdef I2C_DEBUG
  //printf("\t\ti2c_select_slave(%02X)\n", slave_addr);
  //#endif

  if ( ioctl( i2c_fd, I2C_SLAVE, slave_addr ) < 0 )  {
    perror("ioctl(I2C_SLAVE)");
    return -1;
  }

  current_slave = slave_addr;
  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_xxxx
//  Eliminate this along with the globals...
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void linux_set_i2c_bus ( int bus )  {
  if (i2c_fd)
    i2c_close();
  i2c_bus = bus;
  i2c_addr = 0x68;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_tx
//  Transmit data to the I2C bus.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_tx ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data )  {

  // Add a function parameter to specify the i2c_fd, which is currently global...
  // Eliminated global reg_addr...

  int result, i;
  char buf [ MAX_WRITE_LEN+1 ];

  if ( length > MAX_WRITE_LEN )  {
    printf( "Error (i2c_tx): Exceeded maximum write length. \n" );
    return -1;
  }

  if ( i2c_select_slave(slave_addr) )  return -1;

  if ( length == 0 )  {
    result = write( i2c_fd, &reg_addr, 1 );

    if ( result < 0 )  {
      printf( "Error (i2c_tx): Returned negative value with 'write' command. \n" );
      return result;
    }

    else if ( result != 1 )  {
      printf( "Error (i2c_tx): Did not transmit the byte. \n" );
      return -1;
    }

  }

  else {

    buf[0] = reg_addr;

    for ( i=0; i<length; i++ )  buf[i+1] = data[i];

    result = write( i2c_fd, buf, length+1 );

    if ( result < 0 )  {
      printf( "Error (i2c_tx): Returned negative value with 'write' command. \n" );
      return result;
    }

    else if ( result < (int)length )  {
      printf( "Error (i2c_tx): Only transmitted %d out of %u bytes. \n", result, length );
      return -1;
    }

  }

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_rx
//  Receive data from the I2C.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_rx ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data )  {

  // Add function parameter to specify the i2c_fd, currently a global variable...

  int tries=0, result, total=0;

  if ( i2c_write( slave_addr, reg_addr, 0, NULL ) )  return -1;

  //total = 0;
  //tries = 0;

  while ( total < length && tries < 5 )  {

    result = read( i2c_fd, data + total, length - total );

    if (result < 0) {
      printf( "Error (i2c_rx): Returned a negative value from 'read' command. \n" );
      break;
    }

    total += result;

    if ( total == length )  break;

    tries++;		
    usleep(10000);

  }

  if ( total < length )  return -1;

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_ct
//  Implements a counter for proper I2C timing.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_ct ( unsigned long *count )  {
  struct timeval t;
  if (!count)  return -1;
  if ( gettimeofday( &t, NULL ) < 0 )  {
    printf( "Error (i2c_ct): Returned a negative value from 'gettimeofday' command. \n" );
    return -1;
  }
  *count = ( t.tv_sec * 1000 ) + ( t.tv_usec / 1000 );
  return 0;
}



