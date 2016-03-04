

#include "i2c.h"


/**
 *  i2c_init
 *  Assigns a file descriptor after opening an I2C bus. 
 */
int i2c_init ( int *fd, ushort bus, ushort slave_addr )  {

  char buf[32];
  sprintf( buf, "/dev/i2c-%d", bus );
  *fd = open( buf, O_RDWR );

  if ( *fd < 0 ) {
    printf( "Error (i2c_init): Returned a negative I2C file descriptor. \n" );
    *fd = 0;
    return -1;
  }

  if ( ioctl( *fd, I2C_SLAVE, slave_addr ) < 0 )  {
    printf( "Error (i2c_init): Returned negative value from 'ioctl' command. \n" );
    return -1;
  }

  return 0;
}


/**
 *  i2c_exit
 *  Closes file descriptor and exits I2C channel.
 */
void i2c_exit ( int *fd )  {
  close(*fd);
  *fd=0;
  return;
}


/**
 *  i2c_slave
 *  Configures 'ioctl' when the slave address is changed.
 */
int i2c_slave ( int fd, unsigned char slave_addr )  {

  if ( slave == slave_addr )  return 0;

  if ( ioctl( fd, I2C_SLAVE, slave_addr ) < 0 )  {
    printf( "Error (i2c_slave): Returned negative value from 'ioctl' command. \n" );
    return -1;
  }

  slave = slave_addr;

  return 0;
}


/**
 *  i2c_write
 *  Write data to the I2C bus.
 */
int i2c_write ( int fd, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data )  {

  // Local variables
  int  result, i;
  char buf [I2C_MAX_WRITE];

  // Check size of data
  if ( length > I2C_MAX_WRITE-1 )  {
    printf( "Error (i2c_tx): Exceeded maximum write length. \n" );
    return -1;
  }

  if ( i2c_slave( fd, slave_addr ) )  return -1;

  // No data to send
  if ( length == 0 )  {
    result = write( fd, &reg_addr, 1 );
    if ( result < 0 )  {
      printf( "Error (i2c_tx): Returned negative value with 'write' command. \n" );
      return result;
    }
    else if ( result != 1 )  {
      printf( "Error (i2c_tx): Did not transmit the byte. \n" );
      return -1;
    }
  }

  // Send data
  else {
    buf[0] = reg_addr;
    for ( i=0; i<length; i++ )  buf[i+1] = data[i];
    result = write( fd, buf, length+1 );
    if ( result < 0 )  {
      printf( "Error (i2c_tx): Returned negative value with 'write' command. \n" );
      return result;
    }
    else if ( result < (int) length )  {
      printf( "Error (i2c_tx): Only transmitted %d out of %u bytes. \n", result, length );
      return -1;
    }
  }

  return 0;
}


/**
 *  i2c_read
 *  Read data from the I2C.
 */
int i2c_read ( int fd, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data )  {

  int result;
  int tries=0, total=0;

  if ( i2c_write( fd, slave_addr, reg_addr, 0, NULL ) )  return -1;

  while ( total < length && tries < 5 )  {
    result = read( fd, data + total, length - total );
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


/**
 *  i2c_get_ms
 *  Determines the number of milliseconds since epoch.
 */
int i2c_get_ms ( unsigned long *ms )  {
  struct timeval t;
  if (!ms)  return -1;
  if ( gettimeofday( &t, NULL ) < 0 )  {
    printf( "Error (i2c_get_ms): Returned a negative value from 'gettimeofday' command. \n" );
    return -1;
  }
  *ms = ( t.tv_sec * 1000 ) + ( t.tv_usec / 1000 );
  printf( "ms: %ld \n", *ms );
  return 0;
}



