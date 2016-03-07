

#include "i2c.h"


/**
 *  i2c_init
 *  Assigns a file descriptor after opening an I2C bus. 
 */
int i2c_init ( int *fd, ushort bus, ushort slave_addr )  {

  char buf[32];
  sprintf( buf, "/dev/i2c-%d", bus );
  *fd = open( buf, O_RDWR | O_NONBLOCK );

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

  // Confirm valid fd value
  if ( ( fd != imuA.fd ) && ( fd != imuB.fd ) )  {
    printf( "Error (i2c_slave): Improper fd value. \n" );
    return -1;
  }

  // Check if IMUA addr needs updating
  if ( fd == imuA.fd )  {
    if ( imuA.addr == slave_addr )  return 0;
    else                            imuA.addr = slave_addr;
  }

  // Check if IMUB addr needs updating
  if ( fd == imuB.fd )  {
    if ( imuB.addr == slave_addr )  return 0;
    else                            imuB.addr = slave_addr;
  }

  // Update the appropriate address
  if ( ioctl( fd, I2C_SLAVE, slave_addr ) < 0 )  {
    printf( "Error (i2c_slave): Returned negative value from 'ioctl' command. \n" );
    return -1;
  }

  return 0;
}


/**
 *  i2c_write
 *  Write data to the I2C bus.
 */
int i2c_write ( int fd, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data )  {

  // Confirm valid fd value
  if ( ( fd != imuA.fd ) && ( fd != imuB.fd ) )  {
    printf( "Error (i2c_write): Improper fd value. \n" );
    return -1;
  }

  // Local variables
  int  i, result=0;
  char buf [I2C_MAX_WRITE];

  // Check size of data
  if ( length > I2C_MAX_WRITE-1 )  {
    printf( "Error (i2c_write): Exceeded maximum write length. \n" );
    return -1;
  }

  if ( i2c_slave( fd, slave_addr ) )  return -1;

  // No data to send
  if ( length == 0 )  {

    if ( fd == imuA.fd )  {
      pthread_mutex_lock(&mutex_i2c1);
      result = write( imuA.fd, &reg_addr, 1 );
      pthread_mutex_unlock(&mutex_i2c1);
    }

    if ( fd == imuB.fd )  {
      pthread_mutex_lock(&mutex_i2c2);
      result = write( imuB.fd, &reg_addr, 1 );
      pthread_mutex_unlock(&mutex_i2c2);
    }

    if ( result < 0 )  {
      printf( "Error (i2c_write): Returned '%d' with 'write' command (no data). \n", result );
      printf("errno = %d.\n", errno);
      return result;
    }

    else if ( result != 1 )  {
      printf( "Error (i2c_write): Did not transmit the byte. \n" );
      return -1;
    }

  }

  // Send data
  else {

    buf[0] = reg_addr;
    for ( i=0; i<length; i++ )  buf[i+1] = data[i];

    if ( fd == imuA.fd )  {
      pthread_mutex_lock(&mutex_i2c1);
      result = write( imuA.fd, buf, length+1 );
      pthread_mutex_unlock(&mutex_i2c1);
    }

    if ( fd == imuB.fd )  {
      pthread_mutex_lock(&mutex_i2c2);
      result = write( imuB.fd, buf, length+1 );
      pthread_mutex_unlock(&mutex_i2c2);
    }

    if ( result < 0 )  {
      printf( "Error (i2c_write): Returned '%d' with 'write' command (%d bytes). \n", result, length );
      printf("errno = %d.\n", errno);
      return result;
    }

    else if ( result < (int) length )  {
      printf( "Error (i2c_write): Only transmitted %d out of %u bytes. \n", result, length );
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

  // Confirm valid fd value
  if ( ( fd != imuA.fd ) && ( fd != imuB.fd ) )  {
    printf( "Error (i2c_write): Improper fd value. \n" );
    return -1;
  }

  int result=0, tries=0, total=0;

  if ( i2c_write( fd, slave_addr, reg_addr, 0, NULL ) )  return -1;

  while ( total < length && tries < 5 )  {

    if ( fd == imuA.fd )  {
      pthread_mutex_lock(&mutex_i2c1);
      result = read( imuA.fd, data + total, length - total );
      pthread_mutex_unlock(&mutex_i2c1);
    }

    if ( fd == imuB.fd )  {
      pthread_mutex_lock(&mutex_i2c2);
      result = read( imuB.fd, data + total, length - total );
      pthread_mutex_unlock(&mutex_i2c2);
    }

    if (result < 0) {
      printf( "Error (i2c_read): Returned a negative value from 'read' command. \n" );
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



