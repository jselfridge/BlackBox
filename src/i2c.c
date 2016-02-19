
//============================================================
//  i2c.c
//  Justin M Selfridge
//============================================================
#include "i2c.h"




#define MAX_WRITE_LEN 511


//  Eliminate this along with the globals...
//void linux_set_i2c_bus ( int bus )  {
  /*
  if (i2c_fd)
    i2c_close();
  i2c_bus = bus;
  i2c_addr = 0x68;
  */
  //return;
//}









//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_init
//  Assigns a file descriptor after opening an I2C bus. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_init ( int *fd, ushort bus, ushort addr )  {

  char buf[32];
  sprintf( buf, "/dev/i2c-%d", bus );
  *fd = open( buf, O_RDWR );

  if ( *fd < 0 ) {
    printf( "Error (i2c_init): Returned a negative I2C file descriptor. \n" );
    *fd = 0;
    return -1;
  }

  if ( ioctl( *fd, I2C_SLAVE, addr ) < 0 )  {
    printf( "Error (i2c_init): Returned negative value from 'ioctl' command. \n" );
    return -1;
  }

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_exit
//  Closes file descriptor and exits I2C channel.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void i2c_exit ( int *fd )  {
  close(*fd);
  *fd=0;
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_slave
//  Determines the current I2C slave address.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   

int i2c_slave ( unsigned char slave_addr )  {
  /*
  // Eliminate the global variable 'current_slave'...

  if ( current_slave == slave_addr )  return 0;

  if ( i2c_open() )  return -1;

  if ( ioctl( i2c_fd, I2C_SLAVE, slave_addr ) < 0 )  {
    printf( "Error (i2c_slave): Returned negative value from 'ioctl' command. \n" );
    return -1;
  }

  current_slave = slave_addr;
  */
  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_tx
//  Transmit data to the I2C bus.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_tx ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data )  {

  // Local variables
  int  result, i;
  char buf [ MAX_WRITE_LEN+1 ];

  // Check size of data
  if ( length > MAX_WRITE_LEN )  {
    printf( "Error (i2c_tx): Exceeded maximum write length. \n" );
    return -1;
  }

  //if ( i2c_slave(slave_addr) )  return -1;

  // No data to send
  if ( length == 0 )  {
    result = write( imu1.fd, &reg_addr, 1 );    //-- CHANGE imu1.fd --//
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
    result = write( imu1.fd, buf, length+1 );    //-- CHANGE imu1.fd --//
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


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_rx
//  Receive data from the I2C.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_rx ( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data )  {

  int result;
  int tries=0, total=0;

  if ( i2c_write( slave_addr, reg_addr, 0, NULL ) )  return -1;

  while ( total < length && tries < 5 )  {
    result = read( imu1.fd, data + total, length - total );    //-- CHANGE imu1.fd --//
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
//  i2c_get_ms
//  Determines the number of milliseconds since epoch.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
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



