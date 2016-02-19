
//============================================================
//  i2c.c
//  Justin M Selfridge
//============================================================
#include "i2c.h"




#define MAX_WRITE_LEN 511

//int i2c_bus = 1;
//int i2c_fd;
int current_slave;
unsigned char txBuff[MAX_WRITE_LEN + 1];





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_xxxx
//  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_open (  )  {

  char buff[32];

  if (!i2c_fd) {
    sprintf(buff, "/dev/i2c-%d", i2c_bus);

    //#ifdef I2C_DEBUG
    //printf("\t\t\ti2c_open() : %s\n", buff);
    //#endif

    i2c_fd = open( buff, O_RDWR );
    if ( i2c_fd < 0 ) {
      perror("open(i2c_bus)");
      i2c_fd = 0;
      return -1;
    }
  }

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_xxxx
//  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void i2c_close (  )  {
  if (i2c_fd) {
    close(i2c_fd);
    i2c_fd = 0;
    current_slave = 0;
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_xxxx
//  
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
//  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void linux_set_i2c_bus ( int bus )  {
  if (i2c_fd)
    i2c_close();
  i2c_bus = bus;
  i2c_addr = 0x68;
  addr = 0x68;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_xxxx
//  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_write ( uint fd, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data )  {

  int result, i;

  if ( length > MAX_WRITE_LEN )  {
    printf("Max write length exceeded in linux_i2c_write()\n");
    return -1;
  }

  //#ifdef I2C_DEBUG
  //printf("\tlinux_i2c_write(%02X, %02X, %u, [", slave_addr, reg_addr, length);
  //if (length == 0) {
  //  printf(" NULL ] )\n");
  //}
  //else {
  //  for (i = 0; i < length; i++)
  //    printf(" %02X", data[i]); 
  //  printf(" ] )\n");
  //}
  //#endif

  if ( i2c_select_slave(slave_addr) )
    return -1;

  if ( length == 0 )  {
    result = write( fd, &reg_addr, 1 );  // result = write( i2c_fd, &reg_addr, 1 );

    if ( result < 0 )  {
      perror("write:1");
      return result;
    }
    else if ( result != 1 )  {
      printf("Write fail:1 Tried 1 Wrote 0\n");
      return -1;
    }
  }
  else {
    txBuff[0] = reg_addr;

    for ( i=0; i<length; i++ )
      txBuff[i+1] = data[i];

    result = write( fd, txBuff, length+1 );  // result = write( i2c_fd, txBuff, length+1 );

    if ( result < 0 )  {
      perror("write:2");
      return result;
    }
    else if ( result < (int)length )  {
      printf("Write fail:2 Tried %u Wrote %d\n", length, result); 
      return -1;
    }
  }

  return 0;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  i2c_xxxx
//  
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
int i2c_read ( uint fd, unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data )  {

  int tries, result, total;

  //#ifdef I2C_DEBUG
  //int i;
  //printf("\tlinux_i2c_read(%02X, %02X, %u, ...)\n", slave_addr, reg_addr, length);
  //#endif

  if ( i2c_write( fd, slave_addr, reg_addr, 0, NULL ) )  // if ( i2c_write( slave_addr, reg_addr, 0, NULL ) )
    return -1;

  total = 0;
  tries = 0;

  while (total < length && tries < 5) {
    result = read( fd, data + total, length - total);  // result = read(i2c_fd, data + total, length - total);

    if (result < 0) {
      perror("read");
      break;
    }

    total += result;

    if (total == length)
      break;

    tries++;		
    usleep(10000);
  }

  if (total < length)
    return -1;

  //#ifdef I2C_DEBUG
  //printf("\tLeaving linux_i2c_read(), read %d bytes: ", total);
  //for (i = 0; i < total; i++)
  //  printf("%02X ", data[i]); 
  //printf("\n");
  //#endif

  return 0;
}



