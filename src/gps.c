
//============================================================
//  gps.c
//  Justin M Selfridge
//============================================================
#include "gps.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gps_init
//  Initializes the GPS sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gps_init ( void )  {
  if (DEBUG)  printf("Initializing GPS \n");

  int i;

  //i = write( uart1.fd, GPS_DEFAULT,       sizeof(GPS_DEFAULT)       );  usleep(i*200);  printf("i: %d \n",i);
  //i = write( uart1.fd, GPS_BAUD_57600,    sizeof(GPS_BAUD_57600)    );  usleep(i*200);  printf("i: %d \n",i);
  //i = write( uart1.fd, GPS_UPDATE_200_MHZ, sizeof(GPS_UPDATE_200_MHZ) );  usleep(i*200);  printf("i: %d \n",i);
  i = write( uart1.fd, GPS_RMCONLY, sizeof(GPS_RMCONLY) );  usleep(i*200);  printf("i: %d \n",i);


  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gps_exit
//  Exits the GPS sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gps_exit ( void )  {
  return;
}



