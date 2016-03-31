

#ifndef _GPS_H_
#define _GPS_H_
#include <main.h>


// Define GPS buad rates
#define GPS_BAUD_57600  "$PMTK251,57600*2C"
#define GPS_BAUD_9600   "$PMTK251,9600*17"


// Define GPS update rates
#define GPS_UPDATE_100_MHZ  "$PMTK220,10000*2F\r\n"    // 10.0 sec
#define GPS_UPDATE_200_MHZ  "$PMTK220,5000*1B\r\n"     //  5.0 sec
#define GPS_UPDATE_001_HZ   "$PMTK220,1000*1F\r\n"     //  1.0 sec
#define GPS_UPDATE_005_HZ   "$PMTK220,200*2C\r\n"      //  0.2 sec
#define GPS_UPDATE_010_HZ   "$PMTK220,100*2F\r\n"      //  0.1 sec


// Define GPS data types
#define GPS_RMCONLY         "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define GPS_GGAONLY         "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define GPS_RMCGGA          "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"


// GPS structure
typedef struct gps_struct {
  int     fd;
  char    path [16];
  char    msg  [96];
  double  lat;
  double  lon;
  double  alt;
  double  heading;
  double  speed;
  uint    numsat;
} gps_struct;
gps_struct gps;


// GPS functions
void  gps_init     ( void );
void  gps_exit     ( void );
void  gps_update   ( void );
uint  gps_hex2dec  ( char c );
void  gps_adafruit ( void );


#endif



