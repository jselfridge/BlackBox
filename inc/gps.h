
//============================================================
//  gps.h
//  Justin M Selfridge
//============================================================
#ifndef _GPS_H_
#define _GPS_H_
#include <main.h>


// Define other packets
#define GPS_DEFAULT      "$PMTK104*37"

// Define buad rate packets
#define GPS_BAUD_57600  "$PMTK251,57600*2C"
#define GPS_BAUD_9600   "$PMTK251,9600*17"

// Define update rate packets
#define GPS_UPDATE_100_MHZ  "$PMTK220,10000*2F"    // 10.0 sec
#define GPS_UPDATE_200_MHZ  "$PMTK220,5000*1B"     //  5.0 sec
#define GPS_UPDATE_001_HZ   "$PMTK220,1000*1F"     //  1.0 sec
#define GPS_UPDATE_005_HZ   "$PMTK220,200*2C"      //  0.2 sec
#define GPS_UPDATE_010_HZ   "$PMTK220,100*2F"      //  0.1 sec

#define GPS_RMCONLY         "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"


// GPS functions
void  gps_init   ( void );
void  gps_exit   ( void );


#endif



