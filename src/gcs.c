
//============================================================
//  gcs.c
//  Justin M Selfridge
//============================================================
#include "gcs.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_init
//  Initializes the GCS communication.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_init ( void )  {
  if (DEBUG)  printf("Initializing GCS \n");

  //sprintf( gps.path, "/dev/ttyO1" );
  strcpy( gcs.path, "/dev/ttyO2" );
  memset ( gcs.msg, 0, sizeof(gcs.msg) );

  // Open the file descriptor
  gcs.fd = open ( gcs.path, O_RDWR | O_NOCTTY );
  if ( gcs.fd <0 )  printf( "Error (gcs_init): Couldn't open GCS file descriptor. \n" );

  // Get current (old) parameters
  //struct termios oldparam;
  //memset( &oldparam, 0, sizeof( &oldparam ) );

  // Assign UART settings
  struct termios settings;
  memset( &settings, 0, sizeof( &settings ) );
  settings.c_iflag     = 0;
  settings.c_oflag     = 0;
  settings.c_cflag     = CS8 | CREAD | CLOCAL;
  settings.c_lflag     = 0;
  settings.c_cc[VTIME] = 1;
  settings.c_cc[VMIN]  = 10;

  // Set input baud rate
  if ( cfsetispeed( &settings, B57600 ) <0 )
    printf( "Error (gcs_init): Couldn't set GCS input buad rate. \n" );

  // Set output baud rate
  if ( cfsetospeed( &settings, B57600 ) <0 )
    printf( "Error (gcs_init): Couldn't set GCS output buad rate. \n" );

  // Assign parameters to device
  if ( tcsetattr( gcs.fd, TCSAFLUSH, &settings ) <0 )
    printf( "Error (gcs_init): Failed to assign GCS UART parameters. \n" );

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_exit
//  Exits the GCS sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_exit ( void )  {
  // Add code as needed...
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_tx
//  Transmit to the ground control station.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_tx ( void)  {
  
  /*
  static uint loop = 0;  loop++;
  printf( "GCS Loop: %d \n\n", loop );
  char debugmsg[64];
  memset( debugmsg,0,sizeof(debugmsg) );
  sprintf( debugmsg, "GCS Loop: %d \r\n", loop );
  int w = write( gcs.fd, debugmsg, 64 );
  usleep(w*200);
  */

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // The default UART header for your MCU
  //#include "uart.h"
  //#include <mavlink/v1.0/common/mavlink.h>
 
  mavlink_system_t mavlink_system;
 
  mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
  mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
 
  // Define the system type, in this case an airplane
  uint system_type     = MAV_TYPE_FIXED_WING;
  uint autopilot_type  = MAV_AUTOPILOT_GENERIC;
 
  uint system_mode     = MAV_MODE_PREFLIGHT; ///< Booting up
  uint custom_mode     = 0;                 ///< Custom mode, can be defined by user/adopter
  uint system_state    = MAV_STATE_STANDBY; ///< System ready for flight
 
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack ( mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state );
 
  // Copy the message to the send buffer
  uint len = mavlink_msg_to_send_buffer( buf, &msg );
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  //uart0_send(buf, len);
  int w = write( gcs.fd, buf, len );
  usleep(w*200);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_rx
//  Receive from the ground control station.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_rx ( void)  {
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gps_update
//  Obtains a new set of GPS data.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
/*
void gps_update ( void)  {


  uint sum = 0;
  uint i;
  bool valid = false;
  char msg[96];    memset( msg, 0, sizeof(msg) );

  read( gps.fd, msg, sizeof(msg) );
  tcflush( gps.fd, TCIOFLUSH ); 

  // Debugging message
  //strcpy( msg, "$GPRMC,233325.200,A,3702.0833,N,07628.0617,W,1.74,217.31,240216,,,A*7A\r\n" );    // 7A
  //sprintf( msg, "$GPRMC,233325.200,A,3702.0833,N,07628.0617,W,1.74,217.31,240216,,,A*7A\r\n" );    // 7A

  int len = strlen(msg);

  // Evaluate message
  if ( msg[len-5] == '*' )  {

    // Obtain checksum
    sum  = gps_hex2dec ( msg[len-4] ) * 16;
    sum += gps_hex2dec ( msg[len-3] );

    // Loop through content
    for ( i=1; i < (len-5); i++ )  {
      sum ^= msg[i];
    }

    // Evaluate check sum
    if (sum == 0)   valid = true;

  }

  // Assign message
  if (valid)  msg[len-2] = '\0';
  else        sprintf( msg, "Bad data" );

  // Assign GPS message
  pthread_mutex_lock(&mutex_gps);
  sprintf( gps.msg, msg );
  pthread_mutex_unlock(&mutex_gps);

  return;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gps_hex2dec
//  Converts a hex character into an integer
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
/*
uint gps_hex2dec ( char c )  {
  if ( c <  '0' )  return 0;
  if ( c <= '9' )  return c - '0';
  if ( c <  'A' )  return 0;
  if ( c <= 'F' )  return (c - 'A')+10;
  return 0;
}
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gps_adafruit
//  Temp code obtained from Adafruit sample.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
/*
void gps_adafruit ( void )  {

  // Local variables
  uint sum, i;
  long degree, minutes;
  char degreebuff[10];
  double latitudeDegrees;
  double longitudeDegrees;

  // Testing buffer
  char buf[96] = "$GPRMC,233325.200,A,3702.0833,N,07628.0617,W,1.74,217.31,240216,,,A*7A\r\n";
  //char buf[96] = "$GPRMC,233325.200";
  printf("buf:  %s \n", buf);

  // Get array length
  int len = strlen(buf);
  printf("len: %d \n", len );

  // Star char must be present
  if ( buf[len-5] == '*' )   printf("Found it!\n");
  else                       printf("Keep looking!\n");

  // Obtain checksum
  sum  = gps_hex2dec ( buf[len-4] ) * 16;
  sum += gps_hex2dec ( buf[len-3] );
  printf( "sum: %2d \n", sum );

  // Loop through content
  for ( i=1; i < (len-5); i++ )  {
    sum ^= buf[i];
  }

  // Evaluate check sum
  if (sum != 0)  printf("Bad check sum \n");
  else           printf("Success!! \n");

  // Process RMC data
  if ( strstr( buf, "$GPRMC" ) )  {

    // found RMC
    char *p = buf;
    printf("Found the GPRMC string. \n");

    // Get GPS HMS
    p = strchr( p, ',' ) +1;
    char timestr[6];
    memcpy( &timestr, p, 6 );
    uint timeint = atoi(timestr);

    // Get GPS ms
    p = strchr( p, '.' ) +1;
    char msstr[3];
    memcpy( &msstr, p, 3 );
    uint ms = atoi(msstr);

    // Calculate time
    uint hr  = timeint / 10000;
    uint min = (timeint % 10000) / 100;
    uint sec = (timeint % 100);

    // Display time results
    printf("timeint: %d \n", timeint );
    printf("time:  %2d:%2d:%2d.%3d \n", hr, min, sec, ms );

    // GPS fix status
    //bool fix;
    p = strchr( p, ',' )+1;
    if ( p[0] == 'A' )  {  
      //fix = true;
      printf("Good fix \n");
    }
    else if ( p[0] == 'V' )  {
      //fix = false;
      printf("No fix \n");
    }
    else  {
      //fix = false;
      printf("Bad status \n");
    }

    // Parse GPS latitude
    p = strchr( p, ',' )+1;
    if ( ',' != *p )  {
      strncpy(degreebuff, p, 2);
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      printf( "degree: %ld \n", degree );
      p += 2;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      printf("degbuff: %s \n", degreebuff );
      minutes = 50 * atol(degreebuff) / 3;
      printf("minutes: %ld \n", minutes);
      long latitude_fixed = degree + minutes;
      printf("latitude_fixed: %ld \n", latitude_fixed );
      double latitude = degree / 100000 + minutes * 0.000006F;
      printf("latitude: %f \n", latitude );
      latitudeDegrees = ( latitude- 100* (int)(latitude/100) ) /60.0;
      latitudeDegrees += (int)(latitude/100);
      printf("latitudeDegrees: %f \n", latitudeDegrees );
    }

    // Convert N/S
    p = strchr( p, ',' )+1;
    char lat = 'O';
    if ( ',' != *p )  {
      if      ( p[0] == 'S' )  latitudeDegrees *= -1.0;
      if      ( p[0] == 'N' )  lat = 'N';
      else if ( p[0] == 'S' )  lat = 'S';
      else if ( p[0] == ',' )  lat = '0';
      else                     lat = '?';
    }
    printf("lat: %c \n", lat);

    // Parse GPS longitude
    p = strchr( p, ',' )+1;
    if ( ',' != *p )  {
      strncpy( degreebuff, p, 3 );
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      printf( "degree: %ld \n", degree );
      p += 3;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      printf("degbuff: %s \n", degreebuff );
      minutes = 50 * atol(degreebuff) / 3;
      printf("minutes: %ld \n", minutes);
      long longitude_fixed = degree + minutes;
      printf("longitude_fixed: %ld \n", longitude_fixed );
      double longitude = degree / 100000 + minutes * 0.000006F;
      printf("longitude: %f \n", longitude );
      longitudeDegrees = ( longitude - 100* (int)(longitude/100) ) /60.0;
      longitudeDegrees += (int)(longitude/100);
      printf("longitudeDegrees: %f \n", longitudeDegrees );
    }

    // Convert E/W
    p = strchr(p, ',')+1;
    char lon = 'O';
    if ( ',' != *p )  {
      if      ( p[0] == 'W' )  longitudeDegrees *= -1.0;
      if      ( p[0] == 'W' )  lon = 'W';
      else if ( p[0] == 'E' )  lon = 'E';
      else if ( p[0] == ',' )  lon = '0';
      else                     lon = '?';
    }
    printf( "lon: %c \n", lon );

    // Speed
    p = strchr( p, ',' )+1;
    if ( ',' != *p )  {  double speed = atof(p);  printf( "speed: %f \n", speed );  }
    
    // Angle
    p = strchr( p, ',' )+1;
    if ( ',' != *p )  {  double angle = atof(p);  printf( "angle: %f \n", angle );  }

    // Date
    p = strchr( p, ',' )+1;
    if ( ',' != *p )  {
      ulong fulldate = atof(p);
      ulong day = fulldate / 10000;
      ulong month = (fulldate % 10000) / 100;
      ulong year = (fulldate % 100);
      printf("date: %2ld:%2ld:%2ld \n", month, day, year );
    }

  }

  return;
}
*/


