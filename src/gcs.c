
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

  // Allocate memory
  memset ( gcs.path,      0, sizeof(gcs.path)         );
  //memset ( gcs.msg,       0, sizeof(gcs.msg)          );
  //memset ( &(gcs.system), 0, sizeof(mavlink_system_t) );

  // Assign UART path
  strcpy( gcs.path, "/dev/ttyO2" );

  // Open the file descriptor
  gcs.fd = open ( gcs.path, O_RDWR | O_NOCTTY );
  if ( gcs.fd <0 )  printf( "Error (gcs_init): Couldn't open GCS file descriptor. \n" );

  // Assign UART settings
  struct termios settings;
  memset( &settings, 0, sizeof( &settings ) );
  settings.c_iflag     = 0;
  settings.c_oflag     = 0;
  settings.c_cflag     = CS8 | CREAD | CLOCAL;
  settings.c_lflag     = 0;
  settings.c_cc[VTIME] = 0;
  settings.c_cc[VMIN]  = 0;

  // Set input baud rate
  if ( cfsetispeed( &settings, B57600 ) <0 )
    printf( "Error (gcs_init): Couldn't set GCS input buad rate. \n" );

  // Set output baud rate
  if ( cfsetospeed( &settings, B57600 ) <0 )
    printf( "Error (gcs_init): Couldn't set GCS output buad rate. \n" );

  // Assign parameters to device
  if ( tcsetattr( gcs.fd, TCSAFLUSH, &settings ) <0 )
    printf( "Error (gcs_init): Failed to assign GCS UART parameters. \n" );

  gcs.sendparam = false;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_exit
//  Exits the GCS sensor.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_exit ( void )  {
  close ( gcs.fd );
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_tx
//  Transmit to the ground control station.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_tx ( void)  {
  
  gcs_heartbeat();

  //pthread_mutex_lock(&mutex_gcs);
  if (gcs.sendparam)  gcs_paramlist();  gcs.sendparam = false;
  //pthread_mutex_unlock(&mutex_gcs);

  //pthread_mutex_lock(&mutex_gcs);
  if (gcs.sendmission)  gcs_missionlist();  gcs.sendmission = false;
  //pthread_mutex_unlock(&mutex_gcs);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  /*  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&mutex_eul);
  float roll       = ahrs.eul[0];
  float pitch      = ahrs.eul[1];
  float yaw        = ahrs.eul[2];
  float rollspeed  = ahrs.deul[0];
  float pitchspeed = ahrs.deul[1];
  float yawspeed   = ahrs.deul[2];
  pthread_mutex_unlock(&mutex_eul);

  // Pack the attitude message 
  mavlink_msg_attitude_pack ( 
    20, 
    MAV_COMP_ID_IMU,
    &msg,  
    time_boot_ms, 
    roll, 
    pitch, 
    yaw, 
    rollspeed, 
    pitchspeed, 
    yawspeed
    );

  // Copy the heartbeat message to the send buffer
  uint attlen = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  int attw = write( gcs.fd, buf, attlen );
  usleep(attw*300);

  */

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_rx
//  Receive from the ground control station.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_rx ( void)  {

  bool moredata = true;

  // Local mavlink variables
  mavlink_message_t msg;
  mavlink_status_t status;

  // Check for available serial 
  while( moredata )  {

    uint8_t c;

    pthread_mutex_lock(&mutex_gcs);
    int r = read( gcs.fd, &c, 1 );
    pthread_mutex_unlock(&mutex_gcs);

    if ( r == 0 )  moredata = false;

    // Try to get a new message
    if ( mavlink_parse_char( 0, c, &msg, &status ) )  {

      printf("\n");

      // Handle message
      switch(msg.msgid)  {

        case MAVLINK_MSG_ID_HEARTBEAT:
	  printf("RX: Heartbeat");  fflush(stdout);
        break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	  printf("RX: Param Request List");  fflush(stdout);
          //pthread_mutex_lock(&mutex_gcs);
	  gcs.sendparam = true;
          //pthread_mutex_unlock(&mutex_gcs);
        break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	  printf("RX: Param Request Read");  fflush(stdout);
          //pthread_mutex_lock(&mutex_gcs);
	  gcs.sendparam = true;
          //pthread_mutex_unlock(&mutex_gcs);
        break;

        case MAVLINK_MSG_ID_PARAM_SET:
	  printf("RX: Param Set");  fflush(stdout);
        break;

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	  printf("RX: Mission Request List");  fflush(stdout);
          //pthread_mutex_lock(&mutex_gcs);
	  gcs.sendmission = true;
          //pthread_mutex_unlock(&mutex_gcs);
        break;

        case MAVLINK_MSG_ID_COMMAND_LONG:
	  printf("RX: Command Long");  fflush(stdout);
        break;

        default:
          printf("RX: New ID: %d ", msg.msgid );  fflush(stdout);
        break;

      }
    }
  }

  // Update global packet drops counter
  //packet_drops += status.packet_rx_drop_count;

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_paramlist
//  Sends the onboard parameter list.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_paramlist ( void)  {

  //printf("\nTX: Param List " );  fflush(stdout);

  // Local variables
  int len, w;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack parameter message
  memset( &msg, 0, sizeof(&msg) );
  mavlink_msg_param_value_pack(
    20, 
    MAV_COMP_ID_IMU,
    &msg, 
    "ParamA", 
    1.0, 
    MAVLINK_TYPE_FLOAT,
    1, 
    0
  );

  // Send parameter to GCS
  memset( buf, 0, sizeof(buf) );
  len = mavlink_msg_to_send_buffer( buf, &msg );

  pthread_mutex_lock(&mutex_gcs);
  w = write( gcs.fd, buf, len );
  pthread_mutex_unlock(&mutex_gcs);

  usleep(w*300);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_missionlist
//  Sends the onboard mission list.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_missionlist ( void)  {

  printf("\nTX: Mission List " );  fflush(stdout);

  // Local variables
  int len, w;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack parameter message
  memset( &msg, 0, sizeof(&msg) );

  mavlink_msg_mission_count_pack (
    20, 
    MAV_COMP_ID_IMU, 
    &msg,
    0, 
    0, 
    0
  );

  // Send parameter to GCS
  memset( buf, 0, sizeof(buf) );
  len = mavlink_msg_to_send_buffer( buf, &msg );

  pthread_mutex_lock(&mutex_gcs);
  w = write( gcs.fd, buf, len );
  pthread_mutex_unlock(&mutex_gcs);

  usleep(w*300);

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_heartbeat
//  Sends a heartbeat transmission.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_heartbeat ( void)  {

  //printf("\nTX: Heartbeat " );  fflush(stdout);

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the heartbeat message
  mavlink_msg_heartbeat_pack ( 
    20, 
    MAV_COMP_ID_IMU,
    &msg, 
    MAV_TYPE_FIXED_WING,
    MAV_AUTOPILOT_GENERIC,
    MAV_MODE_PREFLIGHT,
    0,
    MAV_STATE_STANDBY
  );
 
  // Copy the heartbeat message to the send buffer
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Send the message 
  pthread_mutex_lock(&mutex_gcs);
  int w = write( gcs.fd, buf, len );
  pthread_mutex_unlock(&mutex_gcs);

  usleep(w*300);

  return;
}





