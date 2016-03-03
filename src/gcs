
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

  // Assign UART path
  memset ( gcs.path,      0, sizeof(gcs.path)         );
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

  // Set transmission flags
  gcs.sendhb      = true;
  gcs.sendparam   = false;
  gcs.sendmission = false;

  // Load default parameter values
  strcpy( param.name[X_P], "X_P" );  param.val[X_P] = 10.0;  
  strcpy( param.name[X_I], "X_I" );  param.val[X_I] =  0.1;  
  strcpy( param.name[X_D], "X_D" );  param.val[X_D] =  1.0;  
  strcpy( param.name[Y_P], "Y_P" );  param.val[Y_P] = 20.0;  
  strcpy( param.name[Y_I], "Y_I" );  param.val[Y_I] =  0.2;  
  strcpy( param.name[Y_D], "Y_D" );  param.val[Y_D] =  2.0;  

  // Send initial parameters
  gcs_paramlist();
  gcs_missionlist();

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

  static int count = 0;
  if ( count < 10 )  {  count++;  }
  else               {  count = 0;  gcs.sendhb = true;  }

  // Send as needed  
  if (gcs.sendhb)       gcs_heartbeat();    gcs.sendhb = false;
  if (gcs.sendparam)    gcs_paramlist();    gcs.sendparam = false;
  if (gcs.sendmission)  gcs_missionlist();  gcs.sendmission = false;

  // Send always if enabled
  //if (GCS_EUL_ENABLED)   gcs_eul();
  //if (GCS_QUAT_ENABLED)  gcs_quat();
  //if (GCS_GPS_ENABLED)   gcs_gps();

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
    VEHICLE_ID, 
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
  mavlink_status_t  status;

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

        // ID: #0
        case MAVLINK_MSG_ID_HEARTBEAT:
	  printf("RX: Heartbeat");  
          printf("    %3.1f %3.1f %3.1f ", param.val[0], param.val[1], param.val[2] );
          fflush(stdout);
          // Add fail safe code here
        break;

        // ID: #20
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	  printf("RX: Param Request Read");  fflush(stdout);
	  gcs.sendparam = true;
        break;

        // ID: #21
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	  printf("RX: Param Request List");  fflush(stdout);
	  gcs.sendparam = true;
        break;

        // ID: #23
        case MAVLINK_MSG_ID_PARAM_SET:
	  printf("RX: Param Set");  fflush(stdout);
          gcs_paramupdate(&msg);
        break;

        // ID: #43
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	  printf("RX: Mission Request List");  fflush(stdout);
	  gcs.sendmission = true;
        break;

        // ID: #47
        case MAVLINK_MSG_ID_MISSION_ACK:
	  printf("RX: Mission Acknowledge");  fflush(stdout);
        break;

        // ID: #75
        case MAVLINK_MSG_ID_COMMAND_INT:
	  printf("RX: Command Int");  fflush(stdout);
        break;

        // ID: #76
        case MAVLINK_MSG_ID_COMMAND_LONG:
	  printf("RX: Command Long");  fflush(stdout);
        break;

        // ID: #???
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
void gcs_paramlist ( void )  {

  // Local variables
  int len, w, i;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  for ( i=0; i<PARAM_COUNT; i++ )  {

    // Pack parameter message
    memset( &msg, 0, sizeof(&msg) );
    mavlink_msg_param_value_pack(
      VEHICLE_ID, 
      MAV_COMP_ID_GAINS,
      &msg, 
      param.name[i], 
      param.val[i],
      MAVLINK_TYPE_FLOAT,
      PARAM_COUNT, 
      i
    );

    // Send parameter to GCS
    memset( buf, 0, sizeof(buf) );
    len = mavlink_msg_to_send_buffer( buf, &msg );

    pthread_mutex_lock(&mutex_gcs);
    w = write( gcs.fd, buf, len );
    pthread_mutex_unlock(&mutex_gcs);

    usleep(w*300);

  }

  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_send_param
//  Sends a parameter to the GCS.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_send_param ( enum param_index name, float val )  {
  /*
  // Initialize buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Allocate memory
  memset( &msg, 0, sizeof(&msg) );
  memset( buf,  0, sizeof(buf)  );

  // Pack parameter message
  mavlink_msg_param_value_pack(
    VEHICLE_ID, 
    MAV_COMP_ID_GAINS,
    &msg, 
    param.name[i], 
    param.val[i],
    MAVLINK_TYPE_FLOAT,
    PARAM_COUNT, 
    i
  );

  // Send parameter to GCS
  int len = mavlink_msg_to_send_buffer( buf, &msg );

  // Write to UART
  pthread_mutex_lock(&mutex_gcs);
  int w = write( gcs.fd, buf, len );
  pthread_mutex_unlock(&mutex_gcs);

  // Pause during transmission
  usleep(w*200);
  */
  return;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  gcs_paramupdate
//  Updates the parameter values as needed.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
void gcs_paramupdate ( mavlink_message_t *msg )  {

  uint i, j;
  bool match;
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode( msg, &set );

  // Check if this message is for this system
  if (  (uint8_t) set.target_system    == VEHICLE_ID  && 
        (uint8_t) set.target_component == MAV_COMP_ID_GAINS )  {

    char* key = (char*) set.param_id;

    for ( i=0; i < PARAM_COUNT; i++ )  {

      match = true;

      for ( j=0; j < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN; j++ )  {

        // Compare
        if ( ( (char) (param.name[i][j]) ) != (char) (key[j]) )  {  
          match = false;
        }

        // End matching if null termination is reached
        if ( ( (char) param.name[i][j] ) == '\0' )  {
          break;
        }
      }


      // Check if matched
      if (match)  {

        // Only write and emit changes if there is actually a difference
        // AND only write if new value is NOT "not-a-number"
        // AND is NOT infinity

        if ( param.val[i] != set.param_value && !isnan(set.param_value) && !isinf(set.param_value) && set.param_type == MAVLINK_TYPE_FLOAT )  {

          param.val[i] = set.param_value;

          // Report back new value

          //mavlink_msg_param_value_send(MAVLINK_COMM_0,
          //(int8_t*) global_data.param_name[i],
          //global_data.param[i], MAVLINK_TYPE_FLOAT, 
          //ONBOARD_PARAM_COUNT, m_parameter_i);
          //}

          // Pack parameter message
          mavlink_message_t confirm_msg;
          memset( &confirm_msg, 0, sizeof(&confirm_msg) );
          mavlink_msg_param_value_pack(
            VEHICLE_ID, 
            MAV_COMP_ID_GAINS,
            &confirm_msg, 
            param.name[i], 
            param.val[i],
            MAVLINK_TYPE_FLOAT,
            PARAM_COUNT, 
            i
          );

          // Send parameter to GCS
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];
          memset( buf, 0, sizeof(buf) );
          int len = mavlink_msg_to_send_buffer( buf, &confirm_msg );

          pthread_mutex_lock(&mutex_gcs);
          int w = write( gcs.fd, buf, len );
          pthread_mutex_unlock(&mutex_gcs);

          usleep(w*300);


	}
      }
    }
  }

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
    VEHICLE_ID, 
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
    VEHICLE_ID, 
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





