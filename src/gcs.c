

#include "gcs.h"
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include "imu.h"
#include "io.h"
#include "log.h"
#include "stab.h"
#include "sys.h"
#include "timer.h"


static void  gcs_heartbeat    ( void );
static void  gcs_paramlist    ( void );
static void  gcs_missionlist  ( void );

static void  gcs_get_param_value  ( mavlink_message_t *msg );
static void  gcs_set_param_value  ( uint index, double val );

static void  gcs_input        ( void );
static void  gcs_output       ( void );
static void  gcs_att          ( void );


/**
 *  gcs_init
 *  Initializes the GCS communication.
 */
void gcs_init ( void )  {
  if (DEBUG)  printf("Initializing GCS \n");

  // Assign UART path
  if (DEBUG)  printf("  Open UART channel \n");
  memset( gcs.path, 0, sizeof(gcs.path) );
  strcpy( gcs.path, "/dev/ttyO1" );
  gcs.fd = open ( gcs.path, O_RDWR | O_NOCTTY );
  if ( gcs.fd <0 )  printf( "Error (gcs_init): Couldn't open GCS file descriptor. \n" );

  // Assign UART settings
  if (DEBUG)  printf("  Assign UART settings \n");
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

  // Load parameters
  if (DEBUG)  printf( "  Loading %d parameters \n", param_count );

  // Roll (X) stabilization parameters
  strcpy( param.name[X_kp], "X_kp" );  param.val[X_kp] = sfX.kp;
  strcpy( param.name[X_kd], "X_kd" );  param.val[X_kd] = sfX.kd;
  /*
  strcpy( param.name[X_ts], "X_ts" );  param.val[X_ts] = sfX.ts;
  strcpy( param.name[X_mp], "X_mp" );  param.val[X_mp] = sfX.mp;
  strcpy( param.name[X_j ], "X_j"  );  param.val[X_j]  = sfX.j;
  strcpy( param.name[X_Gp], "X_Gp" );  param.val[X_Gp] = sfX.Gp;
  strcpy( param.name[X_Gd], "X_Gd" );  param.val[X_Gd] = sfX.Gd;
  strcpy( param.name[X_Gu], "X_Gu" );  param.val[X_Gu] = sfX.Gu;
  */

  // Pitch (Y) stabilization paramters
  /*
  strcpy( param.name[Y_kp], "Y_kp" );  param.val[Y_kp] = sfY.kp;
  strcpy( param.name[Y_kd], "Y_kd" );  param.val[Y_kd] = sfY.kd;
  strcpy( param.name[Y_ts], "Y_ts" );  param.val[Y_ts] = sfY.ts;
  strcpy( param.name[Y_mp], "Y_mp" );  param.val[Y_mp] = sfY.mp;
  strcpy( param.name[Y_j],  "Y_j"  );  param.val[Y_j]  = sfY.j;
  strcpy( param.name[Y_Gp], "Y_Gp" );  param.val[Y_Gp] = sfY.Gp;
  strcpy( param.name[Y_Gd], "Y_Gd" );  param.val[Y_Gd] = sfY.Gd;
  strcpy( param.name[Y_Gu], "Y_Gu" );  param.val[Y_Gu] = sfY.Gu;
  */

  // Yaw (Z) stabilization parameters
  /*
  strcpy( param.name[Z_kp], "Z_kp" );  param.val[Z_kp] = sfZ.kp;
  strcpy( param.name[Z_kd], "Z_kd" );  param.val[Z_kd] = sfZ.kd;
  strcpy( param.name[Z_ts], "Z_ts" );  param.val[Z_ts] = sfZ.ts;
  strcpy( param.name[Z_mp], "Z_mp" );  param.val[Z_mp] = sfZ.mp;
  strcpy( param.name[Z_j],  "Z_j"  );  param.val[Z_j]  = sfZ.j;
  strcpy( param.name[Z_Gp], "Z_Gp" );  param.val[Z_Gp] = sfZ.Gp;
  strcpy( param.name[Z_Gd], "Z_Gd" );  param.val[Z_Gd] = sfZ.Gd;
  strcpy( param.name[Z_Gu], "Z_Gu" );  param.val[Z_Gu] = sfZ.Gu;
  */

  // Throttle values
  strcpy( param.name[T_min],  "T_min"  );  param.val[T_min]  = stab.thrl[0];
  strcpy( param.name[T_max],  "T_max"  );  param.val[T_max]  = stab.thrl[1];
  strcpy( param.name[T_tilt], "T_tilt" );  param.val[T_tilt] = stab.thrl[2];

  // Range values
  strcpy( param.name[X_Range], "X_Range" );  param.val[X_Range] = stab.range[0];
  strcpy( param.name[Y_Range], "Y_Range" );  param.val[Y_Range] = stab.range[1];
  strcpy( param.name[Z_Range], "Z_Range" );  param.val[Z_Range] = stab.range[2];
  strcpy( param.name[T_Range], "T_Range" );  param.val[T_Range] = stab.range[3];

  // Assign conditional values
  gcs.sendhb      = true;
  gcs.sendparam   = false;
  gcs.sendmission = false;

  // Assign pause for serial stream
  gcs.pause = 10;

  return;
}


/**
 *  gcs_exit
 *  Exits the GCS sensor.
 */
void gcs_exit ( void )  {
  if (DEBUG)  printf("Close GCS \n");
  close (gcs.fd);
  return;
}


/**
 *  gcs_tx
 *  Transmit to the ground control station.
 */
void gcs_tx ( void)  {

  if(GCS_DEBUG)  printf("TX:  ");

  static int count = 0;
  if ( count < HZ_GCSTX )  {  count++;  }
  else                     {  count = 0;  gcs.sendhb = true;  }

  // Send GCS updates
  if (gcs.sendhb)                 gcs_heartbeat();
  if (gcs.sendparam)              gcs_paramlist();
  if (gcs.sendmission)            gcs_missionlist();

  // Send signal values
  if (GCS_INPUT_ENABLED)          gcs_input();
  if (GCS_OUTPUT_ENABLED)         gcs_output();
  if (GCS_ATT_ENABLED)            gcs_att();

  // Print debugging messages
  if(GCS_DEBUG)  {  printf("\n");  fflush(stdout);  }

  return;
}


/**
 *  gcs_rx
 *  Receive from the ground control station.
 */
void gcs_rx ( void)  {

  // Debugging statement
  if(GCS_DEBUG)  printf("RX:  ");

  // Local variables
  mavlink_message_t msg;
  mavlink_status_t  status;
  bool moredata = true;

  // Check for available serial 
  while( moredata )  {

    uint8_t c;

    // Read serial input
    pthread_mutex_lock(&gcs.mutex);
    int r = read( gcs.fd, &c, 1 );
    pthread_mutex_unlock(&gcs.mutex);

    if ( r == 0 )  moredata = false;

    // Try to get a new message
    if ( mavlink_parse_char( 0, c, &msg, &status ) )  {

      // Handle message
      switch(msg.msgid)  {

        // ID: #0
        case MAVLINK_MSG_ID_HEARTBEAT:
          // Reset fail safe timer
	  if (GCS_DEBUG)  printf("(00) Heartbeat  ");
        break;

        // ID: #20
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	  gcs.sendparam = true;
	  if (GCS_DEBUG)  printf("(20) ParamReqRead  ");
        break;

        // ID: #21
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	  gcs.sendparam = true;
	  if (GCS_DEBUG)  printf("(21) ParamReqList  ");
        break;

        // ID: #23
        case MAVLINK_MSG_ID_PARAM_SET:
          gcs_get_param_value(&msg);
          if (datalog.enabled)  log_record(LOG_PARAM);
	  if (GCS_DEBUG)  printf("(23) ParamSet  ");
        break;

        // ID: #43
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	  gcs.sendmission = true;
	  if (GCS_DEBUG)  printf("(43) MisReqList  ");
        break;

        // ID: #47
        case MAVLINK_MSG_ID_MISSION_ACK:
	  // Add "Mission Acknowledge" code...
	  if (GCS_DEBUG)  printf("(47) MisAck  ");
        break;

        // ID: #75
        case MAVLINK_MSG_ID_COMMAND_INT:
	  // Add "Command Int" code...
	  if (GCS_DEBUG)  printf("(75) CmdInt  ");
        break;

        // ID: #76
        case MAVLINK_MSG_ID_COMMAND_LONG:
	  // Add "Command Long" code...
	  if (GCS_DEBUG)  printf("(76) CmdLong  ");
        break;

        // ID: #???
        default:
          // Add unknown ID code...
	  if (GCS_DEBUG)  printf("(?\?) Unknown  ");
        break;

      }
    }
  }

  // Update global packet drops counter
  //packet_drops += status.packet_rx_drop_count;

  // Debugging statement
  if(GCS_DEBUG)  {  printf("\n");  fflush(stdout);  }

  return;
}


/**
 *  gcs_heartbeat
 *  Sends a heartbeat transmission.
 */
static void gcs_heartbeat ( void )  {

  // Debugging statement
  if(GCS_DEBUG)  printf("HB  ");

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the heartbeat message
  mavlink_msg_heartbeat_pack ( 
    GCS_SYSID, 
    GCS_HEARTBEAT,
    &msg, 
    MAV_TYPE_GENERIC,
    MAV_AUTOPILOT_GENERIC,
    MAV_MODE_PREFLIGHT,
    0,
    MAV_STATE_STANDBY
  );

  // Copy the heartbeat message to the send buffer
  int len = mavlink_msg_to_send_buffer( buf, &msg );

  // Send the message 
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  // Reset conditional flag
  gcs.sendhb = false;

  return;
}


/**
 *  gcs_paramlist
 *  Sends the onboard parameter list.
 */
static void gcs_paramlist ( void )  {

  // Debugging statement
  if(GCS_DEBUG)  printf("PL  ");

  // Local variables
  int i;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;

  // Loop through all parameters
  for ( i=0; i < param_count; i++ )  {

    // Pack parameter message
    memset( &msg, 0, sizeof(&msg) );
    mavlink_msg_param_value_pack(
      GCS_SYSID,
      GCS_PARAM,
      &msg, 
      param.name[i], 
      param.val[i],
      MAVLINK_TYPE_FLOAT,
      param_count, 
      i
    );

    // Copy message to buffer
    memset( buf, 0, sizeof(buf) );
    int len = mavlink_msg_to_send_buffer( buf, &msg );

    // Send the message
    pthread_mutex_lock(&gcs.mutex);
    int w = write( gcs.fd, buf, len );
    usleep( w * gcs.pause );
    pthread_mutex_unlock(&gcs.mutex);

  }

  // Reset conditional flag
  gcs.sendparam = false;

  return;
}


/**
 *  gcs_missionlist
 *  Sends the onboard mission list.
 */
static void gcs_missionlist ( void)  {

  // Debugging statement
  if(GCS_DEBUG)  printf("ML  ");

  // Local variables
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;

  // Pack parameter message
  memset( &msg, 0, sizeof(&msg) );
  mavlink_msg_mission_count_pack (
    GCS_SYSID, 
    GCS_MISSION,
    &msg,
    GCS_SYSID, 
    GCS_MISSION, 
    0
  );

  // Copy message to buffer
  memset( buf, 0, sizeof(buf) );
  int len = mavlink_msg_to_send_buffer( buf, &msg );

  // Send the message
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  // Reset conditional flag
  gcs.sendmission = false;

  return;
}


/**
 *  gcs_get_param_value
 *  Obtains an updated parameter value from a message
 */
static void gcs_get_param_value ( mavlink_message_t *msg )  {

  // Local variables
  uint i, j;
  bool match;
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode( msg, &set );

  // Check if this message is for this system
  if (  (uint8_t) set.target_system    == GCS_SYSID  && 
        (uint8_t) set.target_component == GCS_PARAM )  {

    char *key = (char*) set.param_id;

    for ( i=0; i < param_count; i++ )  {

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

	if(GCS_DEBUG)  printf("\nmatched %d \n\n", i);

        // Only write and emit changes if there is actually a difference
        // AND only write if new value is NOT "not-a-number"
        // AND is NOT infinity

        if ( param.val[i] != set.param_value && 
             !isnan(set.param_value) && 
             !isinf(set.param_value) && set.param_type == MAVLINK_TYPE_FLOAT )  {

          param.val[i] = set.param_value;
          gcs_set_param_value( i, param.val[i] );

          // Pack parameter message
          mavlink_message_t confirm_msg;
          memset( &confirm_msg, 0, sizeof(&confirm_msg) );
          mavlink_msg_param_value_pack(
            GCS_SYSID,
            GCS_PARAM,
            &confirm_msg, 
            param.name[i], 
            param.val[i],
            MAVLINK_TYPE_FLOAT,
            param_count, 
            i
          );

          // Send parameter to GCS
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];
          memset( buf, 0, sizeof(buf) );
          int len = mavlink_msg_to_send_buffer( buf, &confirm_msg );

          // Write out to GCS
          pthread_mutex_lock(&gcs.mutex);
          int w = write( gcs.fd, buf, len );
          usleep( w * gcs.pause );
          pthread_mutex_unlock(&gcs.mutex);

	}
      }
    }
  }

  return;
}


/**
 *  gcs_set_param_value
 *  Assigns an updated parameter value to a structure
 */
static void gcs_set_param_value ( uint index, double val )  {

  // Jump to parameter
  switch (index)  {

  // Roll (X) Stabilization Parameters
  case X_kp :
    pthread_mutex_lock(&sfX.mutex);
    sfX.kp = val;
    pthread_mutex_unlock(&sfX.mutex);
    break;
  case X_kd :
    pthread_mutex_lock(&sfX.mutex);
    sfX.kd = val;
    pthread_mutex_unlock(&sfX.mutex);
    break;
  /*
  case X_ts :
    pthread_mutex_lock(&sfX.mutex);
    sfX.ts = val;
    pthread_mutex_unlock(&sfX.mutex);
    stab_refmdl( &sfX );
    break;
  case X_mp :
    pthread_mutex_lock(&sfX.mutex);
    sfX.mp = val;
    pthread_mutex_unlock(&sfX.mutex);
    stab_refmdl( &sfX );
    break;
  case X_j :
    pthread_mutex_lock(&sfX.mutex);
    sfX.j = val;
    pthread_mutex_unlock(&sfX.mutex);
    stab_refmdl( &sfX );
    break;
  case X_Gp :
    pthread_mutex_lock(&sfX.mutex);
    sfX.Gp = val;
    pthread_mutex_unlock(&sfX.mutex);
    break;
  case X_Gd :
    pthread_mutex_lock(&sfX.mutex);
    sfX.Gd = val;
    pthread_mutex_unlock(&sfX.mutex);
    break;
  case X_Gu :
    pthread_mutex_lock(&sfX.mutex);
    sfX.Gu = val;
    pthread_mutex_unlock(&sfX.mutex);
    break;
  */


  // Pitch (Y) Stabilization Parameters
  /*
  case Y_kp :
    pthread_mutex_lock(&sfY.mutex);
    sfY.kp = val;
    pthread_mutex_unlock(&sfY.mutex);
    break;
  case Y_kd :
    pthread_mutex_lock(&sfY.mutex);
    sfY.kd = val;
    pthread_mutex_unlock(&sfY.mutex);
    break;
  case Y_ts :
    pthread_mutex_lock(&sfY.mutex);
    sfY.ts = val;
    pthread_mutex_unlock(&sfY.mutex);
    stab_refmdl( &sfY );
    break;
  case Y_mp :
    pthread_mutex_lock(&sfY.mutex);
    sfY.mp = val;
    pthread_mutex_unlock(&sfY.mutex);
    stab_refmdl( &sfY );
    break;
  case Y_j :
    pthread_mutex_lock(&sfY.mutex);
    sfY.j = val;
    pthread_mutex_unlock(&sfY.mutex);
    stab_refmdl( &sfY );
    break;
  case Y_Gp :
    pthread_mutex_lock(&sfY.mutex);
    sfY.Gp = val;
    pthread_mutex_unlock(&sfY.mutex);
    break;
  case Y_Gd :
    pthread_mutex_lock(&sfY.mutex);
    sfY.Gd = val;
    pthread_mutex_unlock(&sfY.mutex);
    break;
  case Y_Gu :
    pthread_mutex_lock(&sfY.mutex);
    sfY.Gu = val;
    pthread_mutex_unlock(&sfY.mutex);
    break;
  */


  // Yaw (Z) Stabilization Parameters
  /*
  case Z_kp :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.kp = val;
    pthread_mutex_unlock(&sfZ.mutex);
    break;
  case Z_kd :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.kd = val;
    pthread_mutex_unlock(&sfZ.mutex);
    break;
  case Z_ts :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.ts = val;
    pthread_mutex_unlock(&sfZ.mutex);
    stab_refmdl( &sfZ );
    break;
  case Z_mp :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.mp = val;
    pthread_mutex_unlock(&sfZ.mutex);
    stab_refmdl( &sfZ );
    break;
  case Z_j :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.j = val;
    pthread_mutex_unlock(&sfZ.mutex);
    stab_refmdl( &sfZ );
    break;
  case Z_Gp :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.Gp = val;
    pthread_mutex_unlock(&sfZ.mutex);
    break;
  case Z_Gd :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.Gd = val;
    pthread_mutex_unlock(&sfZ.mutex);
    break;
  case Z_Gu :
    pthread_mutex_lock(&sfZ.mutex);
    sfZ.Gu = val;
    pthread_mutex_unlock(&sfZ.mutex);
    break;
  */


  // Throttle settings
  case T_min :
    pthread_mutex_lock(&stab.mutex);
    stab.thrl[0] = val;
    pthread_mutex_unlock(&stab.mutex);
    break;
  case T_max :
    pthread_mutex_lock(&stab.mutex);
    stab.thrl[1] = val;
    pthread_mutex_unlock(&stab.mutex);
    break;
  case T_tilt :
    pthread_mutex_lock(&stab.mutex);
    stab.thrl[2] = val;
    pthread_mutex_unlock(&stab.mutex);
    break;

  // Range settings
  case X_Range :
    pthread_mutex_lock(&stab.mutex);
    stab.range[0] = val;
    pthread_mutex_unlock(&stab.mutex);
    break;
  case Y_Range :
    pthread_mutex_lock(&stab.mutex);
    stab.range[1] = val;
    pthread_mutex_unlock(&stab.mutex);
    break;
  case Z_Range :
    pthread_mutex_lock(&stab.mutex);
    stab.range[2] = val;
    pthread_mutex_unlock(&stab.mutex);
    break;
  case T_Range :
    pthread_mutex_lock(&stab.mutex);
    stab.range[3] = val;
    pthread_mutex_unlock(&stab.mutex);
    break;


  default :
    break;

  }

  return;
}


/**
 *  gcs_input
 *  Sends the radio input commands.
 */
static void gcs_input ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  uint8_t  port = 1;
  uint8_t  rssi = 255;

  pthread_mutex_lock(&input.mutex);
  int16_t ch1 = (int16_t) ( input.norm[CH1] * 10000 );
  int16_t ch2 = (int16_t) ( input.norm[CH2] * 10000 );
  int16_t ch3 = (int16_t) ( input.norm[CH3] * 10000 );
  int16_t ch4 = (int16_t) ( input.norm[CH4] * 10000 );
  int16_t ch5 = (int16_t) ( input.norm[CH5] * 10000 );
  int16_t ch6 = (int16_t) ( input.norm[CH6] * 10000 );
  int16_t ch7 = (int16_t) ( input.norm[CH7] * 10000 );
  int16_t ch8 = (int16_t) ( input.norm[CH8] * 10000 );
  pthread_mutex_unlock(&input.mutex);

  // Pack the attitude message 
  mavlink_msg_rc_channels_scaled_pack ( 
    GCS_SYSID, GCS_INPUT, &msg, time_boot_ms, port, 
    ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, rssi
    );

  // Copy the heartbeat message to the send buffer
  int len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  return;
}


/**
 *  gcs_output
 *  Sends the system output commands.
 */
static void gcs_output ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_usec = 0;
  uint8_t  port = 1;

  pthread_mutex_lock(&output.mutex);
  int16_t ch1 = (int16_t) ( output.pwm[CH1] );
  int16_t ch2 = (int16_t) ( output.pwm[CH2] );
  int16_t ch3 = (int16_t) ( output.pwm[CH3] );
  int16_t ch4 = (int16_t) ( output.pwm[CH4] );
  int16_t ch5 = (int16_t) ( output.pwm[CH5] );
  int16_t ch6 = (int16_t) ( output.pwm[CH6] );
  int16_t ch7 = (int16_t) ( output.pwm[CH7] );
  int16_t ch8 = (int16_t) ( output.pwm[CH8] );
  pthread_mutex_unlock(&output.mutex);

  // Pack the attitude message 
  mavlink_msg_servo_output_raw_pack ( 
    GCS_SYSID, GCS_OUTPUT, &msg, time_usec, port, 
    ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8
    );

  // Copy the heartbeat message to the send buffer
  int len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  return;
}


/**
 *  gcs_att
 *  Sends the attitude states of the system.
 */
static void gcs_att ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&rot.mutex);
  float roll       = (float) rot.att[0];
  float pitch      = (float) rot.att[1];
  float yaw        = (float) rot.att[2];
  float rollspeed  = (float) rot.ang[0];
  float pitchspeed = (float) rot.ang[1];
  float yawspeed   = (float) rot.ang[2];
  pthread_mutex_unlock(&rot.mutex);

  // Pack the attitude message 
  mavlink_msg_attitude_pack ( 
    GCS_SYSID, GCS_ATT, &msg, time_boot_ms, 
    roll, pitch, yaw, 
    rollspeed, pitchspeed, yawspeed
    );

  // Copy the heartbeat message to the send buffer
  int len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  return;
}


/**
 *  gcs_gps
 *  Sends the GPS location data.
 */
/*static void gcs_gps ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint64_t time_usec = 0;
  pthread_mutex_lock(&gps.mutex);
  uint8_t  fix =  3;
  int32_t  lat =  370349210;
  int32_t  lon = -764677960;
  int32_t  alt =  3000;
  uint16_t eph = UINT16_MAX;
  uint16_t epv = UINT16_MAX;
  uint16_t vel = 100;
  uint16_t cog = 31400;
  uint8_t  num = 6;
  pthread_mutex_unlock(&gps.mutex);

  // Pack the attitude message 
  mavlink_msg_gps_raw_int_pack ( 
    GCS_SYSID, GCS_OUTPUT, &msg, time_usec,
    fix, lat, lon, alt, 
    eph, epv, vel, cog, num
  );

  // Copy the heartbeat message to the send buffer
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  return;
}
*/


