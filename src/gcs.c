

#include "gcs.h"
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include "ahrs.h"
#include "ctrl.h"
#include "imu.h"
#include "io.h"
#include "log.h"
#include "lpf.h"
#include "sys.h"
#include "timer.h"


static void  gcs_heartbeat    ( void );
static void  gcs_paramlist    ( void );
static void  gcs_missionlist  ( void );

static void  gcs_param_value  ( mavlink_message_t *msg );

static void  gcs_input        ( void );
static void  gcs_output       ( void );
static void  gcs_imuA_raw     ( void );
static void  gcs_imuA_scaled  ( void );
static void  gcs_imuA_filter  ( void );
static void  gcs_imuB_raw     ( void );
static void  gcs_imuB_scaled  ( void );
static void  gcs_imuB_filter  ( void );
//static void  gcs_ahrs_eul     ( void );
//static void  gcs_ahrs_quat    ( void );
//static void  gcs_gps          ( void );


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

  // LPF cutoff frequency
  strcpy( param.name[lpf_freq_gyr], "lpf_freq_gyr" );  param.val[lpf_freq_gyr] = LPF_FREQ_GYR;
  strcpy( param.name[lpf_freq_acc], "lpf_freq_acc" );  param.val[lpf_freq_acc] = LPF_FREQ_ACC;
  strcpy( param.name[lpf_freq_mag], "lpf_freq_mag" );  param.val[lpf_freq_mag] = LPF_FREQ_MAG;

  // LPF sample history
  strcpy( param.name[lpf_hist_gyr], "lpf_hist_gyr" );  param.val[lpf_hist_gyr] = LPF_HIST_GYR;
  strcpy( param.name[lpf_hist_acc], "lpf_hist_acc" );  param.val[lpf_hist_acc] = LPF_HIST_ACC;
  strcpy( param.name[lpf_hist_mag], "lpf_hist_mag" );  param.val[lpf_hist_mag] = LPF_HIST_MAG;

  // Roll gains
  strcpy( param.name[X_Kp], "X_Kp" );  param.val[X_Kp] = QUAD_PX;
  strcpy( param.name[X_Ki], "X_Ki" );  param.val[X_Ki] = QUAD_IX;
  strcpy( param.name[X_Kd], "X_Kd" );  param.val[X_Kd] = QUAD_DX;

  // Pitch gains
  strcpy( param.name[Y_Kp], "Y_Kp" );  param.val[Y_Kp] = QUAD_PY;
  strcpy( param.name[Y_Ki], "Y_Ki" );  param.val[Y_Ki] = QUAD_IY;
  strcpy( param.name[Y_Kd], "Y_Kd" );  param.val[Y_Kd] = QUAD_DY;

  // Yaw gains
  strcpy( param.name[Z_Kp], "Z_Kp" );  param.val[Z_Kp] = QUAD_PZ;
  strcpy( param.name[Z_Ki], "Z_Ki" );  param.val[Z_Ki] = QUAD_IZ;
  strcpy( param.name[Z_Kd], "Z_Kd" );  param.val[Z_Kd] = QUAD_DZ;

  // Throttle values
  strcpy( param.name[T_min],  "T_min"  );  param.val[T_min]  = QUAD_TMIN;
  strcpy( param.name[T_max],  "T_max"  );  param.val[T_max]  = QUAD_TMAX;
  strcpy( param.name[T_tilt], "T_tilt" );  param.val[T_tilt] = QUAD_TILT;

  // Range values
  strcpy( param.name[X_Range], "X_Range" );  param.val[X_Range] = QUAD_X_RANGE;
  strcpy( param.name[Y_Range], "Y_Range" );  param.val[Y_Range] = QUAD_Y_RANGE;
  strcpy( param.name[Z_Range], "Z_Range" );  param.val[Z_Range] = QUAD_Z_RANGE;
  strcpy( param.name[T_Range], "T_Range" );  param.val[T_Range] = QUAD_T_RANGE;

  // Assign conditional values
  gcs.sendhb      = false;
  gcs.sendparam   = false;
  gcs.sendmission = false;

  // Assign pause for serial stream
  gcs.pause = 100;

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

  static int count = 0;
  if ( count < HZ_GCSTX )  {  count++;  }
  else                     {  count = 0;  gcs.sendhb = true;  }

  // Send GCS updates
  if (gcs.sendhb)                 gcs_heartbeat();
  if (gcs.sendparam)              gcs_paramlist();
  if (gcs.sendmission)            gcs_missionlist();

  // Send input/output values
  if (GCS_INPUT_ENABLED)          gcs_input();
  if (GCS_OUTPUT_ENABLED)         gcs_output();

  // Send IMUA data
  if (IMUA_ENABLED)  {
    if (GCS_IMUA_RAW_ENABLED)     gcs_imuA_raw();
    if (GCS_IMUA_SCALED_ENABLED)  gcs_imuA_scaled();
    if (GCS_IMUA_FILTER_ENABLED)  gcs_imuA_filter();
  }

  // Send IMUB data
  if (IMUB_ENABLED)  {
    if (GCS_IMUB_RAW_ENABLED)     gcs_imuB_raw();
    if (GCS_IMUB_SCALED_ENABLED)  gcs_imuB_scaled();
    if (GCS_IMUB_FILTER_ENABLED)  gcs_imuB_filter();
  }

  // Send attitude/heading data
  //if (GCS_AHRS_EUL_ENABLED)       gcs_ahrs_eul();
  //if (GCS_AHRS_QUAT_ENABLED)      gcs_ahrs_quat();

  // Send GPS data
  //if (GCS_GPS_ENABLED)            gcs_gps();

  return;
}


/**
 *  gcs_rx
 *  Receive from the ground control station.
 */
void gcs_rx ( void)  {

  // Local variables
  mavlink_message_t msg;
  mavlink_status_t  status;
  bool moredata = true;

  // Check for available serial 
  while( moredata )  {

    uint8_t c;

    pthread_mutex_lock(&gcs.mutex);
    int r = read( gcs.fd, &c, 1 );
    pthread_mutex_unlock(&gcs.mutex);

    if ( r == 0 )  moredata = false;

    // Try to get a new message
    if ( mavlink_parse_char( 0, c, &msg, &status ) )  {

      // Debugging
      //printf( "\n  Received message with ID %d, sequence: %d from component %d of system %d \n", msg.msgid, msg.seq, msg.compid, msg.sysid );

      // Handle message
      switch(msg.msgid)  {

        // ID: #0
        case MAVLINK_MSG_ID_HEARTBEAT:
          // Reset fail safe timer
	  if (GCS_DEBUG)  { printf("(--) Heartbeat    ");  fflush(stdout); }
        break;

        // ID: #20
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	  gcs.sendparam = true;
          if (datalog.enabled)  log_record(LOG_PARAM);
	  if (GCS_DEBUG)  { printf("(20) ParamReqRead    ");  fflush(stdout); }
        break;

        // ID: #21
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
	  gcs.sendparam = true;
	  if (GCS_DEBUG)  { printf("(21) ParamReqList    ");  fflush(stdout); }
        break;

        // ID: #23
        case MAVLINK_MSG_ID_PARAM_SET:
          gcs_param_value(&msg);
          if (datalog.enabled)  log_record(LOG_PARAM);
	  if (GCS_DEBUG)  { printf("(23) ParamSet    ");  fflush(stdout); }
        break;

        // ID: #43
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	  gcs.sendmission = true;
	  if (GCS_DEBUG)  { printf("(43) MisReqList    ");  fflush(stdout); }
        break;

        // ID: #47
        case MAVLINK_MSG_ID_MISSION_ACK:
	  // Add "Mission Acknowledge" code...
	  if (GCS_DEBUG)  { printf("(47) MisAck    ");  fflush(stdout); }
        break;

        // ID: #75
        case MAVLINK_MSG_ID_COMMAND_INT:
	  // Add "Command Int" code...
	  if (GCS_DEBUG)  { printf("(75) CmdInt    ");  fflush(stdout); }
        break;

        // ID: #76
        case MAVLINK_MSG_ID_COMMAND_LONG:
	  // Add "Command Long" code...
	  if (GCS_DEBUG)  { printf("(76) CmdLong    ");  fflush(stdout); }
        break;

        // ID: #???
        default:
          // Add unknown ID code...
	  if (GCS_DEBUG)  { printf("(?\?) Unknown    ");  fflush(stdout); }
        break;

      }

      if (GCS_DEBUG)  { printf("\n");  fflush(stdout); }

    }
  }

  // Update global packet drops counter
  //packet_drops += status.packet_rx_drop_count;

  return;
}


/**
 *  gcs_heartbeat
 *  Sends a heartbeat transmission.
 */
static void gcs_heartbeat ( void )  {

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
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Send the message 
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  gcs.sendhb = false;

  return;
}


/**
 *  gcs_paramlist
 *  Sends the onboard parameter list.
 */
static void gcs_paramlist ( void )  {

  // Local variables
  int len, w, i;
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

    // Send parameter to GCS
    memset( buf, 0, sizeof(buf) );
    len = mavlink_msg_to_send_buffer( buf, &msg );

    pthread_mutex_lock(&gcs.mutex);
    w = write( gcs.fd, buf, len );
    usleep( w * gcs.pause );
    pthread_mutex_unlock(&gcs.mutex);

  }

  gcs.sendparam = false;

  return;
}


/**
 *  gcs_missionlist
 *  Sends the onboard mission list.
 */
static void gcs_missionlist ( void)  {

  // Local variables
  int len, w;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;

  // Pack parameter message
  memset( &msg, 0, sizeof(&msg) );
  mavlink_msg_mission_count_pack (
    GCS_SYSID, 
    GCS_MISSION,
    &msg,
    0, 
    0, 
    0
  );

  // Send parameter to GCS
  memset( buf, 0, sizeof(buf) );
  len = mavlink_msg_to_send_buffer( buf, &msg );

  pthread_mutex_lock(&gcs.mutex);
  w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  gcs.sendmission = false;

  return;
}


/**
 *  gcs_param_value
 *  Updates the parameter values as needed.
 */
static void gcs_param_value ( mavlink_message_t *msg )  {

  // Local variables
  uint i, j;
  bool match;
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode( msg, &set );

  // Check if this message is for this system
  if (  (uint8_t) set.target_system    == GCS_SYSID  && 
        (uint8_t) set.target_component == GCS_PARAM )  {

    char* key = (char*) set.param_id;

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

        // Only write and emit changes if there is actually a difference
        // AND only write if new value is NOT "not-a-number"
        // AND is NOT infinity

        if ( param.val[i] != set.param_value && 
             !isnan(set.param_value) && 
             !isinf(set.param_value) && set.param_type == MAVLINK_TYPE_FLOAT )  {

          param.val[i] = set.param_value;

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

  ushort x=0, y=1, z=2, t=3;

  // Update LPF cutoff frequency
  lpf_freq( &lpf_gyrA, param.val[lpf_freq_gyr] );  lpf_freq( &lpf_gyrB, param.val[lpf_freq_gyr] );
  lpf_freq( &lpf_accA, param.val[lpf_freq_acc] );  lpf_freq( &lpf_accB, param.val[lpf_freq_acc] );
  lpf_freq( &lpf_magA, param.val[lpf_freq_mag] );  lpf_freq( &lpf_magB, param.val[lpf_freq_mag] );

  // Update LPF sample history
  lpf_hist( &lpf_gyrA, param.val[lpf_hist_gyr] );  lpf_hist( &lpf_gyrB, param.val[lpf_hist_gyr] );
  lpf_hist( &lpf_accA, param.val[lpf_hist_acc] );  lpf_hist( &lpf_accB, param.val[lpf_hist_acc] );
  lpf_hist( &lpf_magA, param.val[lpf_hist_mag] );  lpf_hist( &lpf_magB, param.val[lpf_hist_mag] );

  // Update roll gains
  ctrl.pgain[x] = param.val[X_Kp];
  ctrl.igain[x] = param.val[X_Ki];
  ctrl.dgain[x] = param.val[X_Kd];

  // Update pitch gains
  ctrl.pgain[y] = param.val[Y_Kp];
  ctrl.igain[y] = param.val[Y_Ki];
  ctrl.dgain[y] = param.val[Y_Kd];

  // Update yaw gains
  ctrl.pgain[z] = param.val[Z_Kp];
  ctrl.igain[z] = param.val[Z_Ki];
  ctrl.dgain[z] = param.val[Z_Kd];

  // Update throttle values
  ctrl.thrl[0] = param.val[T_min];
  ctrl.thrl[1] = param.val[T_max];
  ctrl.thrl[2] = param.val[T_tilt];

  // Update range values
  ctrl.range[x] = param.val[X_Range];
  ctrl.range[y] = param.val[Y_Range];
  ctrl.range[z] = param.val[Z_Range];
  ctrl.range[t] = param.val[T_Range];

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
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

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
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep(w*300);
  pthread_mutex_unlock(&gcs.mutex);

  return;
}


/**
 *  gcs_imuA_raw
 *  Sends the raw IMUA data.
 */
static void gcs_imuA_raw ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&accA.mutex);
  int16_t xacc  = (int16_t) imuA.acc->raw[0];
  int16_t yacc  = (int16_t) imuA.acc->raw[1];
  int16_t zacc  = (int16_t) imuA.acc->raw[2];
  pthread_mutex_unlock(&accA.mutex);
  pthread_mutex_lock(&gyrA.mutex);
  int16_t xgyro = (int16_t) imuA.gyr->raw[0];
  int16_t ygyro = (int16_t) imuA.gyr->raw[1];
  int16_t zgyro = (int16_t) imuA.gyr->raw[2];
  pthread_mutex_unlock(&gyrA.mutex);
  pthread_mutex_lock(&magA.mutex);
  int16_t xmag  = (int16_t) imuA.mag->raw[0];
  int16_t ymag  = (int16_t) imuA.mag->raw[1];
  int16_t zmag  = (int16_t) imuA.mag->raw[2];
  pthread_mutex_unlock(&magA.mutex);

  // Pack the attitude message 
  mavlink_msg_raw_imu_pack ( 
    GCS_SYSID, GCS_IMUA_RAW, &msg, time_boot_ms, 
    xacc, yacc, zacc, 
    xgyro, ygyro, zgyro,
    xmag, ymag, zmag );

  // Copy the heartbeat message to the send buffer
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_unlock(&gcs.mutex);

  return;
}


/**
 *  gcs_imuA_scaled
 *  Sends the scaled IMUA data.
 */
static void gcs_imuA_scaled ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&accA.mutex);
  int16_t xacc  = (int16_t) ( imuA.acc->scaled[0] *1000.0 );
  int16_t yacc  = (int16_t) ( imuA.acc->scaled[1] *1000.0 );
  int16_t zacc  = (int16_t) ( imuA.acc->scaled[2] *1000.0 );
  pthread_mutex_unlock(&accA.mutex);
  pthread_mutex_lock(&gyrA.mutex);
  int16_t xgyro = (int16_t) ( imuA.gyr->scaled[0] *1000.0 );
  int16_t ygyro = (int16_t) ( imuA.gyr->scaled[1] *1000.0 );
  int16_t zgyro = (int16_t) ( imuA.gyr->scaled[2] *1000.0 );
  pthread_mutex_unlock(&gyrA.mutex);
  pthread_mutex_lock(&magA.mutex);
  int16_t xmag  = (int16_t) ( imuA.mag->scaled[0] *1000.0 );
  int16_t ymag  = (int16_t) ( imuA.mag->scaled[1] *1000.0 );
  int16_t zmag  = (int16_t) ( imuA.mag->scaled[2] *1000.0 );
  pthread_mutex_unlock(&magA.mutex);

  // Pack the attitude message 
  mavlink_msg_scaled_imu_pack ( 
    GCS_SYSID, GCS_IMUA_SCALED, &msg, time_boot_ms, 
    xacc, yacc, zacc, 
    xgyro, ygyro, zgyro,
    xmag, ymag, zmag );

  // Copy the heartbeat message to the send buffer
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_lock(&gcs.mutex);

  return;
}


/**
 *  gcs_imuA_filter
 *  Sends the filtered IMUA data.
 */
static void gcs_imuA_filter ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&accA.mutex);
  int16_t xacc  = (int16_t) ( imuA.acc->filter[0] *1000.0 );
  int16_t yacc  = (int16_t) ( imuA.acc->filter[1] *1000.0 );
  int16_t zacc  = (int16_t) ( imuA.acc->filter[2] *1000.0 );
  pthread_mutex_unlock(&accA.mutex);
  pthread_mutex_lock(&gyrA.mutex);
  int16_t xgyro = (int16_t) ( imuA.gyr->filter[0] *1000.0 );
  int16_t ygyro = (int16_t) ( imuA.gyr->filter[1] *1000.0 );
  int16_t zgyro = (int16_t) ( imuA.gyr->filter[2] *1000.0 );
  pthread_mutex_unlock(&gyrA.mutex);
  pthread_mutex_lock(&magA.mutex);
  int16_t xmag  = (int16_t) ( imuA.mag->filter[0] *1000.0 );
  int16_t ymag  = (int16_t) ( imuA.mag->filter[1] *1000.0 );
  int16_t zmag  = (int16_t) ( imuA.mag->filter[2] *1000.0 );
  pthread_mutex_unlock(&magA.mutex);

  // Pack the attitude message 
  mavlink_msg_scaled_imu_pack (
    GCS_SYSID, GCS_IMUA_FILTER, &msg, time_boot_ms, 
    xacc, yacc, zacc, 
    xgyro, ygyro, zgyro,
    xmag, ymag, zmag );

  // Copy the heartbeat message to the send buffer
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause );
  pthread_mutex_lock(&gcs.mutex);

  return;
}


/**
 *  gcs_imuB_raw
 *  Sends the raw IMUB data.
 */
static void gcs_imuB_raw ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&accB.mutex);
  int16_t xacc  = (int16_t) imuB.acc->raw[0];
  int16_t yacc  = (int16_t) imuB.acc->raw[1];
  int16_t zacc  = (int16_t) imuB.acc->raw[2];
  pthread_mutex_unlock(&accB.mutex);
  pthread_mutex_lock(&gyrB.mutex);
  int16_t xgyro = (int16_t) imuB.gyr->raw[0];
  int16_t ygyro = (int16_t) imuB.gyr->raw[1];
  int16_t zgyro = (int16_t) imuB.gyr->raw[2];
  pthread_mutex_unlock(&gyrB.mutex);
  pthread_mutex_lock(&magB.mutex);
  int16_t xmag  = (int16_t) imuB.mag->raw[0];
  int16_t ymag  = (int16_t) imuB.mag->raw[1];
  int16_t zmag  = (int16_t) imuB.mag->raw[2];
  pthread_mutex_unlock(&magB.mutex);

  // Pack the attitude message 
  mavlink_msg_raw_imu_pack ( 
    GCS_SYSID, GCS_IMUB_RAW, &msg, time_boot_ms, 
    xacc, yacc, zacc, 
    xgyro, ygyro, zgyro,
    xmag, ymag, zmag
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


/**
 *  gcs_imuB_scaled
 *  Sends the scaled IMUB data.
 */
static void gcs_imuB_scaled ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&accB.mutex);
  int16_t xacc  = (int16_t) ( imuB.acc->scaled[0] *1000.0 );
  int16_t yacc  = (int16_t) ( imuB.acc->scaled[1] *1000.0 );
  int16_t zacc  = (int16_t) ( imuB.acc->scaled[2] *1000.0 );
  pthread_mutex_unlock(&accB.mutex);
  pthread_mutex_lock(&gyrB.mutex);
  int16_t xgyro = (int16_t) ( imuB.gyr->scaled[0] *1000.0 );
  int16_t ygyro = (int16_t) ( imuB.gyr->scaled[1] *1000.0 );
  int16_t zgyro = (int16_t) ( imuB.gyr->scaled[2] *1000.0 );
  pthread_mutex_unlock(&gyrB.mutex);
  pthread_mutex_lock(&magB.mutex);
  int16_t xmag  = (int16_t) ( imuB.mag->scaled[0] *1000.0 );
  int16_t ymag  = (int16_t) ( imuB.mag->scaled[1] *1000.0 );
  int16_t zmag  = (int16_t) ( imuB.mag->scaled[2] *1000.0 );
  pthread_mutex_unlock(&magB.mutex);

  // Pack the attitude message 
  mavlink_msg_scaled_imu_pack ( 
    GCS_SYSID, GCS_IMUB_SCALED, &msg, time_boot_ms, 
    xacc, yacc, zacc, 
    xgyro, ygyro, zgyro,
    xmag, ymag, zmag
    );

  // Copy the heartbeat message to the send buffer
  uint len = mavlink_msg_to_send_buffer( buf, &msg );

  // Transmit the attitude data
  pthread_mutex_lock(&gcs.mutex);
  int w = write( gcs.fd, buf, len );
  usleep( w * gcs.pause);
  pthread_mutex_unlock(&gcs.mutex);

  return;
}


/**
 *  gcs_imuB_filter
 *  Sends the filtered IMUB data.
 */
static void gcs_imuB_filter ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&accB.mutex);
  int16_t xacc  = (int16_t) ( imuB.acc->filter[0] *1000.0 );
  int16_t yacc  = (int16_t) ( imuB.acc->filter[1] *1000.0 );
  int16_t zacc  = (int16_t) ( imuB.acc->filter[2] *1000.0 );
  pthread_mutex_unlock(&accB.mutex);
  pthread_mutex_lock(&gyrB.mutex);
  int16_t xgyro = (int16_t) ( imuB.gyr->filter[0] *1000.0 );
  int16_t ygyro = (int16_t) ( imuB.gyr->filter[1] *1000.0 );
  int16_t zgyro = (int16_t) ( imuB.gyr->filter[2] *1000.0 );
  pthread_mutex_unlock(&gyrB.mutex);
  pthread_mutex_lock(&magB.mutex);
  int16_t xmag  = (int16_t) ( imuB.mag->filter[0] *1000.0 );
  int16_t ymag  = (int16_t) ( imuB.mag->filter[1] *1000.0 );
  int16_t zmag  = (int16_t) ( imuB.mag->filter[2] *1000.0 );
  pthread_mutex_unlock(&magB.mutex);

  // Pack the attitude message 
  mavlink_msg_scaled_imu_pack ( 
    GCS_SYSID, GCS_IMUB_FILTER, &msg, time_boot_ms, 
    xacc, yacc, zacc, 
    xgyro, ygyro, zgyro,
    xmag, ymag, zmag
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


/**
 *  gcs_ahrs_eul
 *  Sends the Euler attitude representation.
 */
/*static void gcs_ahrs_eul ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&ahrs.mutex);
  float roll       = (float) ahrs.eul[0];
  float pitch      = (float) ahrs.eul[1];
  float yaw        = (float) ahrs.eul[2];
  float rollspeed  = (float) ahrs.ang[0];
  float pitchspeed = (float) ahrs.ang[1];
  float yawspeed   = (float) ahrs.ang[2];
  pthread_mutex_unlock(&ahrs.mutex);

  // Pack the attitude message 
  mavlink_msg_attitude_pack ( 
    GCS_SYSID, GCS_ATT, &msg, time_boot_ms, 
    roll, pitch, yaw, 
    rollspeed, pitchspeed, yawspeed
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

/**
 *  gcs_ahrs_quat
 *  Sends the quaternion attitude representation.
 */
/*static void gcs_ahrs_quat ( void )  {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Clear the message and buffer
  memset( &msg, 0, sizeof(&msg) );
  memset( &buf, 0, sizeof(&buf) );

  // Collect the data
  uint32_t time_boot_ms = 0;
  pthread_mutex_lock(&ahrs.mutex);
  float roll       = (float) ahrs.eul[0];
  float pitch      = (float) ahrs.eul[1];
  float yaw        = (float) ahrs.eul[2];
  float rollspeed  = (float) ahrs.deul[0];
  float pitchspeed = (float) ahrs.deul[1];
  float yawspeed   = (float) ahrs.deul[2];
  pthread_mutex_unlock(&ahrs.mutex);

  // Pack the attitude message 
  mavlink_msg_attitude_pack ( 
    GCS_SYSID, GCS_ATT, &msg, time_boot_ms, 
    roll, pitch, yaw, 
    rollspeed, pitchspeed, yawspeed
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


