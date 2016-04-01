

#ifndef _GCS_H_
#define _GCS_H_
#include <main.h>
#include <common/mavlink.h>


// Define statements
#define MAV_COMP_ID_GAINS    120
#define PARAM_COUNT   16


// Define 
#define GCS_SYSID  01
#define GCS_IMU    10
#define GCS_IMUA   11
#define GCS_IMUB   12
#define GCS_ATT    20
#define GCS_INPUT  30


// Define data transmissions
#define GCS_RAW_IMUA_ENABLED   true
#define GCS_RAW_IMUB_ENABLED   true
#define GCS_EUL_ENABLED        true
#define GCS_RADIO_ENABLED      true


// Parameter enumeration
enum param_index {

  // Roll gains
  X_Kp = 0,
  X_Ki,
  X_Kd,

  // Pitch gains
  Y_Kp,
  Y_Ki,
  Y_Kd,

  // Yaw gains
  Z_Kp,
  Z_Ki,
  Z_Kd,

  // Thrl values
  T_min,
  T_max, 
  T_tilt,

  // Range values
  X_R,
  Y_R,
  Z_R,
  T_R

};


// Parameter structure
typedef struct param_struct {
  float val  [PARAM_COUNT];
  char  name [PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
} param_struct;
param_struct param;


// GCS structure
typedef struct gcs_struct {
  int     fd;
  char    path [16];
  bool    sendhb;
  bool    sendparam;
  bool    sendmission;
} gcs_struct;
gcs_struct gcs;


// GCS functions
void  gcs_init         ( void );
void  gcs_exit         ( void );
void  gcs_tx           ( void );
void  gcs_rx           ( void );
void  gcs_heartbeat    ( void );
void  gcs_paramlist    ( void );
void  gcs_send_param   ( enum param_index name, float val );
void  gcs_paramupdate  ( mavlink_message_t *msg );
void  gcs_missionlist  ( void );

void  gcs_raw_imuA     ( void );
void  gcs_raw_imuB     ( void );
void  gcs_eul          ( void );
void  gcs_radio        ( void );


#endif



