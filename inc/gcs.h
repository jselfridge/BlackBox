

#ifndef GCS_H
#define GCS_H


#include <stdbool.h>
#include <common/mavlink.h>


#define GCS_PARAM_COUNT   22

#define GCS_SYSID        01

#define GCS_IMUA_RAW     11
#define GCS_IMUA_SCALED  12
#define GCS_IMUA_FILTER  13

#define GCS_IMUB_RAW     21
#define GCS_IMUB_SCALED  22
#define GCS_IMUB_FILTER  23

#define GCS_IMU          30
#define GCS_ATT          40
#define GCS_INPUT        50
#define GCS_OUTPUT       60
#define GCS_GPS          70
#define GCS_GAINS       120

#define GCS_INPUT_ENABLED        false
#define GCS_OUTPUT_ENABLED       false
#define GCS_IMUA_RAW_ENABLED     true
#define GCS_IMUA_SCALED_ENABLED  true
#define GCS_IMUA_FILTER_ENABLED  true
#define GCS_IMUB_RAW_ENABLED     true
#define GCS_IMUB_SCALED_ENABLED  true
#define GCS_IMUB_FILTER_ENABLED  true
#define GCS_AHRS_EUL_ENABLED     false
#define GCS_AHRS_QUAT_ENABLED    false
#define GCS_GPS_ENABLED          false


enum param_index {

  // LPF cutoff freq
  lpf_hz_gyr = 0,
  lpf_hz_acc,
  lpf_hz_mag,

  // LPF sample history
  lpf_hist_gyr,
  lpf_hist_acc,
  lpf_hist_mag

  /*  // Roll gains
  X_Kp,
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
  */

};


typedef struct param_struct {
  float val  [GCS_PARAM_COUNT];
  char  name [GCS_PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
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


void  gcs_init         ( void );
void  gcs_exit         ( void );
void  gcs_tx           ( void );
void  gcs_rx           ( void );

//void  gcs_send_param   ( enum param_index name, float val );
//void  gcs_paramupdate  ( mavlink_message_t *msg );



#endif



