

#ifndef GCS_H
#define GCS_H


#include <stdbool.h>
#include <common/mavlink.h>
#include <sys/types.h>


#define GCS_DEBUG        false

#define GCS_SYSID        01

#define GCS_HEARTBEAT    01
#define GCS_PARAM        02
#define GCS_MISSION      03

//#define GCS_IMUA_RAW     11
//#define GCS_IMUA_SCALED  12
//#define GCS_IMUA_FILTER  13

//#define GCS_IMUB_RAW     21
//#define GCS_IMUB_SCALED  22
//#define GCS_IMUB_FILTER  23

//#define GCS_IMU          30
#define GCS_ATT          40
#define GCS_INPUT        50
#define GCS_OUTPUT       60
//#define GCS_GPS          70
//#define GCS_GAINS       120

#define GCS_INPUT_ENABLED        true
#define GCS_OUTPUT_ENABLED       true
#define GCS_ATT_ENABLED          true
//#define GCS_IMUA_RAW_ENABLED     false
//#define GCS_IMUA_SCALED_ENABLED  true
//#define GCS_IMUA_FILTER_ENABLED  true
//#define GCS_IMUB_RAW_ENABLED     false
//#define GCS_IMUB_SCALED_ENABLED  true
//#define GCS_IMUB_FILTER_ENABLED  true
//#define GCS_COMP_ENABLED         true
//#define GCS_AHRS_EUL_ENABLED     false
//#define GCS_AHRS_QUAT_ENABLED    false
//#define GCS_GPS_ENABLED          false


enum param_index {

  // Roll stabilization
  X_ts = 0,
  X_mp,
  X_b,

  // Pitch stabilization
  Y_ts,
  Y_mp,
  Y_b, 

  // Yaw stabilization
  Z_ts,
  Z_mp,
  Z_b, 

  // Thrl values
  T_min,
  T_max, 
  T_tilt,

  // Range values
  X_Range,
  Y_Range,
  Z_Range,
  T_Range,

  // Number of elements
  param_count

};


typedef struct param_struct {
  float val  [param_count];
  char  name [param_count][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
} param_struct;
param_struct param;


typedef struct gcs_struct {
  int     fd;
  char    path [16];
  bool    sendhb;
  bool    sendparam;
  bool    sendmission;
  uint    pause;
  pthread_mutex_t mutex;
} gcs_struct;
gcs_struct gcs;


void  gcs_init         ( void );
void  gcs_exit         ( void );
void  gcs_tx           ( void );
void  gcs_rx           ( void );


#endif



