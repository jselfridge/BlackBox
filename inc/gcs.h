

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

#define GCS_INPUT        40
#define GCS_OUTPUT       50
#define GCS_ATT          60

#define GCS_INPUT_ENABLED     true
#define GCS_OUTPUT_ENABLED    true
#define GCS_ATT_ENABLED       true


enum param_index {

  // Roll stabilization
  X_kp = 0,
  X_kd, /*
  X_ts,
  X_mp,
  X_j,
  X_Gp,
  X_Gd,
  X_Gu,*/

  // Pitch stabilization
  Y_kp,
  Y_kd, /*
  Y_ts,
  Y_mp,
  Y_j, 
  Y_Gp,
  Y_Gd,
  Y_Gu,*/

  // Yaw stabilization
  Z_kp,
  Z_kd, /*
  Z_ts,
  Z_mp,
  Z_j,
  Z_Gp,
  Z_Gd,
  Z_Gu,*/

  // Throttle values
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



