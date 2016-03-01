
//============================================================
//  gcs.h
//  Justin M Selfridge
//============================================================
#ifndef _GCS_H_
#define _GCS_H_
#include <main.h>
#include <common/mavlink.h>


// Define statements
#define MAV_COMP_ID_GAINS    120
#define PARAM_COUNT   3
#define VEHICLE_ID    20


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
void  gcs_paramupdate  ( mavlink_message_t *msg );
void  gcs_missionlist  ( void );


#endif



