
//============================================================
//  gcs.h
//  Justin M Selfridge
//============================================================
#ifndef _GCS_H_
#define _GCS_H_
#include <main.h>


// Define statements
//#define GPS_DEFAULT      "$PMTK104*37"



// TEMP GLOBAL VARIABLES... PUT THEM IN STRUCT ASAP...
//static int packet_drops = 0;
//static int mode = MAV_MODE_UNINIT;    // Defined in mavlink_types.h, which is included by mavlink.h 

//bool gcs_sendparam;


// GCS structure
typedef struct gcs_struct {
  int     fd;
  char    path [16];
  bool    sendparam;
  bool    sendmission;
  //mavlink_message_t   msg  [96];
  //  mavlink_system_t system;
} gcs_struct;
gcs_struct gcs;


// GPS functions
void  gcs_init  ( void );
void  gcs_exit  ( void );
void  gcs_tx    ( void );
void  gcs_rx    ( void );
void  gcs_heartbeat  ( void );
void  gcs_paramlist  ( void );
void  gcs_missionlist  ( void );


#endif



