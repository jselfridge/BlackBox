
//============================================================
//  gcs.h
//  Justin M Selfridge
//============================================================
#ifndef _GCS_H_
#define _GCS_H_
#include <main.h>


// Define other packets
//#define GPS_DEFAULT      "$PMTK104*37"



// TEMP GLOBAL VARIABLES... PUT THEM IN STRUCT ASAP...
static int packet_drops = 0;
static int mode = MAV_MODE_UNINIT;    // Defined in mavlink_types.h, which is included by mavlink.h 


// GCS structure
typedef struct gcs_struct {
  int     fd;
  char    path [16];
  char    msg  [96];
} gcs_struct;
gcs_struct gcs;


// GPS functions
void  gcs_init  ( void );
void  gcs_exit  ( void );
void  gcs_tx    ( void );
void  gcs_rx    ( void );


#endif



