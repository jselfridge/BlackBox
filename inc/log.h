
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Log enumerations
enum log_index {
  LOG_NOTE = 0,
  LOG_GYR  = 1,
  LOG_ACC  = 2,
  LOG_MAG  = 3,
  LOG_FUS  = 4,
  LOG_SIO  = 5,
  LOG_CTRL = 6
} log_index;


// Log structure
typedef struct {
  FILE* note;
  FILE* gyr;
  FILE* acc;
  FILE* mag;
  FILE* fus;
  FILE* sio;
  FILE* stab;
  FILE* nav;
  char* dir;
  char* path;
  bool  open;
  bool  enabled;
  long  offset;
} log_struct;
log_struct datalog; 


// Log functions
void  log_init   ( void );
void  log_write  ( enum log_index index );
void  log_open   ( void );
void  log_record ( enum log_index index );
void  log_exit   ( void );


#endif



