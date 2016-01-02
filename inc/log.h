
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Log enumerations
enum log_index {
  LOG_GYR    = 0,
  LOG_ACC    = 1,
  LOG_MAG    = 2,
  LOG_FUSION = 3,
  LOG_SYSIO  = 4,
  LOG_PARAM  = 5
} log_index;


// Log structure
typedef struct {
  FILE* note;
  FILE* gyr;
  FILE* acc;
  FILE* mag;
  FILE* fusion;
  FILE* sysio;
  FILE* param;
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



