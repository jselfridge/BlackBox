
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Log enumerations
enum log_index {
  //LOG_GYRO = 0,
  //LOG_ACC  = 1,
  //LOG_MAG  = 2
  LOG_RAW  = 0
} log_index;


// Log structure
typedef struct {
  FILE* note;
  FILE* raw;
  //FILE* acc;
  //FILE* gyro;
  //FILE* mag;
  //char* name;
  char* dir;
  char* path;
  bool  open;
  bool  enabled;
  long  offset;
} log_struct;
log_struct datalog; 


// Log functions
void  log_write  ( enum log_index index );
void  log_init   ( void );
void  log_record ( enum log_index index );
void  log_exit   ( void );


#endif



