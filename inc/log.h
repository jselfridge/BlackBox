
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Log structure
typedef struct {
  FILE* file;
  char* name;
  bool  open;
  bool  enabled;
} log_struct;
log_struct datalog; 


// Log functions
void  log_write  ( void );
void  log_init   ( void );
void  log_exit   ( void );
void  log_record ( void );


#endif



