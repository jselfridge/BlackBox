
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Log structure
typedef struct {
  FILE *note;
  FILE *acc;
  FILE *gyro;
  FILE *mag;
  //char *name;
  char *dir;
  char *path;
  bool  open;
  bool  enabled;
  long  offset;
} log_struct;
log_struct datalog; 


// Log functions
void  log_write  ( void );
void  log_init   ( void );
void  log_record ( void );
void  log_exit   ( void );


#endif



