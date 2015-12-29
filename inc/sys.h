
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Define statements
#define DEBUG           true
#define DEBUG_HZ        10
#define SYSIO_HZ        100
#define SYS_STACK       ( 100 * 1024 * 1024 )


// System structure
typedef struct sys_struct {
  bool  running;
  int   ret;
  double input[10];
  double output[10];
} sys_struct;
sys_struct sys;


// Global variables  { Move into struct? }
struct sigaction sys_signal;


// Function declarations  { Add void? }
void sys_err    ( bool cond, char* msg );
void sys_init   (  );
void sys_debug  (  );
void sys_exit   (  );
void sys_memory ( int size );


#endif



