
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Define statements
#define DEBUG           true
#define NSEC_PER_SEC    1000000000u
#define SYS_LOOP        10000000u
#define SYS_FREQ        (double)( NSEC_PER_SEC / SYS_LOOP )
#define SYS_DT          (double)( 1/SYS_FREQ )


// System structure
typedef struct {
  double input[10];
  double output[10];
  bool  running;
} sys_struct;
sys_struct sys;


// Global variables
struct sigaction sys_signal;


// Function declarations
void sys_err    ( bool cond, char* msg );
void sys_init   (  );
void sys_loop   (  );
void sys_debug  (  );
void sys_exit   (  );
void sys_memory ( void );


#endif



