
//============================================================
//  sys.h
//  Justin M Selfridge
//============================================================
#ifndef _SYS_H_
#define _SYS_H_
#include <main.h>


// Define statements
#define DEBUG           true
#define SYS_STACK       ( 100 * 1024 * 1024 )


// System structure
typedef struct sys_struct {
  bool  running;
  int   ret;
  int   input[10];
  int   output[10];
} sys_struct;
//sys_struct sys;


// Global variables
struct sigaction sys_signal;


// Function declarations  { Add void? }
void sys_init   ( sys_struct *sys );
void sys_debug  ( void );
void sys_exit   (  );
void sys_mem    ( sys_struct *sys );


#endif



