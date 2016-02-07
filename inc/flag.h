
//============================================================
//  flag.h
//  Justin M Selfridge
//============================================================
#ifndef _FLAG_H_
#define _FLAG_H_
#include <main.h>


// Define stick hold durations
#define FLG_HOLD0  3.0
#define FLG_HOLD1  3.0
#define FLG_HOLD2  3.0
#define FLG_HOLD3  0.0
#define FLG_HOLD4  0.0
#define FLG_HOLD5  0.0
#define FLG_HOLD6  0.0
#define FLG_HOLD7  0.0
#define FLG_HOLD8  0.0
#define FLG_HOLD9  0.0


// Program execution flag structure
typedef struct {
  ushort upper[10];
  ushort lower[10];
  ushort limit[10];
} flag_struct;
flag_struct flag;


// Program execution flag functions
void  flg_init   ( void );
void  flg_exit   ( void );
void  flg_update ( void );


#endif



