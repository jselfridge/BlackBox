
//============================================================
//  flag.h
//  Justin M Selfridge
//============================================================
#ifndef _FLAG_H_
#define _FLAG_H_
#include <main.h>



//#define STICK_HOLD  3.0
#define FLG_HOLD0  3.0
#define FLG_HOLD1  3.0
#define FLG_HOLD2  3.0
#define FLG_HOLD3  0.0
#define FLG_HOLD4  1.46
#define FLG_HOLD5  0.0
#define FLG_HOLD6  0.0
#define FLG_HOLD7  0.0
#define FLG_HOLD8  0.0
#define FLG_HOLD9  0.0


// Define radio channel index
//#define CH_1    0
//#define CH_2    1
//#define CH_3    2
//#define CH_4    3
//#define CH_5    4
//#define CH_6    5
//#define CH_7    6
//#define CH_8    7


// Define state index
//#define CH_R    0    //  Roll
//#define CH_P    1    //  Pitch
//#define CH_Y    2    //  Yaw
//#define CH_T    3    //  Throttle
//#define CH_S    4    //  Switch
//#define CH_D    5    //  Dial
//#define CH_A    6    //  Auxiliary


// Full scale ranges
//#define MIN   0
//#define MAX   1
//#define LEFT  0
//#define RIGHT 1
//#define DOWN  0
//#define UP    1


// Program execution flag structure
typedef struct {
  ushort upper[10];
  ushort lower[10];
  ushort limit[10];
} flag_struct;
flag_struct flag;


// Program execution flag functions
void  flg_init  ( void );
void  flg_exit  ( void );

// Debugging function
void  flg_debug ( void );


#endif



