

#ifndef _FILTER_H_
#define _FILTER_H_
#include <main.h>


// Define low pass filter values
#define LPF_GYR   20.0
#define LPF_ACC   20.0
#define LPF_MAG    2.0
#define LPF_EUL    0.0
#define LPF_ANG    0.0


// Define sample history duration
#define HIST_GYR   2
#define HIST_ACC   2
#define HIST_MAG   2
#define HIST_EUL   1
#define HIST_ANG   1


// Global storage arrays
double gyrA_data [3][HIST_GYR];
double accA_data [3][HIST_ACC];
double magA_data [3][HIST_MAG];
double gyrB_data [3][HIST_GYR];
double accB_data [3][HIST_ACC];
double magB_data [3][HIST_MAG];


// Filter functions
void    filter_init    ( void );
void    filter_exit    ( void );
double  filter_lpf     ( double val, double gain, double *data, ushort hist );


#endif



