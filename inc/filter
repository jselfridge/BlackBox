

#ifndef _FILTER_H_
#define _FILTER_H_
#include <main.h>


// Define low pass filter values
//#define LPF_GYR   20.0
//#define LPF_ACC   20.0
//#define LPF_MAG    2.0
//#define LPF_EUL    0.0
//#define LPF_ANG    0.0

// Global variables for LPF cutoff freq (move to struct???)
#define lpf_hz_gyr (20.0)
#define lpf_hz_acc (20.0)
#define lpf_hz_mag  (2.0)


// Define sample history duration
#define HIST_GYR   2
#define HIST_ACC   2
#define HIST_MAG   2
#define HIST_EUL   1
#define HIST_ANG   1


// Global storage arrays
double filter_gyrA [3][HIST_GYR];
double filter_accA [3][HIST_ACC];
double filter_magA [3][HIST_MAG];
double filter_gyrB [3][HIST_GYR];
double filter_accB [3][HIST_ACC];
double filter_magB [3][HIST_MAG];
double filter_eul  [3][HIST_EUL];
double filter_ang  [3][HIST_ANG];


// Filter functions
void    filter_init    ( void );
void    filter_exit    ( void );
double  filter_lpf     ( double *data, double sample, double gain, ushort hist );
double  filter_gains   ( double hz );


#endif



