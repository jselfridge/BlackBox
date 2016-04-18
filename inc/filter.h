

#ifndef FILTER_H
#define FILTER_H


#include <sys/types.h>


#define LPF_FREQ_GYR   0.0
#define LPF_FREQ_ACC   0.0
#define LPF_FREQ_MAG   0.0
#define LPF_FREQ_EUL   0.0
#define LPF_FREQ_ANG   0.0

#define LPF_HIST_GYR    1
#define LPF_HIST_ACC    1
#define LPF_HIST_MAG    1
#define LPF_HIST_EUL    1
#define LPF_HIST_ANG    1


typedef struct filter_struct {
  ushort dim;
  ushort hist;
  double dt;
  double freq;
  double gain;
  double *data;
} filter_struct;


filter_struct filter_gyrA;
filter_struct filter_accA;
filter_struct filter_magA;
filter_struct filter_gyrB;
filter_struct filter_accB;
filter_struct filter_magB;
filter_struct filter_eul;
filter_struct filter_ang;


void  filter_init  ( void );
void  filter_exit  ( void );
void  filter_freq  ( filter_struct *filter, double freq );
void  filter_hist  ( filter_struct *filter, uint hist );
void  filter_lpf   ( filter_struct *filter, double *input, double *output );


#endif



