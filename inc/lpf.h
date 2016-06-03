

#ifndef LPF_H
#define LPF_H


#include <sys/types.h>


// Move to a file
#define LPF_FREQ_GYR  3.0
#define LPF_FREQ_ACC  1.0
#define LPF_FREQ_MAG  1.0

// Move to a file
#define LPF_HIST_GYR  100
#define LPF_HIST_ACC  300
#define LPF_HIST_MAG  300


typedef struct lpf_struct {
  ushort dim;
  ushort hist;
  double dt;
  double freq;
  double gain;
  double *data;
} lpf_struct;
lpf_struct lpf_gyrA;
lpf_struct lpf_accA;
lpf_struct lpf_magA;
lpf_struct lpf_gyrB;
lpf_struct lpf_accB;
lpf_struct lpf_magB;


void  lpf_init    ( void );
void  lpf_exit    ( void );
void  lpf_freq    ( lpf_struct *lpf, double freq );
void  lpf_hist    ( lpf_struct *lpf, uint hist );
void  lpf_update  ( lpf_struct *lpf, double *input, double *output );


#endif



