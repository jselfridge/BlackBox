

#ifndef LPF_H
#define LPF_H


#include <sys/types.h>


// Move to a file
#define LPF_FREQ_GYR  3.0
#define LPF_FREQ_ACC  1.0
#define LPF_FREQ_MAG  1.0
#define LPF_FREQ_EUL  0.0
#define LPF_FREQ_ANG  0.0

// Move to a file
#define LPF_HIST_GYR  100
#define LPF_HIST_ACC  300
#define LPF_HIST_MAG  300
#define LPF_HIST_EUL    1
#define LPF_HIST_ANG    1


typedef struct lpf_struct {
  ushort dim;
  ushort hist;
  double dt;
  double freq;
  double gain;
  double *data;
} lpf_struct;


//lpf_struct filter_gyr;
//lpf_struct filter_acc;
//lpf_struct filter_mag;
//lpf_struct filter_eul;
//lpf_struct filter_ang;
//lpf_struct filter_gyrA;
//lpf_struct filter_accA;
//lpf_struct filter_magA;
//lpf_struct filter_eulA;
//lpf_struct filter_angA;
//lpf_struct filter_gyrB;
//lpf_struct filter_accB;
//lpf_struct filter_magB;
//lpf_struct filter_eulB;
//lpf_struct filter_angB;


void  lpf_init    ( void );
void  lpf_exit    ( void );
void  lpf_freq    ( lpf_struct *lpf, double freq );
void  lpf_hist    ( lpf_struct *lpf, uint hist );
void  lpf_update  ( lpf_struct *lpf, double *input, double *output );


#endif



