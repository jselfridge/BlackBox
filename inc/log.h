

#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Define statements
#define MAX_LOG_DUR  (20*60)


// Log enumerations
enum log_index {
  LOG_SIO  = 0,
  LOG_IMUA = 1,
  LOG_IMUB = 2,
  LOG_AHRS = 3,
  LOG_GPS  = 4,
  LOG_CTRL = 5
} log_index;


// Log system input/output structure
typedef struct log_sio_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ushort *reg;
  ushort *pwm;
  float  *norm;
} log_sio_struct;
log_sio_struct log_input;
log_sio_struct log_output;


// Log IMU structure
typedef struct log_imu_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  short *raw;
  float *scaled;
  float *filter;
} log_imu_struct;
log_imu_struct log_gyrA;
log_imu_struct log_accA;
log_imu_struct log_magA;
log_imu_struct log_gyrB;
log_imu_struct log_accB;
log_imu_struct log_magB;


// Log AHRS structure
typedef struct log_ahrs_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  float *quat;
  float *dquat;
  float *eul;
  float *deul;
  float *bias;
  float *fx;
  float *fz;
} log_ahrs_struct;
log_ahrs_struct log_ahrs;


// Log GPS structure
typedef struct log_gps_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  char  *msg;
} log_gps_struct;
log_gps_struct log_gps;


// Log controller structure
typedef struct log_ctrl_struct {
  ulong  count;
  ulong  limit;
  float  *time;
  ulong  *dur;
  float  *perr;
  float  *ierr;
  float  *derr;
  float  *cmd;
} log_ctrl_struct;
log_ctrl_struct log_ctrl;


// Log structure
typedef struct datalog_struct {
  bool   enabled;
  bool   setup;
  bool   saving;
  float  offset;
  char   *dir;
  char   *path;
  //FILE *fnote, *fin, *fout, *fgyrA, *faccA, *fmagA, *fgyrB, *faccB, *fmagB, *fahrs, *fgps, *fctrl;
} datalog_struct;
datalog_struct datalog;


// Log functions
void  log_init   ( void );
void  log_open   ( void );
void  log_close  ( void );
void  log_exit   ( void );
void  log_record ( enum log_index index );
void  log_free   ( void );


#endif



