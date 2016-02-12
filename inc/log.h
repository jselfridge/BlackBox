
//============================================================
//  log.h
//  Justin M Selfridge
//============================================================
#ifndef _LOG_H_
#define _LOG_H_
#include <main.h>


// Define statements
#define MAX_LOG_DUR  (20*60)


// Log enumerations
enum log_index {
  LOG_SIO  = 0 //,
  //LOG_IMU  = 1,
  //LOG_AHR  = 2,
  //LOG_CTL  = 3
} log_index;


/*// Log IMU structure
typedef struct log_imu_struct {
  ulong count;
  ulong limit;
  float *time;
  ulong *dur;
  short *raw;
  float *avg;
  float *cal;
} log_imu_struct;
log_imu_struct log_gyr;
log_imu_struct log_acc;
log_imu_struct log_mag;
*/

/*// Log AHR structure
typedef struct log_ahr_struct {
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
} log_ahr_struct;
log_ahr_struct log_ahr;
*/

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


/*// Log controller structure
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
*/

// Log structure
typedef struct datalog_struct {
  bool   enabled;
  bool   setup;
  bool   saving;
  float  offset;
  char   *dir;
  char   *path;
} datalog_struct;
datalog_struct datalog;


// Log functions
void  log_init   ( void );
void  log_open   ( void );
void  log_close  ( void );
void  log_exit   ( void );
void  log_record ( enum log_index index );


#endif



