
BLACK BOX
=========

Develops a cape for a BeagleBone Black to enable "Black 
Box" functionality for research and development. 


Current tasks
-------------
<ul>
  <li> (WIP) Add new timing thread for AHRS </li>
  <li> (WIP) Add AHRS data structure </li> 
  <li> (WIP) Add 9DOF data fusion algorithm </li>
  <li> (WIP) Enable AHRS datalogging </li>
  <li> (WIP) Calibrate IMU/AHRS settings <li>
  <li> (WIP) Evaluate LPF algorithm and settings </li>
  <li> (WIP) Check loop durations and CPU usage </li>
</ul>


V0.0.3
------
<ul>
  <li> New timing thread for program execution flags </li>
  <li> Register stick endpoint positions </li>
  <li> Counter records hold duration </li> 
  <li> Reset counters once stick condition is released </li>
  <li> Trips boolean flags for stick positions </li> 
  <li> Enable/Disable datalog </li>
  <li> Disable IMU and other loops during data download </li>
  <li> Arm/Disarm system outputs </li>
  <li> Exit program through radio commands </li>
</ul>


V0.0.2
------
<ul>
  <li> New timing thread for I/O signals </li>
  <li> Developed data structure for I/O signals </li>
  <li> Included datalog system I/O signals </li>
  <li> Configured system for PRU capabilities </li>
  <li> Enabled PRU-based PWM inputs </li>
  <li> Enabled PRU-based PWM outputs </li>
  <li> Added mutex conditions </li>
  <li> Enabled I2C in DTO </li>
  <li> Confirmed signals on scope </li>
  <li> Adjusted PWM loop value to achieve 400Hz </li>
</ul>


V0.0.1
------
<ul>
  <li> Linux beaglebone 4.1.15-ti-rt-r41 SMP PREEMPT RT armv7l </li>
  <li> Configured high speed 400kHz I2C timer </li>
  <li> Program exit condition </li>
  <li> Establish real time priority </li>
  <li> Lock and reserve memeory </li>
  <li> Setup multithread for 'debug' and 'imu' systems </li> 
  <li> Established mutex conditions for data protection </li>
  <li> Enable single IMU sensor </li>
  <li> Read calibration data from file </li>
  <li> Read gyr/acc at 1000hz, and mag at 100 hz </li>
  <li> Low pass filter on IMU data </li>
  <li> Scale raw IMU data to physical values </li> 
  <li> Setup crude initial datalogging system </li>
  <li> GPIO functions to toggle pins </li>
  <li> LED functions to manipulate user LEDs </li>
</ul>


Pending
-------
The following list outlines proposed upcoming changes 
<ul>

  <li><b> Priority </b></li>
  <li> Reestablish controller code </li>
  <li> Transistor power distribution kill switch </li> 
  <li> Add health/status monitoring thread </li>
  <li> Mavlink serial output </li>
  <li> Develop GCS </li>
  <li> Add health/status monitoring thread </li>
  <li> System identification </li>
  <li> State feedback controller </li>
  <li> VICON data comparision </li>
  <li> Kalman filter </li>

  <li><b> Sooner than later </b></li>
  <li> Butterworth filter </li>
  <li> Test pull up resistors on Sparkfun breakout board </li>
  <li> IMU interrupt when data is available </li>
  <li> Add notes section to data log </li>
  <li> Add warning for missed timing on thread </li>
  <li> Enable address 0x69 in IMU code </li>
  <li> Second IMU sensor on second I2C channel </li>
  <li> Second IMU sensor on single I2C channel </li>
  <li> Incorporate altitude hold </li> 
  <li> Add altitude sensor </li>
  <li> Disable clkout2 to enable CH5 input </li>

  <li><b> Maybe next board </b></li>
  <li> IMU convergence on start up </li>
  <li> IMU indicator for successful convergence </li>
  <li> Calibrated radio ranges </li>
  <li> Define statements to enable specific datalogs </li>
  <li> Log generation with a function </li>

</ul>




